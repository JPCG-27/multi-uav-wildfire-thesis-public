"""Create an immutable YOLO training experiment and optionally run it.

Training is restricted to train/validation splits. The test split is deliberately
never passed to Ultralytics during tuning. Pipeline smoke runs are explicitly
marked non-study and cannot change a dataset's training_ready decision.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import random
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# CUBLAS reads this during CUDA initialization, so set it before optional
# provenance imports load torch later in the process.
os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"

SPLITS = ("train", "val", "test")
DEFAULTS = {
    "epochs": 100,
    "patience": 20,
    "imgsz": 640,
    "batch": 8,
    "optimizer": "AdamW",
    "lr0": 0.001,
    "seed": 11,
    "deterministic": True,
    "workers": 0,
    "amp": False,
    # Keep the essential results.csv/checkpoints/args.yaml while avoiding
    # diagnostic plot images and per-epoch console chatter.
    "plots": False,
    "verbose": False,
}


def canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8-sig"),
                      parse_constant=lambda token: (_ for _ in ()).throw(ValueError(f"Invalid JSON number: {token}")))


def _contained(root: Path, relative: str) -> Path:
    rel = Path(relative)
    candidate = (root / rel).resolve()
    if rel.is_absolute() or not candidate.is_relative_to(root.resolve()):
        raise ValueError(f"Dataset path escapes root: {relative}")
    return candidate


def _dataset_yaml(root: Path) -> str:
    return (f"path: {json.dumps(root.as_posix())}\n"
            "train: images/train\nval: images/val\ntest: images/test\nnames:\n  0: person\n")


def _training_yaml(root: Path) -> str:
    return (f"path: {json.dumps(root.as_posix())}\n"
            f"train: {json.dumps((root / 'images' / 'train').as_posix())}\n"
            f"val: {json.dumps((root / 'images' / 'val').as_posix())}\n"
            "names:\n  0: person\n")


def verify_dataset(dataset: Path, *, pipeline_smoke: bool = False, audit_path: Path | None = None,
                   integrity_only: bool = False) -> dict:
    """Verify dataset-level hashes and every manifest image/label before training."""
    root = dataset.resolve(strict=True)
    summary_path = root / "dataset.json"
    summary = _read_json(summary_path)
    if summary.get("schema_version") != "sar-dataset-v1" or summary.get("complete") is not True:
        raise ValueError("Incomplete or unsupported YOLO dataset")
    if (summary.get("training_ready") is not True and not pipeline_smoke and
            audit_path is None and not integrity_only):
        raise ValueError("Dataset is not training_ready; supply a valid --audit approval or use --pipeline-smoke")

    manifest_path = root / "manifest.jsonl"
    policy_path = root / "label_policy.json"
    splits_path = root / "split_manifest.json"
    if hashlib.sha256(manifest_path.read_bytes()).hexdigest() != summary.get("manifest_sha256"):
        raise ValueError("Dataset manifest hash mismatch")
    if sha256_file(policy_path) != summary.get("label_policy_sha256"):
        raise ValueError("Label policy hash mismatch")
    if sha256_file(splits_path) != summary.get("split_manifest_sha256"):
        raise ValueError("Split manifest hash mismatch")

    split_doc = _read_json(splits_path)
    if split_doc.get("schema_version") != "sar-splits-v1" or split_doc.get("unit") != "scenario_family":
        raise ValueError("Unsupported scenario split manifest")
    assignments = split_doc.get("assignments")
    if not isinstance(assignments, dict) or not assignments or any(v not in SPLITS for v in assignments.values()):
        raise ValueError("Invalid split assignments")

    records = []
    for line_number, line in enumerate(manifest_path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            raise ValueError(f"Blank manifest line {line_number}")
        row = json.loads(line)
        if row.get("split") not in SPLITS:
            raise ValueError(f"Invalid sample split on manifest line {line_number}")
        if assignments.get(row.get("scenario_family_id")) != row["split"]:
            raise ValueError(f"Sample split disagrees with frozen family split: {row.get('sample_id')}")
        image = _contained(root, row["image_path"])
        label = _contained(root, f"labels/{row['split']}/{row['sample_id']}.txt")
        if not image.is_file() or not label.is_file():
            raise ValueError(f"Missing image or label for sample {row.get('sample_id')}")
        if sha256_file(image) != row.get("image_sha256"):
            raise ValueError(f"Image hash mismatch: {row.get('sample_id')}")
        if sha256_file(label) != row.get("label_sha256"):
            raise ValueError(f"Label hash mismatch: {row.get('sample_id')}")
        records.append(row)
    if not records:
        raise ValueError("Dataset manifest is empty")
    if pipeline_smoke and len(records) > 4:
        raise ValueError("--pipeline-smoke is restricted to the existing maximum four-image fixture")
    sample_ids = {row.get("sample_id") for row in records}
    if len(sample_ids) != len(records) or None in sample_ids:
        raise ValueError("Manifest sample IDs are missing or duplicated")
    counts = {split: sum(row["split"] == split for row in records) for split in SPLITS}
    if counts != summary.get("counts") or any(counts[s] == 0 for s in SPLITS):
        raise ValueError("Manifest split counts mismatch or a split is empty")

    expected_images = {row["image_path"].replace("\\", "/") for row in records}
    expected_labels = {f"labels/{row['split']}/{row['sample_id']}.txt" for row in records}
    for split in SPLITS:
        actual_images = {p.relative_to(root).as_posix() for p in (root / "images" / split).glob("*") if p.is_file()}
        actual_labels = {p.relative_to(root).as_posix() for p in (root / "labels" / split).glob("*") if p.is_file()}
        if actual_images != {p for p in expected_images if p.startswith(f"images/{split}/")}:
            raise ValueError(f"Image inventory differs from manifest in {split}")
        if actual_labels != {p for p in expected_labels if p.startswith(f"labels/{split}/")}:
            raise ValueError(f"Label inventory differs from manifest in {split}")

    if (root / "data.yaml").read_text(encoding="utf-8") != _dataset_yaml(root):
        raise ValueError("data.yaml differs from the canonical immutable dataset configuration")
    audit = None
    if audit_path is not None:
        audit = _read_json(audit_path.resolve(strict=True))
        if (audit.get("approved_for_training") is not True or
                audit.get("dataset_manifest_sha256") != summary["manifest_sha256"] or
                audit.get("label_policy_sha256") != summary["label_policy_sha256"] or
                audit.get("split_manifest_sha256") != summary["split_manifest_sha256"]):
            raise ValueError("Label audit is unapproved or its dataset/policy/split hashes do not match")
        if audit.get("schema_version") == "sar-label-audit-v1":
            reviewed = audit.get("reviewed_sample_ids")
            if (not isinstance(reviewed, list) or
                    any(not isinstance(sample_id, str) for sample_id in reviewed) or
                    len(set(reviewed)) != len(reviewed) or len(reviewed) < 200 or
                    not set(reviewed).issubset(sample_ids) or
                    isinstance(audit.get("reviewed_count"), bool) or
                    audit.get("reviewed_count") != len(reviewed) or audit.get("reviewed_count", 0) < 200):
                raise ValueError("Manual label audit reviews fewer than 200 unique valid dataset samples")
            if (not isinstance(audit.get("reviewer_type"), str) or not audit["reviewer_type"].strip() or
                    not isinstance(audit.get("decision_notes"), str) or not audit["decision_notes"].strip()):
                raise ValueError("Manual label audit must identify reviewer_type and include decision_notes")
        elif audit.get("schema_version") == "sar-machine-label-audit-v1":
            expected_samples = len(records)
            checked = audit.get("checked", {})
            errors = audit.get("counts", {})
            check_keys = ("package_integrity", "mask_label_mismatches", "visibility_safeguard_errors",
                          "split_leakage_errors", "near_duplicate_candidates", "total")
            current_tool = Path(__file__).with_name("audit_yolo_dataset.py")
            if (audit.get("audit_kind") != "automated_ground_truth_mask_validation" or
                    audit.get("automated_checks_complete") is not True or
                    audit.get("human_review_performed") is not False or
                    audit.get("reviewer_type") != "automated_machine_validation" or
                    any(key in audit for key in ("reviewed_count", "reviewed_sample_ids", "human_review_count")) or
                    not isinstance(checked, dict) or not isinstance(errors, dict) or
                    any(isinstance(checked.get(key), bool) or not isinstance(checked.get(key), int) or
                        checked.get(key) != expected_samples for key in ("samples", "images", "masks", "labels")) or
                    any(isinstance(errors.get(key), bool) or errors.get(key) != 0 for key in check_keys) or
                    audit.get("near_duplicate_audit_truncated") is not False or
                    audit.get("near_duplicate_candidates") != [] or
                    not current_tool.is_file() or audit.get("audit_tool_sha256") != sha256_file(current_tool)):
                raise ValueError("Machine label audit is incomplete, failed, human-mislabeled, hash-mismatched, or from another QA tool version")
            rejected_path = root / "rejected.jsonl"
            leakage_path = root / "leakage_audit.json"
            if (not rejected_path.is_file() or not leakage_path.is_file() or
                    audit.get("rejected_file_sha256") != sha256_file(rejected_path) or
                    audit.get("leakage_audit_sha256") != sha256_file(leakage_path)):
                raise ValueError("Machine audit no longer matches the rejected-frame or leakage-audit files")
            for row in records:
                mask_path = _contained(root, row["mask_path"])
                if not mask_path.is_file() or sha256_file(mask_path) != row.get("mask_sha256"):
                    raise ValueError(f"Machine-audited instance mask changed: {row.get('sample_id')}")
        else:
            raise ValueError("Unsupported label audit schema")
    elif not integrity_only and summary.get("training_ready") is not True and not pipeline_smoke:
        raise ValueError("Dataset is not training_ready; valid external audit required")
    return {"root": root, "summary": summary, "manifest_sha256": summary["manifest_sha256"],
            "label_policy_sha256": summary["label_policy_sha256"],
            "split_manifest_sha256": summary["split_manifest_sha256"], "counts": counts,
            "audit": audit, "rows": records}


def _run(command: list[str], cwd: Path) -> str:
    try:
        return subprocess.run(command, cwd=cwd, check=True, capture_output=True, text=True,
                              encoding="utf-8", errors="replace").stdout.strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        return f"unavailable: {exc}"


def _git_patch_bytes(project_root: Path) -> bytes:
    try:
        return subprocess.run(["git", "diff", "--binary", "HEAD"], cwd=project_root,
                              check=True, capture_output=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        raise ValueError("Cannot preserve the Git source patch for this experiment") from exc


def environment_provenance(project_root: Path) -> dict:
    git_revision = _run(["git", "rev-parse", "HEAD"], project_root)
    git_status = _run(["git", "status", "--porcelain=v1"], project_root)
    patch = _git_patch_bytes(project_root)
    source_hashes = {}
    try:
        listing = subprocess.run(["git", "ls-files", "--others", "--exclude-standard", "Source", "Tools"],
                                 cwd=project_root, check=True, capture_output=True, text=True, timeout=30)
        for name in listing.stdout.splitlines():
            source = project_root / name
            if source.suffix.lower() not in (".h", ".cpp", ".cs", ".py", ".usf", ".ush") or not source.is_file():
                continue
            source_hashes[name.replace("\\", "/")] = sha256_file(source)
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        source_hashes = {"error": str(exc)}
    try:
        import torch
        torch_info = {"version": torch.__version__, "cuda_version": torch.version.cuda,
                      "cuda_available": torch.cuda.is_available(),
                      "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None}
    except Exception as exc:  # Environment capture must still work on a CPU-only host.
        torch_info = {"error": str(exc)}
    try:
        import ultralytics
        ultralytics_version = ultralytics.__version__
    except Exception as exc:
        ultralytics_version = f"unavailable: {exc}"
    try:
        project_files = list(project_root.glob("*.uproject"))
        engine_project = json.loads(project_files[0].read_text(encoding="utf-8")) if len(project_files) == 1 else {}
    except Exception as exc:
        engine_project = {"error": str(exc)}
    nvidia_smi = _run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"], project_root)
    return {
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "python": {"version": sys.version, "executable": sys.executable, "implementation": platform.python_implementation()},
        "platform": platform.platform(), "torch": torch_info, "ultralytics": ultralytics_version,
        "gpu_driver": nvidia_smi, "unreal_project": engine_project,
        "git_revision": git_revision, "git_status_porcelain": git_status,
        "untracked_source_sha256": source_hashes,
        "git_dirty_patch_sha256": hashlib.sha256(patch).hexdigest(),
        "git_dirty_patch": patch,
    }


def _strict_determinism_snapshot(torch, event: str) -> dict:
    """Reassert strict flags after Ultralytics' seed setup and verify their state."""
    os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
    torch.use_deterministic_algorithms(True, warn_only=False)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cuda.matmul.allow_tf32 = False
    state = {
        "event": event,
        "deterministic_algorithms": torch.are_deterministic_algorithms_enabled(),
        "warn_only": torch.is_deterministic_algorithms_warn_only_enabled(),
        "cudnn_deterministic": torch.backends.cudnn.deterministic,
        "cudnn_benchmark": torch.backends.cudnn.benchmark,
        "cudnn_allow_tf32": torch.backends.cudnn.allow_tf32,
        "cuda_matmul_allow_tf32": torch.backends.cuda.matmul.allow_tf32,
        "cublas_workspace_config": os.environ["CUBLAS_WORKSPACE_CONFIG"],
    }
    if (not state["deterministic_algorithms"] or state["warn_only"] or
            not state["cudnn_deterministic"] or state["cudnn_benchmark"] or
            state["cudnn_allow_tf32"] or state["cuda_matmul_allow_tf32"]):
        raise RuntimeError(f"Strict deterministic settings were not active at {event}: {state}")
    return state


def attach_strict_determinism_callbacks(model, torch, record_path: Path) -> list[dict]:
    """Reapply and record strict determinism both before pretraining and at train start."""
    observations = []

    def callback(event):
        def apply(_trainer):
            observations.append(_strict_determinism_snapshot(torch, event))
            record_path.write_bytes(canonical(observations))
        return apply

    model.add_callback("on_pretrain_routine_start", callback("on_pretrain_routine_start"))
    model.add_callback("on_train_start", callback("on_train_start"))
    return observations


def create_experiment(*, dataset: Path, checkpoint: Path, output_root: Path, name: str,
                      project_root: Path, config: dict, pipeline_smoke: bool,
                      audit_path: Path | None = None) -> Path:
    verified = verify_dataset(dataset, pipeline_smoke=pipeline_smoke, audit_path=audit_path)
    checkpoint = checkpoint.resolve(strict=True)
    if not checkpoint.is_file():
        raise ValueError(f"Checkpoint is not a file: {checkpoint}")
    if not name or Path(name).name != name or name in (".", ".."):
        raise ValueError("Experiment name must be a single path component")
    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    experiment = output_root / name
    experiment.mkdir(exist_ok=False)
    (experiment / "weights").mkdir()
    (experiment / "training").mkdir()
    (experiment / "checkpoint.pt").write_bytes(checkpoint.read_bytes())
    checkpoint_hash = sha256_file(experiment / "checkpoint.pt")
    provenance = environment_provenance(project_root.resolve())
    # The immutable source snapshot includes a byte-for-byte copy of the git patch.
    patch_path = experiment / "git-dirty.patch"
    patch_path.write_bytes(provenance.pop("git_dirty_patch"))
    for relative in provenance.get("untracked_source_sha256", {}):
        if relative == "error":
            continue
        source = project_root.resolve() / relative
        destination = experiment / "source" / "untracked" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
    (experiment / "environment.json").write_bytes(canonical(provenance))
    (experiment / "training_args.json").write_bytes(canonical(config))
    (experiment / "training_data.yaml").write_text(_training_yaml(verified["root"]), encoding="utf-8")
    experiment_doc = {
        "schema_version": "sar-yolo-experiment-v1", "status": "prepared",
        "study_eligible": not pipeline_smoke,
        "classification": "pipeline-smoke-nonstudy" if pipeline_smoke else "study-candidate",
        "pipeline_smoke": pipeline_smoke,
        "dataset_root": str(verified["root"]), "dataset_manifest_sha256": verified["manifest_sha256"],
        "label_policy_sha256": verified["label_policy_sha256"],
        "split_manifest_sha256": verified["split_manifest_sha256"], "split_counts": verified["counts"],
        "checkpoint_original_path": str(checkpoint), "checkpoint_sha256": checkpoint_hash,
        "checkpoint_copy": "checkpoint.pt", "checkpoint_identity": f"sha256:{checkpoint_hash}",
        "config": config, "validation_split": "val", "test_evaluation": "not_run",
        "environment_file": "environment.json", "git_patch_file": "git-dirty.patch",
        "git_patch_sha256": sha256_file(patch_path),
        "untracked_source_directory": "source/untracked",
        "training_args_sha256": sha256_file(experiment / "training_args.json"),
        "training_data_yaml_sha256": sha256_file(experiment / "training_data.yaml"),
        "label_audit": str(audit_path.resolve()) if audit_path else None,
        "label_audit_sha256": sha256_file(audit_path.resolve()) if audit_path else None,
    }
    if audit_path:
        shutil.copyfile(audit_path.resolve(strict=True), experiment / "label-audit.json")
    (experiment / "experiment.json").write_bytes(canonical(experiment_doc))
    return experiment


def train(experiment: Path) -> None:
    """Train via the installed Ultralytics Python API, never opening test data."""
    doc = _read_json(experiment / "experiment.json")
    config = _read_json(experiment / "training_args.json")
    if config != doc.get("config") or sha256_file(experiment / "training_args.json") != doc.get("training_args_sha256"):
        raise ValueError("Training configuration changed after experiment preparation")
    smoke = doc.get("pipeline_smoke") is True
    expected_classification = "pipeline-smoke-nonstudy" if smoke else "study-candidate"
    if doc.get("classification") != expected_classification or doc.get("study_eligible") is not (not smoke):
        raise ValueError("Experiment study/smoke eligibility changed after preparation")
    if (config.get("deterministic") is not True or config.get("workers") != 0 or
            config.get("amp") is not False or not isinstance(config.get("batch"), int) or
            config.get("optimizer") not in ("AdamW", "SGD", "Adam")):
        raise ValueError("Training configuration violates pinned deterministic settings")
    if smoke and (config.get("epochs", 99) > 2 or config.get("patience", 99) > 1):
        raise ValueError("Pipeline smoke exceeds its two-epoch non-study cap")
    if not smoke and not doc.get("study_eligible"):
        raise ValueError("Non-study experiment cannot be promoted to study training")
    audit_copy = experiment / "label-audit.json"
    current_dataset = verify_dataset(Path(doc["dataset_root"]), pipeline_smoke=smoke,
                                     audit_path=audit_copy if audit_copy.is_file() else None)
    expected_dataset_hashes = {
        "manifest_sha256": doc["dataset_manifest_sha256"],
        "label_policy_sha256": doc["label_policy_sha256"],
        "split_manifest_sha256": doc["split_manifest_sha256"],
    }
    for key, expected_hash in expected_dataset_hashes.items():
        if current_dataset[key] != expected_hash:
            raise ValueError(f"Dataset changed after experiment preparation: {key}")
    if sha256_file(experiment / "checkpoint.pt") != doc["checkpoint_sha256"]:
        raise ValueError("Experiment checkpoint changed after preparation")
    if doc.get("label_audit") and sha256_file(experiment / "label-audit.json") != doc["label_audit_sha256"]:
        raise ValueError("Copied label audit changed after preparation")
    data_yaml = experiment / "training_data.yaml"
    expected_training_yaml = _training_yaml(Path(doc["dataset_root"]).resolve())
    if (data_yaml.read_text(encoding="utf-8") != expected_training_yaml or
            sha256_file(data_yaml) != doc["training_data_yaml_sha256"]):
        raise ValueError("Train/validation-only data configuration changed after preparation")
    # Set torch's strict deterministic behavior before Ultralytics/model construction.
    import numpy as np
    import torch
    random.seed(config["seed"])
    np.random.seed(config["seed"])
    torch.manual_seed(config["seed"])
    _strict_determinism_snapshot(torch, "before_model_construction")
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(config["seed"])
    from ultralytics import YOLO
    try:
        model = YOLO(str(experiment / "checkpoint.pt"))
        determinism_path = experiment / "training_determinism.json"
        determinism_observations = attach_strict_determinism_callbacks(model, torch, determinism_path)
        model.train(data=str(data_yaml), **config, project=str(experiment / "training"),
                    name="fit", exist_ok=False, val=True, split="val", save=True)
        expected_events = ["on_pretrain_routine_start", "on_train_start"]
        if [row["event"] for row in determinism_observations] != expected_events:
            raise RuntimeError("Ultralytics did not execute both strict-determinism callbacks")
        produced = experiment / "training" / "fit" / "weights"
        for filename in ("best.pt", "last.pt"):
            source = produced / filename
            if not source.is_file():
                raise RuntimeError(f"Training did not produce {filename}")
            shutil.copyfile(source, experiment / "weights" / filename)
        hashes = {name: sha256_file(experiment / "weights" / name) for name in ("best.pt", "last.pt")}
        (experiment / "weights.sha256").write_bytes(canonical(hashes))
        doc["status"] = "complete"
        doc["weights_sha256"] = hashes
        doc["results_csv"] = "training/fit/results.csv"
        doc["training_determinism_file"] = "training_determinism.json"
        doc["training_determinism_sha256"] = sha256_file(determinism_path)
        doc["test_evaluation"] = "not_run"
        (experiment / "experiment.json").write_bytes(canonical(doc))
    except Exception as exc:
        doc["status"] = "failed"
        doc["failure"] = f"{type(exc).__name__}: {exc}"
        (experiment / "experiment.json").write_bytes(canonical(doc))
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--name", required=True)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--audit", type=Path,
                        help="External, hash-bound label-audit approval; dataset.json remains immutable")
    parser.add_argument("--device", default="cpu", help="Explicit Ultralytics device, e.g. cpu or 0")
    parser.add_argument("--epochs", type=int, default=DEFAULTS["epochs"])
    parser.add_argument("--patience", type=int, default=DEFAULTS["patience"])
    parser.add_argument("--imgsz", type=int, default=DEFAULTS["imgsz"])
    parser.add_argument("--batch", type=int, default=DEFAULTS["batch"])
    parser.add_argument("--optimizer", choices=("AdamW", "SGD", "Adam"), default=DEFAULTS["optimizer"])
    parser.add_argument("--lr0", type=float, default=DEFAULTS["lr0"])
    parser.add_argument("--seed", type=int, default=DEFAULTS["seed"])
    parser.add_argument("--pipeline-smoke", action="store_true",
                        help="Explicitly permit a tiny run on training_ready=false; always labeled non-study")
    parser.add_argument("--prepare-only", action="store_true", help="Create provenance bundle without training")
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.epochs < 1 or args.patience < 0 or args.imgsz < 1 or args.batch < 1 or args.seed < 0 or args.lr0 <= 0:
        parser.error("epochs/imgsz/batch must be positive; patience/seed non-negative; lr0 positive")
    config = {**DEFAULTS, "epochs": args.epochs, "patience": args.patience,
              "imgsz": args.imgsz, "batch": args.batch, "optimizer": args.optimizer,
              "lr0": args.lr0, "seed": args.seed, "device": args.device}
    if args.pipeline_smoke:
        config["epochs"] = min(config["epochs"], 2)
        config["patience"] = min(config["patience"], 1)
    try:
        experiment = create_experiment(dataset=args.dataset, checkpoint=args.checkpoint,
                                       output_root=args.output_root, name=args.name,
                                       project_root=args.project_root, config=config,
                                       pipeline_smoke=args.pipeline_smoke, audit_path=args.audit)
        print(experiment)
        if not args.prepare_only:
            train(experiment)
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"YOLO experiment rejected or failed: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
