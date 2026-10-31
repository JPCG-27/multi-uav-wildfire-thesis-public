"""Run complete machine QA on an assembled SAR YOLO dataset.

This validates the simulator instance-mask labels and package integrity. It is
not human visual review and writes an external audit without changing the
immutable dataset. Approval is blocked on any failed check or cross-split near
duplicate candidate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prepare_yolo_dataset import (canonical, image_signature, mask_annotations,
                                 near_duplicate_candidates, read_json, sha256,
                                 yolo_lines)
from train_yolo_experiment import verify_dataset

SCHEMA = "sar-machine-label-audit-v1"
TOOL_VERSION = "machine-mask-qa-v1"
ALLOWED_EXCLUSIONS = {"unvalidated_smoke", "paired_variant_support_exclusion",
                      "cross_split_near_duplicate_quarantine"}
SUPPORT_EXCLUSION = re.compile(r"^insufficient_visible_support:person=(\d+)$")
ERROR_KEYS = ("package_integrity", "mask_label_mismatches", "visibility_safeguard_errors",
              "split_leakage_errors", "near_duplicate_candidates")


def _error(failures: list[dict], category: str, message: str, sample_id: str | None = None):
    item = {"category": category, "message": message}
    if sample_id is not None:
        item["sample_id"] = sample_id
    failures.append(item)


def _load_jsonl(path: Path) -> list[dict]:
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            raise ValueError(f"Blank JSONL row at line {line_number}")
        rows.append(json.loads(line))
    return rows


def _empty_record(dataset: Path, output: Path) -> dict:
    summary = {}
    try:
        summary = read_json(dataset / "dataset.json")
    except Exception:
        pass
    tool_path = Path(__file__).resolve()
    return {
        "schema_version": SCHEMA,
        "audit_kind": "automated_ground_truth_mask_validation",
        "tool_version": TOOL_VERSION,
        "audit_tool_sha256": sha256(tool_path),
        "dataset_root": str(dataset.resolve()),
        "dataset_manifest_sha256": summary.get("manifest_sha256"),
        "label_policy_sha256": summary.get("label_policy_sha256"),
        "split_manifest_sha256": summary.get("split_manifest_sha256"),
        "approved_for_training": False,
        "automated_checks_complete": False,
        "human_review_performed": False,
        "reviewer_type": "automated_machine_validation",
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "checked": {"samples": 0, "images": 0, "masks": 0, "labels": 0, "rejected_frames": 0},
        "counts": {key: 0 for key in ERROR_KEYS},
        "near_duplicate_candidates": [],
        "near_duplicate_audit_truncated": False,
        "environment_presentation_versions": {},
        "failures": [],
        "rejected_file_sha256": None,
        "policy": None,
    }


def audit_dataset(dataset: Path) -> dict:
    """Audit all packaged samples and return an approval record, failing closed."""
    dataset = dataset.resolve()
    record = _empty_record(dataset, dataset)
    failures = record["failures"]
    error_counts = Counter()
    try:
        verified = verify_dataset(dataset, integrity_only=True)
    except Exception as exc:
        _error(failures, "package_integrity", f"Dataset package validation failed: {exc}")
        error_counts["package_integrity"] += 1
        record["counts"] = {key: error_counts[key] for key in ERROR_KEYS}
        record["counts"]["total"] = sum(error_counts.values())
        return record

    root, summary, rows = verified["root"], verified["summary"], verified["rows"]
    try:
        policy = read_json(root / "label_policy.json")
    except Exception as exc:
        _error(failures, "package_integrity", f"Cannot read label policy JSON: {exc}")
        error_counts["package_integrity"] += 1
        record["counts"] = {key: error_counts[key] for key in ERROR_KEYS}
        record["counts"]["total"] = sum(error_counts.values())
        return record
    record.update(dataset_root=str(root), dataset_manifest_sha256=verified["manifest_sha256"],
                  label_policy_sha256=verified["label_policy_sha256"],
                  split_manifest_sha256=verified["split_manifest_sha256"], policy=policy)

    # These policy clauses protect the annotation semantics. Threshold values
    # remain exactly those frozen in the dataset manifest.
    min_pixels = policy.get("min_visible_pixels")
    min_height = policy.get("min_visible_box_height")
    if (policy.get("version") != "person-visible-v1" or
            isinstance(min_pixels, bool) or not isinstance(min_pixels, int) or min_pixels < 1 or
            isinstance(min_height, bool) or not isinstance(min_height, int) or min_height < 1 or
            policy.get("tiny_person_action") != "exclude_entire_frame" or
            policy.get("smoke") != "quarantine_unvalidated_frames" or
            policy.get("bbox") != "visible_pixels_xyxy_exclusive_max"):
        _error(failures, "visibility_safeguard_errors", "Unsupported or incomplete visibility/smoke label policy")
        error_counts["visibility_safeguard_errors"] += 1
        # Invalid thresholds cannot safely be used for mask recomputation.
        record["checked"] = {"samples": 0, "images": 0, "masks": 0, "labels": 0, "rejected_frames": 0}
        record["counts"] = {key: error_counts[key] for key in ERROR_KEYS}
        record["counts"]["total"] = sum(error_counts.values())
        return record

    checked_images = checked_masks = checked_labels = 0
    versions = Counter()
    family_split = {}
    human_source_split = {}
    exact_pixels = {}
    sample_ids = set()
    for row in rows:
        sid, split = row["sample_id"], row["split"]
        sample_ids.add(sid)
        version = row.get("environment_presentation_version")
        versions[str(version) if version is not None else "legacy_unversioned"] += 1
        previous_split = family_split.setdefault(row["scenario_family_id"], split)
        if previous_split != split:
            _error(failures, "split_leakage_errors", "Scenario family crosses splits", sid)
            error_counts["split_leakage_errors"] += 1
        source_hashes = row.get("source_hashes")
        human_hash = source_hashes.get("humans") if isinstance(source_hashes, dict) else None
        if not isinstance(human_hash, str) or not human_hash:
            _error(failures, "split_leakage_errors", "Missing source human trajectory hash", sid)
            error_counts["split_leakage_errors"] += 1
        else:
            prior = human_source_split.setdefault(human_hash, split)
            if prior != split:
                _error(failures, "split_leakage_errors", "Shared human trajectories cross splits", sid)
                error_counts["split_leakage_errors"] += 1
        pixel_hash = row.get("pixel_sha256")
        if not isinstance(pixel_hash, str):
            _error(failures, "split_leakage_errors", "Missing canonical RGB pixel hash", sid)
            error_counts["split_leakage_errors"] += 1
        else:
            prior = exact_pixels.setdefault(pixel_hash, split)
            if prior != split:
                _error(failures, "split_leakage_errors", "Exact RGB pixels cross splits", sid)
                error_counts["split_leakage_errors"] += 1

        try:
            width, height = row["width"], row["height"]
            image = root / row["image_path"]
            size, pixels_sha, perceptual = image_signature(image)
            checked_images += 1
            if size != (width, height) or pixels_sha != row.get("pixel_sha256"):
                _error(failures, "package_integrity", "Decoded RGB dimensions/pixels disagree with manifest", sid)
                error_counts["package_integrity"] += 1
            if f"{perceptual:016x}" != row.get("perceptual_hash"):
                _error(failures, "package_integrity", "Perceptual hash disagrees with decoded RGB", sid)
                error_counts["package_integrity"] += 1
            mask_path = root / row["mask_path"]
            if sha256(mask_path) != row.get("mask_sha256"):
                _error(failures, "package_integrity", "Instance-mask hash disagrees with manifest", sid)
                error_counts["package_integrity"] += 1
            checked_masks += 1
            annotations, exclusions = mask_annotations(root, row, min_pixels, min_height)
            if annotations != row.get("persons"):
                _error(failures, "mask_label_mismatches", "Instance mask does not reproduce manifest boxes/visibility", sid)
                error_counts["mask_label_mismatches"] += 1
            if exclusions != row.get("quality_exclusions") or exclusions:
                _error(failures, "visibility_safeguard_errors", "Accepted sample has quality exclusions or inconsistent mask policy", sid)
                error_counts["visibility_safeguard_errors"] += 1
            expected_label = yolo_lines(annotations, width, height)
            label_path = root / "labels" / split / f"{sid}.txt"
            if label_path.read_bytes() != expected_label:
                _error(failures, "mask_label_mismatches", "YOLO label file differs from mask-derived annotations", sid)
                error_counts["mask_label_mismatches"] += 1
            checked_labels += 1
        except Exception as exc:
            _error(failures, "mask_label_mismatches", f"Sample QA failed: {type(exc).__name__}: {exc}", sid)
            error_counts["mask_label_mismatches"] += 1

    versions = dict(sorted(versions.items()))
    if len(versions) > 1:
        _error(failures, "package_integrity", "Dataset mixes environment presentation versions", None)
        error_counts["package_integrity"] += 1

    # Verify the packager's quarantine file has explicit, recognized reasons.
    rejected_path = root / "rejected.jsonl"
    rejected_rows = []
    try:
        rejected_rows = _load_jsonl(rejected_path)
        record["rejected_file_sha256"] = sha256(rejected_path)
        if set(s.get("sample_id") for s in rejected_rows) & sample_ids:
            raise ValueError("A sample is both accepted and rejected")
        for row in rejected_rows:
            reasons = row.get("quality_exclusions")
            if not isinstance(reasons, list) or not reasons:
                raise ValueError(f"Rejected sample has no explicit quality exclusion: {row.get('sample_id')}")
            for reason in reasons:
                if reason in ALLOWED_EXCLUSIONS:
                    continue
                match = SUPPORT_EXCLUSION.fullmatch(reason) if isinstance(reason, str) else None
                if match is None:
                    raise ValueError(f"Unknown rejected-frame policy reason: {reason}")
                person = next((p for p in row.get("persons", []) if p.get("person_id") == int(match.group(1))), None)
                if person is None:
                    raise ValueError(f"Tiny-support exclusion references missing person: {reason}")
                visible = person.get("visible_pixels")
                bbox = person.get("bbox_xyxy")
                box_height = bbox[3] - bbox[1] if isinstance(bbox, list) and len(bbox) == 4 else 0
                if not (isinstance(visible, int) and (visible < min_pixels or box_height < min_height)):
                    raise ValueError(f"Tiny-support exclusion does not match pixel/height thresholds: {reason}")
    except Exception as exc:
        _error(failures, "visibility_safeguard_errors", f"Rejected-frame quarantine is invalid: {exc}")
        error_counts["visibility_safeguard_errors"] += 1

    # Recompute the split-leakage candidate audit instead of trusting the
    # stored review file; any near match needs explicit human adjudication.
    try:
        candidates = near_duplicate_candidates(rows)
        record["near_duplicate_candidates"] = candidates["candidates"]
        record["near_duplicate_audit_truncated"] = candidates["truncated"]
        stored_candidates = read_json(root / "leakage_audit.json")
        if stored_candidates != candidates:
            _error(failures, "split_leakage_errors", "Stored leakage audit differs from recomputation")
            error_counts["split_leakage_errors"] += 1
        if candidates["truncated"] or candidates["candidates"]:
            _error(failures, "near_duplicate_candidates",
                   "Cross-split near-duplicate candidates require human adjudication")
            error_counts["near_duplicate_candidates"] += max(1, len(candidates["candidates"]))
    except Exception as exc:
        _error(failures, "split_leakage_errors", f"Near-duplicate audit failed: {exc}")
        error_counts["split_leakage_errors"] += 1

    checked = {"samples": len(rows), "images": checked_images, "masks": checked_masks,
               "labels": checked_labels, "rejected_frames": len(rejected_rows)}
    complete = checked_images == checked_masks == checked_labels == len(rows)
    errors = {key: error_counts[key] for key in ERROR_KEYS}
    errors["total"] = sum(errors.values())
    approved = complete and errors["total"] == 0
    record.update(
        automated_checks_complete=complete,
        approved_for_training=approved,
        checked=checked,
        counts=errors,
        environment_presentation_versions=versions,
        leakage_audit_sha256=sha256(root / "leakage_audit.json") if (root / "leakage_audit.json").is_file() else None,
        audit_tool_sha256=sha256(Path(__file__).resolve()),
    )
    return record


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path,
                        help="New external audit JSON path; never written inside the dataset")
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("Audit output already exists; machine audit records are immutable")
    if args.output.resolve().is_relative_to(args.dataset.resolve()):
        parser.error("Audit output must be outside the immutable dataset directory")
    result = audit_dataset(args.dataset)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        stream.write(canonical(result))
    print(json.dumps(result, indent=2))
    return 0 if result["approved_for_training"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
