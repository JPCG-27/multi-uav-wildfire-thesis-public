"""Validate native SAR exports and assemble an immutable, scenario-split YOLO dataset.

Requires NumPy and Pillow. No training or test-set evaluation is performed.
Boxes are independently reconstructed from uint32 instance masks, never bones.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

SPLITS = ("train", "val", "test")
POLICY = "person-visible-v1"


def canonical(value) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8-sig"),
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError(f"Nonfinite JSON: {x}")))


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def contained(root: Path, name: str) -> Path:
    path = (root / name).resolve()
    if Path(name).is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError(f"Export path escapes its directory: {name}")
    return path


def load_runs(paths: list[Path]) -> list[tuple[Path, dict]]:
    runs = []
    for path in paths:
        path = path.resolve()
        if path.is_dir():
            path /= "run.json"
        run = read_json(path)
        if run.get("schema_version") != "sar-export-v1" or run.get("complete") is not True:
            raise ValueError(f"Incomplete or unsupported export: {path}")
        if not isinstance(run.get("scenario_family_id"), str) or not run["scenario_family_id"].strip():
            raise ValueError(f"Missing scenario family: {path}")
        if not run.get("frames") or len(set(run["frames"])) != len(run["frames"]):
            raise ValueError(f"Missing/duplicate frame list: {path}")
        audit = path.parent / "launch_audit.json"
        if audit.is_file():
            run["provenance"] = {**run.get("provenance", {}), "launcher": read_json(audit)["provenance"]}
        runs.append((path.parent, run))
    return runs


def freeze_splits(runs: list[tuple[Path, dict]], seed: int) -> dict:
    families = sorted({r["scenario_family_id"] for _, r in runs},
                      key=lambda f: hashlib.sha256(f"{seed}:{f}".encode()).hexdigest())
    if len(families) < 3:
        raise ValueError("At least three independent scenario families are needed for train/val/test")
    n_val = max(1, int(len(families) * .2))
    n_test = max(1, int(len(families) * .2))
    n_train = len(families) - n_val - n_test
    assignments = {f: ("train" if i < n_train else "val" if i < n_train + n_val else "test")
                   for i, f in enumerate(families)}
    return {"schema_version": "sar-splits-v1", "seed": seed,
            "unit": "scenario_family", "requested_fractions": [0.6, 0.2, 0.2],
            "assignments": assignments}


def validate_splits(runs, splits):
    if splits.get("schema_version") != "sar-splits-v1":
        raise ValueError("Unsupported split manifest")
    assignments = splits["assignments"]
    if any(s not in SPLITS for s in assignments.values()):
        raise ValueError("Invalid split name")
    # A renamed run/family must not move the same source human trajectories to another split.
    sources = {}
    for _, run in runs:
        family = run["scenario_family_id"]
        if family not in assignments:
            raise ValueError(f"Family absent from frozen splits: {family}")
        digest = run.get("source_hashes", {}).get("humans")
        if not digest:
            raise ValueError(f"Missing source human trajectory hash: {family}")
        previous = sources.setdefault(digest, assignments[family])
        if previous != assignments[family]:
            raise ValueError(f"Shared human trajectories cross splits: {family}")
    return assignments


def mask_annotations(root: Path, frame: dict, min_pixels: int, min_height: int):
    if frame.get("schema_version") != "sar-frame-v1" or frame.get("sensor_contract_version") != 5:
        raise ValueError("Unsupported frame/sensor contract")
    if frame.get("capture_source") != "SCS_FinalColorLDR":
        raise ValueError("Dataset RGB must use the production FinalColorLDR capture")
    width, height = frame["width"], frame["height"]
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ValueError("Invalid dimensions")
    mask_path = contained(root, frame["mask_path"])
    if mask_path.stat().st_size != width * height * 4:
        raise ValueError(f"Wrong mask byte count: {mask_path}")
    mask = np.fromfile(mask_path, dtype="<u4").reshape(height, width)
    people = frame["persons"]
    ids = [p["person_id"] for p in people]
    if len(set(ids)) != len(ids) or any(not isinstance(i, int) or not 0 <= i < 2**32 - 1 for i in ids):
        raise ValueError("Invalid or duplicate person IDs")
    unknown = set(int(i) for i in np.unique(mask)) - {0} - {i + 1 for i in ids}
    if unknown:
        raise ValueError(f"Mask has unknown IDs: {sorted(unknown)}")
    annotations, exclusions = [], list(frame["quality_exclusions"])
    for p in sorted(people, key=lambda p: p["person_id"]):
        ys, xs = np.nonzero(mask == p["person_id"] + 1)
        count = len(xs)
        amodal = p["amodal_pixels"]
        abox = p["amodal_bbox_xyxy"]
        if not isinstance(amodal, int) or not 0 <= count <= amodal <= width * height:
            raise ValueError("Visible/amodal pixel counts are inconsistent")
        if amodal:
            if (not isinstance(abox, list) or len(abox) != 4 or
                any(not isinstance(v, int) for v in abox) or
                not 0 <= abox[0] < abox[2] <= width or not 0 <= abox[1] < abox[3] <= height):
                raise ValueError("Invalid clipped amodal box")
        elif abox is not None:
            raise ValueError("Empty amodal mask has a box")
        box = [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1] if count else None
        if "visible_pixels" in p and p["visible_pixels"] != count:
            raise ValueError("Native visible count does not match instance mask")
        if "visible_bbox_xyxy" in p and p["visible_bbox_xyxy"] != box:
            raise ValueError("Native visible box does not match instance mask")
        if box and (box[0] < abox[0] or box[1] < abox[1] or box[2] > abox[2] or box[3] > abox[3]):
            raise ValueError("Visible box exceeds amodal support")
        if box and (count < min_pixels or box[3] - box[1] < min_height):
            exclusions.append(f"insufficient_visible_support:person={p['person_id']}")
        annotations.append({**p, "visible_pixels": count, "bbox_xyxy": box,
                            "occlusion_ratio": 1 - count / amodal if amodal else None,
                            "projected_height": abox[3] - abox[1] if abox else 0,
                            "touches_image_border": bool(abox and (abox[0] == 0 or abox[1] == 0 or abox[2] == width or abox[3] == height))})
    return annotations, sorted(set(exclusions))


def yolo_lines(annotations, width, height):
    lines = []
    for p in annotations:
        if p["bbox_xyxy"] is None:
            continue
        x0, y0, x1, y1 = p["bbox_xyxy"]
        lines.append(f"0 {(x0+x1)/(2*width):.10f} {(y0+y1)/(2*height):.10f} {(x1-x0)/width:.10f} {(y1-y0)/height:.10f}")
    return ("\n".join(lines) + ("\n" if lines else "")).encode()


def image_signature(path):
    with Image.open(path) as im:
        rgb = im.convert("RGB")
        pixels_hash = hashlib.sha256(canonical(list(rgb.size)) + rgb.tobytes()).hexdigest()
        gray = np.asarray(rgb.convert("L").resize((8, 8), Image.Resampling.LANCZOS))
        bits = np.packbits(gray >= gray.mean()).tobytes()
        return rgb.size, pixels_hash, int.from_bytes(bits, "big")


def near_duplicate_candidates(rows, limit=1000):
    """Five disjoint bands guarantee candidates for a 64-bit aHash distance <=4.

    This is a review aid, not evidence that semantically similar scenes are independent.
    Stop at the review limit rather than allowing all-sky frames to cause quadratic output.
    """
    bands, found = {}, []
    for row in rows:
        signature = int(row["perceptual_hash"], 16)
        candidates = {}
        for band in range(5):
            shift, count = band * 13, min(13, 64 - band * 13)
            key = band, (signature >> shift) & ((1 << count) - 1)
            for other in bands.get(key, []):
                if other["split"] != row["split"]:
                    candidates[other["sample_id"]] = other
            bands.setdefault(key, []).append(row)
        for other in candidates.values():
            distance = (signature ^ int(other["perceptual_hash"], 16)).bit_count()
            if distance <= 4:
                found.append({"left": other["sample_id"], "right": row["sample_id"], "hamming_distance": distance})
                if len(found) >= limit:
                    return {"candidates": found, "truncated": True}
    return {"candidates": found, "truncated": False}


def assemble(paths, split_path, output, *, min_pixels=4, min_height=2, qa_limit=200,
             train_frame_allowlist=None):
    if min_pixels < 1 or min_height < 1 or qa_limit < 0:
        raise ValueError("Invalid label support/QA parameters")
    output = output.resolve()
    if output.exists():
        raise ValueError(f"Refusing to overwrite dataset: {output}")
    runs = load_runs(paths)
    splits = read_json(split_path)
    assignments = validate_splits(runs, splits)
    # Optional paired-ablation gate. Keys are (scenario_family, tick, camera_id);
    # keeping it opt-in preserves the normal assembler behavior byte-for-byte.
    train_frame_allowlist = (None if train_frame_allowlist is None else
                             {tuple(identity) for identity in train_frame_allowlist})
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent)).resolve()
    try:
        for split in SPLITS:
            for kind in ("images", "labels"):
                (staging / kind / split).mkdir(parents=True)
        (staging / "qa").mkdir()
        rows, rejected, exact, identities = [], [], {}, set()
        for root, run in sorted(runs, key=lambda r: (r[1]["scenario_family_id"], str(r[0]))):
            family = run["scenario_family_id"]
            split = assignments[family]
            for name in sorted(run["frames"]):
                frame = read_json(contained(root, name))
                annotations, exclusions = mask_annotations(root, frame, min_pixels, min_height)
                if (split == "train" and train_frame_allowlist is not None and
                        (family, frame.get("tick"), frame.get("camera_id")) not in train_frame_allowlist):
                    exclusions = sorted(set(exclusions) | {"paired_variant_support_exclusion"})
                image = contained(root, frame["image_path"])
                size, pixel_hash, ahash = image_signature(image)
                if size != (frame["width"], frame["height"]):
                    raise ValueError(f"RGB/mask dimensions differ: {image}")
                # Content plus semantic identity permits repeat exports to be recognized.
                identity = [family, frame["tick"], frame["camera_id"], pixel_hash,
                            sha256(contained(root, frame["mask_path"]))]
                sample_id = hashlib.sha256(canonical(identity)).hexdigest()[:24]
                if sample_id in identities:
                    raise ValueError(f"Duplicate sample/repeated export: {name}")
                identities.add(sample_id)
                record = {**frame, "sample_id": sample_id, "scenario_family_id": family,
                          "split": split, "image_sha256": sha256(image), "pixel_sha256": pixel_hash,
                          "mask_sha256": identity[-1], "perceptual_hash": f"{ahash:016x}",
                          "persons": annotations, "quality_exclusions": exclusions,
                          "source_hashes": run["source_hashes"], "provenance": run.get("provenance", {}),
                          "label_policy": POLICY}
                if exclusions:
                    rejected.append(record)
                    continue
                previous = exact.setdefault(pixel_hash, split)
                if previous != split:
                    raise ValueError(f"Exact RGB duplicate crosses splits: {name}")
                relative = f"images/{split}/{sample_id}.png"
                shutil.copyfile(image, staging / relative)
                label = yolo_lines(annotations, *size)
                (staging / "labels" / split / f"{sample_id}.txt").write_bytes(label)
                record["image_path"] = relative
                record["label_sha256"] = hashlib.sha256(label).hexdigest()
                # Keep auditable raw masks with the immutable dataset.
                mask_relative = f"masks/{sample_id}.u32"
                (staging / "masks").mkdir(exist_ok=True)
                shutil.copyfile(contained(root, frame["mask_path"]), staging / mask_relative)
                record["mask_path"] = mask_relative
                rows.append(record)
        rows.sort(key=lambda r: r["sample_id"])
        if not rows:
            raise ValueError("No eligible frames; inspect native quality exclusions")
        for split in SPLITS:
            if not any(r["split"] == split for r in rows):
                raise ValueError(f"No eligible frames in {split}; collect independent families")
        # QA selection interleaves pose/mesh strata rather than taking the first run's frames.
        pools = {}
        for row in rows:
            for p in row["persons"]:
                if p["visible_pixels"]:
                    pools.setdefault((p.get("pose"), p.get("mesh")), []).append(row)
            if not any(p["visible_pixels"] for p in row["persons"]):
                pools.setdefault(("empty", "empty"), []).append(row)
        chosen = {}
        for index in range(len(rows)):
            for key in sorted(pools, key=str):
                if index < len(pools[key]) and len(chosen) < qa_limit:
                    row = pools[key][index]
                    chosen[row["sample_id"]] = row
            if len(chosen) >= min(qa_limit, len(rows)):
                break
        for sid, row in chosen.items():
            with Image.open(staging / row["image_path"]) as src:
                im = src.convert("RGB")
            draw = ImageDraw.Draw(im)
            for p in row["persons"]:
                if p["bbox_xyxy"]:
                    x0, y0, x1, y1 = p["bbox_xyxy"]
                    draw.rectangle((x0, y0, x1 - 1, y1 - 1), outline="red", width=1)
                    draw.text((x0, max(0, y0 - 11)), f"{p['person_id']} {p.get('pose', '')}", fill="red")
            im.save(staging / "qa" / f"{sid}.png")
        audit = near_duplicate_candidates(rows)
        policy = {"version": POLICY, "min_visible_pixels": min_pixels,
                  "min_visible_box_height": min_height, "tiny_person_action": "exclude_entire_frame",
                  "dead_bodies": "person", "bbox": "visible_pixels_xyxy_exclusive_max",
                  "truncation": "border_contact_proxy_only", "smoke": "quarantine_unvalidated_frames",
                  "support_threshold_status": "provisional_until_pilot_review"}
        (staging / "label_policy.json").write_bytes(canonical(policy))
        (staging / "split_manifest.json").write_bytes(canonical(splits))
        manifest_bytes = b"".join(canonical(r) for r in rows)
        (staging / "manifest.jsonl").write_bytes(manifest_bytes)
        (staging / "rejected.jsonl").write_bytes(b"".join(canonical(r) for r in rejected))
        (staging / "leakage_audit.json").write_bytes(canonical(audit))
        (staging / "qa" / "index.json").write_bytes(canonical(list(chosen)))
        summary = {"schema_version": "sar-dataset-v1", "complete": True, "training_ready": False,
                   "pending": ["manual_label_QA", "freeze_support_thresholds", "review_near_duplicate_candidates"],
                   "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
                   "label_policy_sha256": sha256(staging / "label_policy.json"),
                   "split_manifest_sha256": sha256(staging / "split_manifest.json"),
                   "counts": {s: sum(r["split"] == s for r in rows) for s in SPLITS},
                   "rejected_frames": len(rejected), "qa_images": len(chosen)}
        (staging / "dataset.json").write_bytes(canonical(summary))
        # Explicit final absolute path works with Ultralytics' global datasets_dir setting.
        yaml = f"path: {json.dumps(output.as_posix())}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: person\n"
        (staging / "data.yaml").write_bytes(yaml.encode())
        staging.rename(output)
        return summary
    finally:
        # Only our fresh staging directory, never an existing dataset.
        if staging.exists() and staging.parent == output.parent and staging.name.startswith(f".{output.name}-"):
            shutil.rmtree(staging)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    freeze = sub.add_parser("freeze-splits")
    freeze.add_argument("runs", nargs="+", type=Path)
    freeze.add_argument("--output", required=True, type=Path)
    freeze.add_argument("--seed", type=int, default=9340)
    build = sub.add_parser("assemble")
    build.add_argument("runs", nargs="+", type=Path)
    build.add_argument("--splits", required=True, type=Path)
    build.add_argument("--output", required=True, type=Path)
    build.add_argument("--min-visible-pixels", type=int, default=4)
    build.add_argument("--min-visible-height", type=int, default=2)
    build.add_argument("--qa-limit", type=int, default=200)
    args = p.parse_args(argv)
    try:
        if args.command == "freeze-splits":
            runs = load_runs(args.runs)
            result = freeze_splits(runs, args.seed)
            validate_splits(runs, result)
            with args.output.open("xb") as f:
                f.write(canonical(result))
        else:
            result = assemble(args.runs, args.splits, args.output,
                              min_pixels=args.min_visible_pixels, min_height=args.min_visible_height,
                              qa_limit=args.qa_limit)
        print(json.dumps(result, indent=2))
        return 0
    except (ValueError, KeyError, OSError, TypeError) as error:
        p.exit(1, f"Dataset rejected: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
