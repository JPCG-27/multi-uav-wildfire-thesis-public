"""Common single-person-class evaluation on immutable simulator images.

AP uses confidence-ordered one-to-one IoU matching and a 101-point precision
envelope (COCO-style interpolation, no area/crowd/maxDet averaging). All models
use this same evaluator. Validation selects the operating threshold; test never
selects it. Latency includes predict preprocessing/inference/postprocessing but
excludes image decoding. Requires NumPy/Pillow; inference additionally requires
the pinned Ultralytics/PyTorch environment.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import time
from pathlib import Path

import numpy as np
from PIL import Image

from prepare_yolo_dataset import canonical, contained, read_json, sha256

IOU_THRESHOLDS = tuple(round(.5 + .05 * i, 2) for i in range(10))
METRIC_VERSION = "sar-person-ap101-v1"


def iou(box, others):
    if not len(others):
        return np.empty(0)
    box, others = np.asarray(box, dtype=float), np.asarray(others, dtype=float)
    wh = np.maximum(0, np.minimum(box[2:], others[:, 2:]) - np.maximum(box[:2], others[:, :2]))
    intersection = wh[:, 0] * wh[:, 1]
    union = (box[2] - box[0]) * (box[3] - box[1]) + (others[:, 2] - others[:, 0]) * (others[:, 3] - others[:, 1]) - intersection
    return np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)


def match(rows, predictions, threshold=.5, confidence=0.0):
    """Stable ties: sample ID, prediction input order, then lowest GT index."""
    detections, matched_people = [], {}
    total_gt = 0
    for row in sorted(rows, key=lambda r: r["sample_id"]):
        sid = row["sample_id"]
        gt = [p for p in row["persons"] if p["bbox_xyxy"] is not None]
        total_gt += len(gt)
        used = set()
        ordered = sorted(enumerate(predictions[sid]["detections"]), key=lambda pair: (-pair[1]["confidence"], pair[0]))
        for order, prediction in ordered:
            if prediction["confidence"] < confidence:
                continue
            overlaps = iou(prediction["bbox_xyxy"], [p["bbox_xyxy"] for p in gt])
            candidates = [i for i in range(len(gt)) if i not in used and overlaps[i] >= threshold]
            index = max(candidates, key=lambda i: (overlaps[i], -i)) if candidates else None
            if index is not None:
                used.add(index)
            detections.append((prediction["confidence"], sid, order, index is not None))
        matched_people[sid] = {gt[i]["person_id"] for i in used}
    detections.sort(key=lambda d: (-d[0], d[1], d[2]))
    return detections, total_gt, matched_people


def precision_recall(detections, total_gt):
    tp = np.cumsum([d[3] for d in detections], dtype=float)
    fp = np.arange(1, len(detections) + 1) - tp
    precision = tp / (tp + fp) if len(tp) else np.empty(0)
    recall = tp / total_gt if total_gt else np.zeros_like(tp)
    return precision, recall


def ap101(detections, total_gt):
    if not total_gt:
        return None
    precision, recall = precision_recall(detections, total_gt)
    if not len(precision):
        return 0.0
    envelope = np.maximum.accumulate(precision[::-1])[::-1]
    return float(np.mean([envelope[np.searchsorted(recall, r, side="left")] if r <= recall[-1] else 0.0
                          for r in np.linspace(0, 1, 101)]))


def operating_metrics(rows, predictions, confidence):
    detections, n_gt, matched = match(rows, predictions, .5, confidence)
    tp = sum(d[3] for d in detections)
    fp = len(detections) - tp
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / n_gt if n_gt else None
    f1 = 2 * tp / (2 * tp + fp + n_gt - tp) if n_gt + tp + fp else 0.0
    return {"confidence": confidence, "iou": .5, "tp": tp, "fp": fp, "fn": n_gt - tp,
            "precision": precision, "recall": recall, "f1": f1, "fp_per_image": fp / len(rows)}, matched


def choose_threshold(rows, predictions):
    # Evaluate only complete confidence tie groups, not an impossible partial tie.
    detections, n_gt, _ = match(rows, predictions, .5)
    best = (0.0, 1.0)
    tp = 0
    for index, detection in enumerate(detections):
        tp += detection[3]
        if index + 1 < len(detections) and detections[index + 1][0] == detection[0]:
            continue
        fp = index + 1 - tp
        f1 = 2 * tp / (n_gt + tp + fp) if n_gt + tp + fp else 0.0
        best = max(best, (f1, detection[0]))
    return best[1]


def metrics(rows, predictions, confidence):
    ap = {}
    for threshold in IOU_THRESHOLDS:
        detections, n_gt, _ = match(rows, predictions, threshold)
        ap[f"{threshold:.2f}"] = ap101(detections, n_gt)
    operating, _ = operating_metrics(rows, predictions, confidence)
    detections, n_gt, _ = match(rows, predictions, .5)
    precision, recall = precision_recall(detections, n_gt)
    # Only feasible confidence thresholds, never a partial equal-score group.
    ends = [i for i in range(len(detections)) if i + 1 == len(detections) or detections[i + 1][0] != detections[i][0]]
    recall90 = max((float(recall[i]) for i in ends if precision[i] >= .9), default=0.0) if n_gt else None
    return {"metric_version": METRIC_VERSION, "images": len(rows), "persons": n_gt,
            "ap50": ap["0.50"], "ap50_95": float(np.mean(list(ap.values()))) if n_gt else None,
            "ap_by_iou": ap, "recall_at_precision90": recall90, "operating": operating}


def strata(rows, predictions, confidence):
    _, matched = operating_metrics(rows, predictions, confidence)
    bins = {}
    for row in rows:
        for person in row["persons"]:
            if person["bbox_xyxy"] is None:
                continue
            height = person["bbox_xyxy"][3] - person["bbox_xyxy"][1]
            size = "<16" if height < 16 else "16-31" if height < 32 else "32-63" if height < 64 else ">=64"
            occlusion = person["occlusion_ratio"]
            occ_bin = "0-10%" if occlusion <= .1 else "10-50%" if occlusion <= .5 else ">50%"
            for dimension, value in (("pose", person["pose"]), ("mesh", person["mesh"]),
                                     ("native_visible_box_height", size), ("opaque_occlusion", occ_bin)):
                key = dimension, value
                group = bins.setdefault(key, {"persons": 0, "tp": 0, "images": set()})
                group["persons"] += 1
                group["tp"] += person["person_id"] in matched[row["sample_id"]]
                group["images"].add(row["sample_id"])
    return [{"dimension": k[0], "stratum": k[1], "persons": v["persons"], "tp": v["tp"],
             "recall": v["tp"] / v["persons"], "images": len(v["images"])} for k, v in sorted(bins.items())]


def load_predictions(path, rows, expected_dataset_hash):
    predictions = {}
    expected = {r["sample_id"]: r for r in rows}
    weights = set()
    inference_settings = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        p = json.loads(line)
        sid = p["sample_id"]
        if sid in predictions or sid not in expected:
            raise ValueError("Duplicate or unexpected prediction sample ID")
        if p["dataset_manifest_sha256"] != expected_dataset_hash or p["image_sha256"] != expected[sid]["image_sha256"]:
            raise ValueError("Predictions do not belong to these exact dataset images")
        if (not isinstance(p["weights_sha256"], str) or len(p["weights_sha256"]) != 64 or
                any(c not in "0123456789abcdef" for c in p["weights_sha256"])):
            raise ValueError("Missing checkpoint SHA-256")
        weights.add(p["weights_sha256"])
        inference_settings.add(canonical(p["inference"]))
        for d in p["detections"]:
            box = d["bbox_xyxy"]
            if d["class_id"] != 0 or len(box) != 4 or not all(math.isfinite(v) for v in box + [d["confidence"]]):
                raise ValueError("Invalid person prediction")
            if not (0 <= d["confidence"] <= 1 and 0 <= box[0] < box[2] <= expected[sid]["width"] and 0 <= box[1] < box[3] <= expected[sid]["height"]):
                raise ValueError("Prediction outside image/confidence bounds")
        predictions[sid] = p
    if predictions.keys() != expected.keys() or len(weights) != 1 or len(inference_settings) != 1:
        raise ValueError("Missing predictions or mixed checkpoint/inference settings")
    return predictions, next(iter(weights))


def predict(dataset, rows, weights, output, dataset_hash, *, device="cpu", imgsz=640, conf=.001, nms_iou=.7, max_det=300):
    if not weights.is_file():
        raise ValueError("Checkpoint must be an existing local file; automatic downloads are disabled")
    os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
    import torch
    import ultralytics
    from ultralytics import YOLO
    random.seed(0)
    np.random.seed(0)
    torch.manual_seed(0)
    torch.use_deterministic_algorithms(True, warn_only=False)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    if device != "cpu" and not torch.cuda.is_available():
        raise ValueError("CUDA requested but unavailable")
    model = YOLO(str(weights))
    if str(model.names.get(0, "")).lower() != "person":
        raise ValueError("This protocol requires class 0 to be person")
    digest = sha256(weights)
    inference = {"imgsz": imgsz, "confidence_floor": conf, "nms_iou": nms_iou, "max_det": max_det,
                 "person_class_id": 0, "device": device, "half": False, "augment": False,
                 "seed": 0, "deterministic_algorithms": True, "tf32": False,
                 "torch": torch.__version__, "ultralytics": ultralytics.__version__}
    options = dict(imgsz=imgsz, conf=conf, iou=nms_iou, max_det=max_det, classes=[0],
                   device=device, half=False, augment=False, verbose=False)
    with Image.open(contained(dataset, rows[0]["image_path"])) as im:
        warm = np.asarray(im.convert("RGB"))[:, :, ::-1].copy()
    for _ in range(3):
        model.predict(warm, **options)
    cuda = device != "cpu"
    if cuda:
        torch.cuda.reset_peak_memory_stats()
    timings = []
    with output.open("xb") as f:
        for row in sorted(rows, key=lambda r: r["sample_id"]):
            with Image.open(contained(dataset, row["image_path"])) as im:
                bgr = np.asarray(im.convert("RGB"))[:, :, ::-1].copy()
            if cuda:
                torch.cuda.synchronize()
            start = time.perf_counter()
            result = model.predict(bgr, **options)[0]
            boxes = result.boxes.cpu()
            if cuda:
                torch.cuda.synchronize()
            elapsed = (time.perf_counter() - start) * 1000
            timings.append(elapsed)
            detections = [{"class_id": 0, "confidence": float(c), "bbox_xyxy": [float(v) for v in b]}
                          for b, c in zip(boxes.xyxy.tolist(), boxes.conf.tolist())]
            detections.sort(key=lambda d: (-d["confidence"], *d["bbox_xyxy"]))
            f.write(canonical({"sample_id": row["sample_id"], "weights_sha256": digest,
                               "dataset_manifest_sha256": dataset_hash, "image_sha256": row["image_sha256"],
                               "inference": inference, "detections": detections, "latency_ms": elapsed}))
    return {"p50_ms": float(np.median(timings)), "p95_ms": float(np.percentile(timings, 95)),
            "warmup_images": 3, "timed_images": len(rows), "image_decode_included": False,
            "peak_gpu_allocated_bytes": torch.cuda.max_memory_allocated() if cuda else None,
            "hardware": torch.cuda.get_device_name() if cuda else "cpu", "inference": inference}


def paired_bootstrap(rows, baseline, candidate, base_threshold, candidate_threshold, replicates=2000, seed=9340):
    families = sorted({r["scenario_family_id"] for r in rows})
    if len(families) < 2:
        raise ValueError("Scenario-clustered uncertainty needs at least two families")
    groups = {f: [r for r in rows if r["scenario_family_id"] == f] for f in families}
    rng = np.random.default_rng(seed)
    deltas = {"ap50_95": [], "recall": []}
    for _ in range(replicates):
        sample, a, b = [], {}, {}
        for copy, family in enumerate(rng.choice(families, size=len(families), replace=True)):
            for row in groups[family]:
                sid = f"{copy}:{row['sample_id']}"
                sample.append({**row, "sample_id": sid})
                a[sid], b[sid] = baseline[row["sample_id"]], candidate[row["sample_id"]]
        ma, mb = metrics(sample, a, base_threshold), metrics(sample, b, candidate_threshold)
        if ma["ap50_95"] is not None:
            deltas["ap50_95"].append(mb["ap50_95"] - ma["ap50_95"])
            deltas["recall"].append(mb["operating"]["recall"] - ma["operating"]["recall"])
    return {"unit": "scenario_family", "families": len(families), "replicates": replicates, "seed": seed,
            "percentile95": {k: np.percentile(v, [2.5, 97.5]).tolist() if v else None for k, v in deltas.items()},
            "valid_replicates": len(deltas["recall"])}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--split", choices=("val", "test"), default="val")
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--weights", type=Path)
    source.add_argument("--predictions", type=Path)
    p.add_argument("--device", default="cpu")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--confidence-floor", type=float, default=.001)
    p.add_argument("--nms-iou", type=float, default=.7)
    p.add_argument("--max-det", type=int, default=300)
    p.add_argument("--validation-metrics", type=Path, help="Required for test: freezes the validation-selected threshold")
    p.add_argument("--audit", type=Path, help="Hash-bound label audit; required for final test")
    args = p.parse_args(argv)
    from train_yolo_experiment import verify_dataset
    if args.output.exists():
        p.error("Output already exists; evaluations are immutable")
    if args.imgsz <= 0 or not 0 <= args.confidence_floor <= 1 or not 0 < args.nms_iou <= 1 or args.max_det <= 0:
        p.error("Invalid inference settings")
    verified = verify_dataset(args.dataset, integrity_only=args.split == "val" and not args.audit, audit_path=args.audit)
    summary = verified["summary"]
    digest = summary["manifest_sha256"]
    rows = [r for r in verified["rows"] if r["split"] == args.split]
    if not rows:
        p.error("Split is empty")
    validation = None
    if args.split == "test":
        if not args.validation_metrics:
            p.error("Test requires frozen validation metrics; no threshold selection on test")
        validation = read_json(args.validation_metrics)
        if validation.get("split") != "val" or validation.get("dataset_manifest_sha256") != digest:
            p.error("Validation selection belongs to a different dataset")
        if validation.get("metric_version") != METRIC_VERSION:
            p.error("Validation metrics use a different metric definition")
        if validation.get("persons", 0) <= 0:
            p.error("Validation has no labeled people; its operating threshold is undefined")
        if args.weights and sha256(args.weights) != validation.get("weights_sha256"):
            p.error("Test checkpoint differs from validation checkpoint")
    args.output.mkdir(parents=True, exist_ok=False)
    latency = None
    prediction_path = args.predictions
    if args.weights:
        prediction_path = args.output / "predictions.jsonl"
        latency = predict(args.dataset, rows, args.weights, prediction_path, digest, device=args.device,
                          imgsz=args.imgsz, conf=args.confidence_floor, nms_iou=args.nms_iou, max_det=args.max_det)
    predictions, weights_hash = load_predictions(prediction_path, rows, digest)
    if validation and validation["weights_sha256"] != weights_hash:
        p.error("Test checkpoint differs from validation checkpoint")
    threshold = validation["threshold_selection"]["confidence"] if validation else choose_threshold(rows, predictions)
    inference = next(iter(predictions.values()))["inference"]
    if validation and validation["inference"] != inference:
        p.error("Test inference settings differ from validation")
    report = metrics(rows, predictions, threshold)
    report.update(split=args.split, dataset_manifest_sha256=digest, weights_sha256=weights_hash,
                  inference=inference, latency=latency,
                  provisional=not bool(args.audit),
                  threshold_selection={"rule": "max_validation_f1_then_higher_confidence", "confidence": threshold},
                  shared_threshold=operating_metrics(rows, predictions, .25)[0],
                  predictions_sha256=sha256(prediction_path),
                  label_audit_sha256=sha256(args.audit) if args.audit else None,
                  validation_metrics_sha256=sha256(args.validation_metrics) if validation else None)
    (args.output / "metrics.json").write_bytes(canonical(report))
    groups = strata(rows, predictions, threshold)
    with (args.output / "metrics_by_stratum.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["dimension", "stratum", "persons", "tp", "recall", "images"])
        writer.writeheader()
        writer.writerows(groups)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
