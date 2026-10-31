#!/usr/bin/env python3
"""Recheck packaged camera area and temporal-shift aggregates from summaries.

This verifies already-derived per-run summaries. It does not replay caches,
reload images, or project boxes onto terrain.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data" / "camera_studies"
T95_DF14 = 2.1447866879169273


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interval(values_by_seed: dict[str, list[float]]) -> tuple[float, float, float]:
    if set(values_by_seed) != {str(x) for x in range(27, 42)}:
        raise ValueError("derived summaries do not cover all 15 seed blocks")
    blocks = [statistics.fmean(v) for _, v in sorted(values_by_seed.items(), key=lambda x: int(x[0]))]
    mean = statistics.fmean(blocks)
    half = T95_DF14*statistics.stdev(blocks)/math.sqrt(15)
    return mean, mean-half, mean+half


def close(a: float, b: float, label: str) -> None:
    if not math.isclose(a, b, rel_tol=0, abs_tol=1e-9):
        raise ValueError(f"{label} differs: {a} vs {b}")


def main() -> None:
    receipt = json.loads((DATA/"camera_derived_evidence_receipt.json").read_text(encoding="utf-8"))
    temporal_path = DATA/"temporal_shift_per_run_summary.csv"
    area_path = DATA/"mixed_camera_territory_per_run.csv"
    if digest(temporal_path) != receipt["temporal_shift_compact_output"]["per_run_summary_sha256"]:
        raise ValueError("temporal-shift per-run summary hash mismatch")
    if digest(area_path) != receipt["area_share"]["input_summary_sha256"]:
        raise ValueError("territory per-run summary hash mismatch")

    temporal = read_csv(temporal_path)
    if len(temporal) != 6480:
        raise ValueError(f"expected 6,480 temporal-shift run summaries, found {len(temporal)}")
    for cohort in ("mixed", "four_camera"):
        rows = [r for r in temporal if r["cohort"] == cohort]
        if len(rows) != 3240 or any(int(r["shift_ranks"]) != 17 for r in rows):
            raise ValueError(f"{cohort}: incomplete per-run shift summary")
        original = statistics.fmean(float(r["original_equal_run_coverage"]) for r in rows)
        shifted = statistics.fmean(float(r["mean_17_shift_equal_run_coverage"]) for r in rows)
        deltas: dict[str, list[float]] = defaultdict(list)
        for r in rows:
            deltas[r["seed"]].append(float(r["original_minus_shift_mean_pp"])/100)
        delta_ci = interval(deltas)
        expected = receipt["temporal_shift_reported_values"][cohort]
        close(100*original, expected["aligned_equal_run_percent"], f"{cohort} aligned equal-run")
        close(100*shifted, expected["mean_17_shifted_equal_run_percent"], f"{cohort} mean shifted equal-run")
        close(100*delta_ci[0], expected["aligned_minus_shifted_pp"], f"{cohort} paired difference")
        close(100*delta_ci[1], expected["aligned_minus_shifted_ci95_pp"][0], f"{cohort} paired CI low")
        close(100*delta_ci[2], expected["aligned_minus_shifted_ci95_pp"][1], f"{cohort} paired CI high")

    area = read_csv(area_path)
    norm = json.loads((DATA/"mixed_camera_territory_normalized_analysis.json").read_text(encoding="utf-8"))
    if len(area) != 3240:
        raise ValueError(f"expected 3,240 mixed-camera territory run summaries, found {len(area)}")
    for grid in (100, 200, 300):
        rows = [r for r in area if int(r["grid"]) == grid]
        static = [r for r in rows if r["method_id"] == "mine_static"]
        dynamic = [r for r in rows if r["method_id"] == "mine_dynamic"]
        if len(static) != 540 or len(dynamic) != 540:
            raise ValueError(f"grid {grid}: expected 540 static and 540 dynamic runs")
        sta = 100*statistics.fmean(float(r["drone0_mean_owned_fraction"]) for r in static)
        dyn = 100*statistics.fmean(float(r["drone0_mean_owned_fraction"]) for r in dynamic)
        by_id = {r["run_id"].replace("mine_static", "mine_dynamic"): r for r in static}
        if set(by_id) != {r["run_id"] for r in dynamic}:
            raise ValueError(f"grid {grid}: static and dynamic territory scenarios do not pair")
        block: dict[str, list[float]] = defaultdict(list)
        for r in dynamic:
            s = by_id[r["run_id"]]
            block[r["seed"]].append(100*(float(r["drone0_mean_owned_fraction"])-float(s["drone0_mean_owned_fraction"])))
        diff = interval(block)
        expected = norm["strata"][f"grid={grid}"]["drone0"]
        close(sta, expected["static_mean_grid_share_percent"], f"grid {grid} static area")
        close(dyn, expected["dynamic_mean_grid_share_percent"], f"grid {grid} dynamic area")
        close(diff[0], expected["dynamic_minus_static_share_percentage_points"]["mean"], f"grid {grid} area difference")
        close(diff[1], expected["dynamic_minus_static_share_percentage_points"]["ci95_low"], f"grid {grid} area CI low")
        close(diff[2], expected["dynamic_minus_static_share_percentage_points"]["ci95_high"], f"grid {grid} area CI high")
    print("Verified camera cell-belief inputs are hash-pinned; temporal-shift and mixed-team area aggregates reproduce from packaged per-run summaries.")


if __name__ == "__main__":
    main()
