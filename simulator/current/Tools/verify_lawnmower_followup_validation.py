#!/usr/bin/env python3
"""Independently validate the V2/V1Aligned preproduction parity gate.

Read-only with respect to run outputs and manifests. On success or failure it
writes one immutable validation receipt under the follow-up controller root.
No Unreal process is launched.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_CONTROLLER_ROOT = PROJECT / "Saved/Benchmarks/lawnmower_followup_20261027"
DEFAULT_PILOT_ROOT = PROJECT / "Saved/Benchmarks/lawnmower_v2_pilot_run_20261027_r3"
DEFAULT_V2_ROOT = PROJECT / "Saved/Benchmarks/lawnmower_v2_full_20261027"
DEFAULT_ALIGNED_ROOT = PROJECT / "Saved/Benchmarks/lawnmower_v1_aligned_full_20261027"
RECEIPT_NAME = "validation_receipt.json"
VALIDATION_EXECUTION_NAME = "validation_execution.json"
STABLE_CSVS = (
    "summary.csv", "persons.csv", "events.csv", "yolo_frames.csv",
    "ideal_sensor_witness_people.csv", "ideal_sensor_witness_events.csv",
    "ideal_sensor_witness_summary.csv", "drones.csv", "ticks.csv",
)
V2_RUNTIME_METADATA = {
    "lawnmower_v2_algorithm_version": 2,
    "lawnmower_v2_lane_spacing_cells": 4,
    "lawnmower_v2_lane_spacing_rule": "max1_twice_floor_range_cells_sin_half_fov",
    "lawnmower_v2_scan_order": "spawn_side_inward_global_lanes_serpentine",
    "lawnmower_v2_progress_policy": "shared_completed_waypoints_server_slot_global_epoch",
    "lawnmower_v2_initial_waypoint": "nearest_unfinished_owned_lane_direct_flight",
}
ALIGNED_RUNTIME_METADATA = {
    "lawnmower_v1_aligned_algorithm_version": 1,
    "lawnmower_v1_aligned_lane_spacing_cells": 1,
    "lawnmower_v1_aligned_scan_order": "spawn_side_inward_one_cell_serpentine",
    "lawnmower_v1_aligned_initial_waypoint": "owned_spawn_cell_direct_flight",
    "lawnmower_v1_aligned_waypoint_policy": "arrival_only_direct_flight_v1_progress",
    "lawnmower_v1_aligned_static_partition": "SpawnAwareUniform",
}
ALLOWED_CONFIG_DIFFS = {
    "run_id", "benchmark_run_subdir", "method_id", "planner_strategy",
    "generated_fire_csv_output_path", "generated_humans_csv_output_path",
    "generated_terrain_csv_output_path", "generated_fuel_csv_output_path",
}


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def same_value(a: Any, b: Any) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=1e-7)
    return a == b


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def json_path(*candidates: Path) -> Path:
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError("None of the expected manifest paths exist: " + ", ".join(map(str, candidates)))


def manifest_pair(root: Path) -> tuple[Path, dict[str, Any], Path, dict[str, Any]]:
    pre_path = json_path(root / "manifest_preexecution.json", root / "arm_manifest_preexecution.json")
    runtime_path = json_path(root / "manifest_runtime.json", root / "arm_manifest_runtime.json")
    return pre_path, read_json(pre_path), runtime_path, read_json(runtime_path)


def parity_groups(pre: dict[str, Any], runtime: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    parity = runtime.get("parity", pre.get("parity"))
    if isinstance(parity, dict) and "fresh_runs" in parity and "batch_runs" in parity:
        return parity["fresh_runs"], parity["batch_runs"]
    rows = runtime.get("parity_jobs", pre.get("parity_jobs", []))
    fresh = [j for j in rows if j.get("cohort_role") == "parity_fresh"]
    batch = [j for j in rows if j.get("cohort_role") == "parity_batch" or j.get("repeat")]
    return fresh, batch


def production_rows(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if isinstance(manifest.get("production"), dict):
        rows = manifest["production"].get("runs", [])
        if rows:
            return rows
    for key in ("production_runs", "production_jobs"):
        if isinstance(manifest.get(key), list):
            return manifest[key]
    rows = []
    for batch in manifest.get("production_batches", []):
        rows.extend(batch.get("runs", []))
    return rows


def config_for(job: dict[str, Any]) -> dict[str, Any]:
    path = Path(job["config_path"])
    require(path.is_file(), f"Launch config missing: {path}")
    expected = job.get("config_sha256")
    require(bool(expected) and sha(path) == expected, f"Launch config hash mismatch: {path}")
    return read_json(path)


def artifact_hashes(job: dict[str, Any]) -> dict[str, str]:
    raw = job.get("artifact_hashes")
    require(isinstance(raw, dict), f"Artifact hashes missing for {job.get('run_id')}")
    return {str(k): str(v.get("sha256") if isinstance(v, dict) else v) for k, v in raw.items()}


def check_artifacts(job: dict[str, Any]) -> dict[str, str]:
    run_dir = Path(job["run_dir"])
    recorded = artifact_hashes(job)
    computed: dict[str, str] = {}
    for name in STABLE_CSVS:
        path = run_dir / name
        require(path.is_file(), f"Stable CSV missing: {path}")
        digest = sha(path)
        require(recorded.get(name) == digest, f"Recorded artifact hash mismatch: {job['run_id']}/{name}")
        computed[name] = digest
    summary = read_csv(run_dir / "summary.csv")
    require(len(summary) == 1 and int(float(summary[0]["final_tick"])) == 8000,
            f"Summary horizon mismatch: {job['run_id']}")
    ticks = read_csv(run_dir / "ticks.csv")
    require(len(ticks) == 8000, f"Tick trace length mismatch: {job['run_id']}")
    return computed


def run_key(job: dict[str, Any]) -> tuple[int, int, str]:
    return int(job["seed"]), int(job["grid"]), str(job["regime"])


def validate_job_common(job: dict[str, Any], arm_id: str, expected_pins: dict[str, Any],
                        metadata: dict[str, Any] | None = None) -> dict[str, Any]:
    require(job.get("status") == "runtime_evidence_passed",
            f"Parity job is not complete: {job.get('run_id')} ({job.get('status')})")
    require(len(job.get("expected_pinned_parameters", {})) == 49,
            f"Expected 49 runtime pins: {job.get('run_id')}")
    require(job["expected_pinned_parameters"] == expected_pins,
            f"Job pin map differs from arm pin map: {job.get('run_id')}")
    cfg = config_for(job)
    runtime_path = Path(job["runtime_config_path"])
    require(runtime_path.is_file(), f"Runtime config missing: {runtime_path}")
    expected_runtime_hash = job.get("runtime_config_sha256")
    if expected_runtime_hash:
        require(sha(runtime_path) == expected_runtime_hash, f"Runtime config hash mismatch: {runtime_path}")
    runtime = read_json(runtime_path)
    for key, expected in expected_pins.items():
        require(key in runtime and same_value(runtime[key], expected),
                f"Runtime pin mismatch in {job['run_id']}: {key}")
    for key in ("method_id", "planner_strategy", "partition_strategy", "region_regime", "max_sim_ticks"):
        if key in cfg:
            require(key in runtime and same_value(runtime[key], cfg[key]),
                    f"Launch/runtime selector mismatch in {job['run_id']}: {key}")
    require(int(runtime.get("max_sim_ticks", -1)) == 8000, f"Runtime tick cap mismatch: {job['run_id']}")
    if metadata:
        for key, expected in metadata.items():
            require(key in runtime and same_value(runtime[key], expected),
                    f"Planner metadata mismatch in {job['run_id']}: {key}")
    hashes = check_artifacts(job)
    return {"run_id": job["run_id"], "run_key": list(run_key(job)), "method_id": cfg.get("method_id"),
            "planner_strategy": cfg.get("planner_strategy"), "partition_strategy": cfg.get("partition_strategy"),
            "runtime_config_sha256": sha(runtime_path), "csv_sha256": hashes}


def compare_csv_jobs(left: dict[str, Any], right: dict[str, Any], label: str) -> dict[str, Any]:
    left_dir, right_dir = Path(left["run_dir"]), Path(right["run_dir"])
    files = {}
    for name in STABLE_CSVS:
        lp, rp = left_dir / name, right_dir / name
        require(lp.is_file() and rp.is_file(), f"Parity CSV missing ({label}): {lp if not lp.is_file() else rp}")
        lh, rh = sha(lp), sha(rp)
        files[name] = {"left_sha256": lh, "right_sha256": rh, "byte_identical": lh == rh}
    require(all(item["byte_identical"] for item in files.values()), f"Stable CSV parity failed ({label})")
    return {"label": label, "left_run_id": left["run_id"], "right_run_id": right["run_id"], "files": files}


def validate_arm_parity(arm_id: str, arm_pre: dict[str, Any], arm_runtime: dict[str, Any]) -> dict[str, Any]:
    pins = arm_pre.get("pins", arm_pre.get("inherited_parameter_pins"))
    require(isinstance(pins, dict) and len(pins) == 49, f"{arm_id}: expected 49 arm pins")
    fresh, batch = parity_groups(arm_pre, arm_runtime)
    require(len(fresh) == 6 and len(batch) == 8,
            f"{arm_id}: expected six fresh and eight batch parity jobs")
    expected_keys = {(27, grid, regime) for grid in (100, 200, 300) for regime in ("static", "dynamic")}
    fresh_by_key = {run_key(j): j for j in fresh}
    require(len(fresh_by_key) == 6 and set(fresh_by_key) == expected_keys,
            f"{arm_id}: fresh parity roster is not seed27 × grids100/200/300 × static/dynamic")
    require(len({j.get("run_id") for j in fresh}) == 6, f"{arm_id}: fresh run IDs are not unique")
    checked_fresh = []
    metadata = ALIGNED_RUNTIME_METADATA if arm_id == "v1_aligned" else V2_RUNTIME_METADATA
    for job in fresh:
        require(job.get("cohort_role") == "parity_fresh" and job.get("comparison_key") ==
                f"seed{job['seed']}_g{job['grid']}_{job['regime']}",
                f"{arm_id}: fresh parity key/role mismatch for {job['run_id']}")
        require(not job.get("repeat", False), f"{arm_id}: fresh parity row marked as repeat")
        checked_fresh.append(validate_job_common(job, arm_id, pins, metadata if arm_id == "v1_aligned" else metadata))
    batch_by_id = {j.get("run_id"): j for j in batch}
    require(len(batch_by_id) == 8, f"{arm_id}: batch run IDs are not unique")
    batch_comparisons = []
    used = set()
    repeats = []
    for job in batch:
        key = run_key(job)
        require(key in fresh_by_key, f"{arm_id}: unexpected batch parity key {key}")
        baseline = fresh_by_key[key]
        require(job.get("parity_baseline_run_id") == baseline["run_id"],
                f"{arm_id}: baseline run ID mismatch for {job['run_id']}")
        if job.get("repeat"):
            require(key[1] == 300 and bool(job.get("repeat_of")) and
                    job.get("cohort_role") == "parity_batch_repeat",
                    f"{arm_id}: invalid repeat batch row {job['run_id']}")
            repeats.append(job)
        else:
            require(job.get("cohort_role") == "parity_batch" and job.get("comparison_key") ==
                    f"seed{job['seed']}_g{job['grid']}_{job['regime']}",
                    f"{arm_id}: unexpected batch role/key {job['run_id']}")
            require(key not in used, f"{arm_id}: duplicate primary batch key {key}")
            used.add(key)
        validate_job_common(job, arm_id, pins, metadata)
        batch_comparisons.append(compare_csv_jobs(baseline, job, f"{arm_id} fresh to batch {job['run_id']}"))
    require(used == expected_keys and len(repeats) == 2,
            f"{arm_id}: expected six primary batch jobs and two G300 repeats")
    for repeat in repeats:
        primary = batch_by_id.get(repeat["repeat_of"])
        require(primary is not None and not primary.get("repeat"),
                f"{arm_id}: repeat target missing: {repeat['repeat_of']}")
        require(run_key(primary) == run_key(repeat), f"{arm_id}: repeat target key mismatch")
        batch_comparisons.append(compare_csv_jobs(primary, repeat, f"{arm_id} batch repeat {repeat['run_id']}"))
    groups = arm_runtime.get("parity_batches", arm_pre.get("parity_batches", []))
    require(len(groups) == 3, f"{arm_id}: expected three in-process grid batches")
    expected_sizes = {100: 2, 200: 2, 300: 4}
    grouped_jobs = []
    for group in groups:
        group_jobs = group.get("jobs", [])
        require(len(group_jobs) == expected_sizes[int(group_jobs[0]["grid"])],
                f"{arm_id}: unexpected same-grid reset batch size {group.get('batch_id')}")
        grids = {int(j["grid"]) for j in group_jobs}
        require(len(grids) == 1 and len(group_jobs) == expected_sizes[next(iter(grids))],
                f"{arm_id}: mixed-grid or malformed reset batch {group.get('batch_id')}")
        grouped_jobs.extend(j["run_id"] for j in group_jobs)
    require(set(grouped_jobs) == set(batch_by_id) and len(grouped_jobs) == 8,
            f"{arm_id}: batch groups do not cover all eight parity jobs once")
    return {"fresh_runs_checked": len(checked_fresh), "batch_runs_checked": len(batch),
            "fresh_to_batch_comparisons": len(batch), "repeat_comparisons": len(repeats),
            "in_process_reset_batches": [{"batch_id": b["batch_id"], "run_ids": [j["run_id"] for j in b.get("jobs", [])],
                                           "grid": int(b["jobs"][0]["grid"])} for b in groups],
            "fresh_runs": checked_fresh, "csv_comparisons": batch_comparisons}


def normalized_config(cfg: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in cfg.items() if key not in ALLOWED_CONFIG_DIFFS}


def validate_arm_fairness(v2_pre: dict[str, Any], aligned_pre: dict[str, Any]) -> dict[str, Any]:
    v2_runs = production_rows(v2_pre)
    aligned_runs = production_rows(aligned_pre)
    require(len(v2_runs) == 3240 and len(aligned_runs) == 3240,
            "Both production phases must contain 3,240 paired configs")
    key = lambda j: (str(j.get("scenario_id")), int(j["seed"]), int(j["grid"]), str(j["regime"]))
    by_v2 = {key(j): j for j in v2_runs}
    by_aligned = {key(j): j for j in aligned_runs}
    require(len(by_v2) == len(by_aligned) == 3240, "Production scenario IDs must be unique per arm")
    require(set(by_v2) == set(by_aligned), "Production arms do not cover the same scenarios")
    selector_counts: dict[str, int] = {}
    for sid in sorted(by_v2):
        v2, aligned = by_v2[sid], by_aligned[sid]
        for field in ("seed", "grid", "regime", "source_run_id"):
            require(v2.get(field) == aligned.get(field), f"Production pair mismatch {sid}: {field}")
        require(v2.get("input_hashes") == aligned.get("input_hashes"), f"Input hash mismatch: {sid}")
        require(v2.get("expected_pinned_parameters") == aligned.get("expected_pinned_parameters"),
                f"Pin map mismatch: {sid}")
        require(len(v2.get("expected_pinned_parameters", {})) == 49, f"Missing 49 pins: {sid}")
        vcfg, acfg = config_for(v2), config_for(aligned)
        require(normalized_config(vcfg) == normalized_config(acfg),
                f"Unplanned V2/V1Aligned config difference in {sid}")
        regime = v2["regime"]
        expected_v2_method = f"lawnmower_v2_{regime}"
        expected_aligned_method = f"lawnmower_v1_aligned_{regime}"
        expected_partition = "SpawnAwareUniform" if regime == "static" else "ZoneRegionGrowing"
        require(vcfg.get("method_id") == expected_v2_method and vcfg.get("planner_strategy") == "LawnmowerV2",
                f"V2 selector mismatch for {sid}")
        require(acfg.get("method_id") == expected_aligned_method and acfg.get("planner_strategy") == "LawnmowerV1Aligned",
                f"V1Aligned selector mismatch for {sid}")
        require(vcfg.get("partition_strategy") == acfg.get("partition_strategy") == expected_partition,
                f"Expected common partition selector {expected_partition} in {sid}")
        selector_counts[f"{regime}:{expected_partition}"] = selector_counts.get(f"{regime}:{expected_partition}", 0) + 1
    return {"paired_configs_checked": len(v2_runs), "selector_pair_counts": selector_counts,
            "allowed_config_diff_fields": sorted(ALLOWED_CONFIG_DIFFS)}


def validate_aligned_routes(jobs: list[dict[str, Any]]) -> dict[str, Any]:
    route_reports = []
    for job in jobs:
        grid = int(job["grid"])
        rows = read_csv(Path(job["run_dir"]) / "drones.csv")
        require(len(rows) == 4 * 8000, f"V1Aligned full trace row count mismatch: {job['run_id']}")
        by_drone: dict[int, list[dict[str, str]]] = {i: [] for i in range(4)}
        for row in rows:
            d = int(row["drone"])
            require(d in by_drone, f"Unexpected V1Aligned drone id {d} in {job['run_id']}")
            by_drone[d].append(row)
        per_drone = []
        for drone, trace in by_drone.items():
            trace.sort(key=lambda r: int(r["tick"]))
            require(len(trace) == 8000, f"V1Aligned trace incomplete for {job['run_id']} drone {drone}")
            require([int(r["tick"]) for r in trace] == list(range(1, 8001)),
                    f"V1Aligned ticks not contiguous for {job['run_id']} drone {drone}")
            x0, y0 = float(trace[0]["pos_cell_x"]), float(trace[0]["pos_cell_y"])
            expected_right = drone in (1, 2)
            expected_top = drone in (2, 3)
            require((x0 >= grid / 2) == expected_right and (y0 >= grid / 2) == expected_top,
                    f"V1Aligned drone {drone} starts in wrong quadrant in {job['run_id']}")
            first_target = int(trace[0]["target_cell"])
            corner_cells = (0, grid - 1, grid * grid - 1, grid * (grid - 1))
            require(first_target == corner_cells[drone],
                    f"V1Aligned drone {drone} did not target its spawn corner first in {job['run_id']}")
            first_y_sign = 0
            previous_y = float(trace[0]["pos_cell_y"])
            for row in trace[1:]:
                delta = float(row["pos_cell_y"]) - previous_y
                if abs(delta) > 1e-5:
                    first_y_sign = 1 if delta > 0 else -1
                    break
                previous_y = float(row["pos_cell_y"])
            require(first_y_sign == (-1 if expected_top else 1),
                    f"V1Aligned drone {drone} has wrong inward sweep direction in {job['run_id']}")

            lane_summary: dict[str, Any] = {"checked": False}
            if job["regime"] == "static":
                require(int(trace[0]["owned_cells"]) == grid * grid // 4,
                        f"V1Aligned static owner count is not one quarter: {job['run_id']} drone {drone}")
                left = drone in (0, 3)
                bottom = drone in (0, 1)
                xmin, xmax = (0, grid // 2 - 1) if left else (grid // 2, grid - 1)
                ymin, ymax = (0, grid // 2 - 1) if bottom else (grid // 2, grid - 1)
                right_to_left = expected_right
                top_down = expected_top
                expected_cells = []
                y_values = range(ymax, ymin - 1, -1) if top_down else range(ymin, ymax + 1)
                for rank, y in enumerate(y_values):
                    rtl = right_to_left ^ bool(rank & 1)
                    x_values = range(xmax, xmin - 1, -1) if rtl else range(xmin, xmax + 1)
                    expected_cells.extend(y * grid + x for x in x_values)
                target_changes = []
                last_target = None
                for row in trace:
                    target = int(row["target_cell"])
                    if target < 0 or target == last_target:
                        continue
                    target_changes.append(target)
                    last_target = target
                require(target_changes, f"V1Aligned static route has no waypoints: {job['run_id']} drone {drone}")
                expected_len = len(expected_cells)
                for idx, target in enumerate(target_changes):
                    require(target == expected_cells[idx % expected_len],
                            f"V1Aligned static trace violates one-cell spawn-aligned sweep: {job['run_id']} drone {drone} rank {idx}")
                lane_summary = {"checked": True, "target_changes_checked": len(target_changes),
                                "expected_owned_cells": expected_len, "matches_oriented_one_cell_serpentine_prefix": True}
            per_drone.append({"drone": drone, "spawn_quadrant_correct": True,
                              "first_target_cell": first_target, "first_vertical_direction_correct": True,
                              "initial_owned_cell_count": int(trace[0]["owned_cells"]),
                              "one_cell_lane_check": lane_summary})
        route_reports.append({"run_id": job["run_id"], "seed": int(job["seed"]), "grid": grid,
                              "regime": job["regime"], "drones": per_drone})
    return {"runs_checked": len(route_reports), "drones_checked": len(route_reports) * 4,
            "static_one_cell_routes_checked": sum(d["one_cell_lane_check"]["checked"] for r in route_reports for d in r["drones"]),
            "runs": route_reports}


def check_v2_pilot_anchors(v2_pre: dict[str, Any], v2_runtime: dict[str, Any], pilot_root: Path) -> dict[str, Any]:
    pilot_manifest_path = pilot_root / "manifest_runtime.json"
    pilot = read_json(pilot_manifest_path)
    require(pilot.get("overall_status") == "passed" and len(pilot.get("runs", [])) == 62,
            "Finished V2 pilot is not the expected 62-run passed source")
    pilot_snapshot = Path(pilot.get("run_module_snapshot", ""))
    pilot_module_hash = pilot.get("run_module_sha256")
    require(bool(pilot_module_hash) and pilot.get("run_module_snapshot_sha256") == pilot_module_hash and
            pilot_snapshot.is_file() and sha(pilot_snapshot) == pilot_module_hash,
            "Finished pilot module snapshot/hash is not intact")
    pilot_jobs = {(int(j["seed"]), int(j["grid"]), j["regime"]): j
                  for j in pilot["runs"] if not j.get("repeat")}
    fresh, _ = parity_groups(v2_pre, v2_runtime)
    anchors = []
    for grid in (100, 300):
        for regime in ("static", "dynamic"):
            key = (27, grid, regime)
            current = next((j for j in fresh if run_key(j) == key), None)
            baseline = pilot_jobs.get(key)
            require(current is not None and baseline is not None, f"Missing V2/pilot anchor {key}")
            require(baseline.get("status") == "runtime_evidence_passed", f"Pilot anchor did not pass: {baseline.get('run_id')}")
            check_artifacts(baseline)
            anchors.append(compare_csv_jobs(current, baseline, f"V2 final build to finished pilot {key}"))
    return {"pilot_manifest_path": str(pilot_manifest_path.resolve()),
            "pilot_manifest_sha256": sha(pilot_manifest_path),
            "pilot_run_module_sha256": pilot.get("run_module_sha256"),
            "anchor_runs_compared": len(anchors), "csv_comparisons": anchors}


def find_controller_manifest(root: Path) -> Path:
    return json_path(root / "manifest_preexecution.json", root / "controller_manifest_preexecution.json")


def validate(controller_root: Path, pilot_root: Path, v2_root: Path, aligned_root: Path) -> dict[str, Any]:
    controller_pre_path = find_controller_manifest(controller_root)
    controller_pre = read_json(controller_pre_path)
    validation_execution_path = controller_root / "validation_execution.json"
    require(validation_execution_path.is_file(), f"Validation execution record missing: {validation_execution_path}")
    validation_execution = read_json(validation_execution_path)
    validation_status = validation_execution.get("overall_status", validation_execution.get("status"))
    if validation_status is not None:
        require(validation_status in ("passed", "validation_passed", "complete_passed"),
                f"Validation execution is not passed: {validation_status}")

    v2_pre_path, v2_pre, v2_runtime_path, v2_runtime = manifest_pair(v2_root)
    aligned_pre_path, aligned_pre, aligned_runtime_path, aligned_runtime = manifest_pair(aligned_root)
    require(v2_pre.get("arm_id", v2_pre.get("arm")) == "v2" and
            aligned_pre.get("arm_id", aligned_pre.get("arm")) == "v1_aligned",
            "Arm manifest identities do not match expected V2/V1Aligned roles")
    v2_module = str(v2_pre.get("module_sha256", ""))
    aligned_module = str(aligned_pre.get("module_sha256", ""))
    require(v2_module and v2_module == aligned_module, "Arm module hashes differ or are missing")
    dll = PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"
    require(dll.is_file() and sha(dll) == v2_module, "Current compiled module differs from validated arm module")
    require(controller_pre.get("prepared_module_sha256") == v2_module,
            "Controller preparation is bound to a different module hash")
    require(v2_pre.get("source_manifest_sha256") == aligned_pre.get("source_manifest_sha256"),
            "Arms are not bound to the same source manifest")

    arms = controller_pre.get("arms", [])
    if isinstance(arms, list):
        arm_entries = {item.get("arm", item.get("arm_id")): item for item in arms if isinstance(item, dict)}
    elif isinstance(arms, dict):
        arm_entries = arms
    else:
        arm_entries = {}
    for arm_id, actual_root, prepath in (("v2", v2_root, v2_pre_path),
                                         ("v1_aligned", aligned_root, aligned_pre_path)):
        entry = arm_entries.get(arm_id)
        if isinstance(entry, dict):
            recorded_root = entry.get("root") or entry.get("path")
            if recorded_root:
                require(Path(recorded_root).resolve() == actual_root.resolve(), f"Controller arm root mismatch: {arm_id}")
            recorded_hash = (entry.get("arm_preexecution_manifest_sha256") or
                             entry.get("manifest_preexecution_sha256") or entry.get("preexecution_sha256"))
            if recorded_hash:
                require(recorded_hash == sha(prepath), f"Controller arm manifest hash mismatch: {arm_id}")

    require(validation_execution.get("status") == "passed" and
            validation_execution.get("module_sha256") == v2_module,
            "Controller parity execution did not pass on the current arm module")
    for audit_name in ("historical_v1_output_audit_before", "historical_v1_output_audit_after",
                       "provenance_audit_before", "provenance_audit_after"):
        audit = validation_execution.get(audit_name, {})
        require(audit.get("status") == "passed", f"Controller integrity gate failed or missing: {audit_name}")
    exec_arms = validation_execution.get("arms", {})
    for arm_id, execution_key in (("v2", "v2"), ("v1_aligned", "aligned")):
        arm_exec = exec_arms.get(execution_key, {})
        require(arm_exec.get("status") == "passed" and arm_exec.get("module_sha256") == v2_module,
                f"Controller parity execution incomplete for {arm_id}")

    v2_fresh_batch = validate_arm_parity("v2", v2_pre, v2_runtime)
    aligned_fresh_batch = validate_arm_parity("v1_aligned", aligned_pre, aligned_runtime)
    fairness = validate_arm_fairness(v2_pre, aligned_pre)
    aligned_fresh, _ = parity_groups(aligned_pre, aligned_runtime)
    aligned_jobs = [j for j in aligned_fresh if run_key(j) in
                    {(27, grid, regime) for grid in (100, 200, 300) for regime in ("static", "dynamic")}]
    routes = validate_aligned_routes(aligned_jobs)
    pilot_anchors = check_v2_pilot_anchors(v2_pre, v2_runtime, pilot_root)
    return {
        "schema_version": 1,
        "phase": "followup_preproduction_validation",
        "status": "passed",
        "checked_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "controller_preexecution_path": str(controller_pre_path.resolve()),
        "controller_preexecution_sha256": sha(controller_pre_path),
        "validation_execution_path": str(validation_execution_path.resolve()),
        "validation_execution_sha256": sha(validation_execution_path),
        "arm_manifests": {
            "v2": {"root": str(v2_root.resolve()), "preexecution_path": str(v2_pre_path.resolve()),
                   "preexecution_sha256": sha(v2_pre_path), "runtime_path": str(v2_runtime_path.resolve()),
                   "runtime_sha256": sha(v2_runtime_path), "module_sha256": v2_module},
            "v1_aligned": {"root": str(aligned_root.resolve()), "preexecution_path": str(aligned_pre_path.resolve()),
                           "preexecution_sha256": sha(aligned_pre_path), "runtime_path": str(aligned_runtime_path.resolve()),
                           "runtime_sha256": sha(aligned_runtime_path), "module_sha256": aligned_module},
        },
        "current_module_sha256": sha(dll),
        "pilot_anchor_validation": pilot_anchors,
        "fresh_batch_parity": {"v2": v2_fresh_batch, "v1_aligned": aligned_fresh_batch},
        "fair_comparison_config_validation": fairness,
        "v1_aligned_route_validation": routes,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-root", type=Path, default=DEFAULT_CONTROLLER_ROOT)
    parser.add_argument("--pilot-root", type=Path, default=DEFAULT_PILOT_ROOT)
    parser.add_argument("--v2-root", type=Path, default=DEFAULT_V2_ROOT)
    parser.add_argument("--v1-aligned-root", type=Path, default=DEFAULT_ALIGNED_ROOT)
    args = parser.parse_args()
    controller_root = args.controller_root.resolve()
    receipt_path = controller_root / RECEIPT_NAME
    if receipt_path.exists():
        raise FileExistsError(f"Refusing to overwrite immutable validation receipt: {receipt_path}")
    try:
        receipt = validate(controller_root, args.pilot_root.resolve(), args.v2_root.resolve(), args.v1_aligned_root.resolve())
    except Exception as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}, indent=2))
        return 1
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    receipt_path.write_text(json.dumps(receipt, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": receipt["status"], "receipt": str(receipt_path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
