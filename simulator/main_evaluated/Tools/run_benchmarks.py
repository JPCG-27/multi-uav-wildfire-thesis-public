#!/usr/bin/env python3
"""Batch benchmark driver for the Thesis simulator.

Generates one JSON config per method/scenario point, launches headless Unreal
with -SimConfig, and aggregates Saved/Benchmarks/<run_id>/summary.csv.

Main matrix:
  terrain x seed x focus x spread x region regime x algorithm

Use --dry-run to write configs and print commands without launching Unreal.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List

_CSV_FALLBACK_WARNED: set[str] = set()

METHODS = {
    ("mine", "dynamic"): "mine_dynamic",
    ("mine", "static_uniform"): "mine_static",
    ("random", "dynamic"): "random_dynamic",
    ("random", "static_uniform"): "random_static",
    ("oracle", "dynamic"): "oracle_dynamic",
    ("oracle", "static_uniform"): "oracle_static",
}

# v4 is the physical clairvoyant policy: it may use all ground truth and future
# trajectories for planning, but every recorded detection still comes from a
# drone flight and sensor observation. Earlier experimental contracts are not
# resumable as v4.
ORACLE_POLICY_VERSION = 4

TERRAINS = ["flat", "rolling_hills", "foothills", "mountains"]
CAMERA_RENDER_PROFILES = (
    "legacy", "fast_v1", "fast_unshadowed_v1", "fast_unshadowed_v2", "fast_shader_v3",
)


def validate_inprocess_sensor_profile(sensor: str, in_process: bool,
                                     camera_render_profile: str) -> None:
    """YOLO batching is allowed only with the profile validated for reload parity."""
    if in_process and sensor == "yolo" and camera_render_profile != "fast_shader_v3":
        raise SystemExit(
            "YOLO --in-process requires --camera-render-profile fast_shader_v3; "
            "legacy and older fast profiles have not passed image-parity checks "
            "across level reloads. Run them with fresh Unreal processes."
        )


@dataclass(frozen=True)
class RunPlan:
    index: int
    run_id: str
    run_dir: str
    cfg_path: Path
    cmd: List[str]
    method_id: str
    terrain: str
    flora: str
    focus: int
    spread: int
    seed: int
    grid: int
    drone_processing: str
    scenario_key: str
    expected_final_tick: int
    requested_config: Dict
    map_name: str


@dataclass
class ActiveRun:
    plan: RunPlan
    process: subprocess.Popen
    started_at: float


def parse_int_list(raw) -> List[int]:
    if isinstance(raw, (list, tuple)):
        raw = " ".join(str(piece) for piece in raw)
    values: List[int] = []
    for piece in raw.replace(",", " ").split():
        if piece:
            values.append(int(piece))
    return values


def persons_for_grid(grid: int, baseline: int = 20) -> int:
    return max(1, round(baseline * (grid / 200.0) ** 2))


def path_token(value) -> str:
    text = str(value)
    return text.replace("-", "m").replace(".", "p").replace("+", "")


def normalize_output_namespace(raw: str) -> str:
    namespace = raw.strip()
    if not namespace:
        return ""
    if namespace in {".", ".."} or not re.fullmatch(r"[A-Za-z0-9_.-]+", namespace):
        raise SystemExit(
            "--output-namespace must be one safe directory name using only "
            "letters, numbers, '.', '_' or '-'"
        )
    return namespace


def normalize_vision_cache_root(raw: str) -> str:
    """Accept only relative directory components beneath Saved/VisionCache."""
    parts = raw.strip().replace("\\", "/").split("/")
    reserved = {"CON", "PRN", "AUX", "NUL"} | {f"{kind}{i}" for kind in ("COM", "LPT") for i in range(1, 10)}
    if any(not re.fullmatch(r"[A-Za-z0-9_-][A-Za-z0-9_.-]*", part)
           or part.endswith(".") or part.upper().split(".")[0] in reserved for part in parts):
        raise SystemExit("--vision-cache-dir must be a relative root under Saved/VisionCache "
                         "using safe directory names; absolute paths and '..' are not allowed")
    return "/".join(parts)


# These do not change captured pixels, planner actions, or inference requests.
# Everything else is included conservatively, including the model metadata tag.
VISION_CACHE_IGNORED_CONFIG_KEYS = frozenset({
    "run_id", "benchmark_run_subdir", "vision_cache_dir", "vision_cache_mode",
    "record_metrics", "record_wall_clock_timing", "metrics_detail_mode",
    "exit_on_max_ticks", "max_ticks_per_second", "suppress_benchmark_viewport",
    "save_yolo_debug_images", "yolo_max_error_images", "yolo_max_correct_images",
    "recall_deadline_ticks", "danger_exposure_threshold", "tracking_window_ticks",
    "grid_stats_every_n_ticks", "grid_stats_readback_lag_ticks",
})


def make_vision_cache_dir(config: Dict, map_name: str, namespace: str, explicit_root: str | None) -> str:
    root = (normalize_vision_cache_root(explicit_root) if explicit_root is not None
            else f"benchmarks/{normalize_output_namespace(namespace) or 'default'}")
    identity = {"schema_version": 1, "map": map_name,
                "config": {key: value for key, value in config.items()
                           if key not in VISION_CACHE_IGNORED_CONFIG_KEYS}}
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(",", ":"),
                                       allow_nan=False).encode("utf-8")).hexdigest()[:24]
    # Keep paths below Windows' traditional path limit for ordinary run names.
    # The readable run id separates methods/scenarios; the digest separates knobs.
    return f"{root}/{config['run_id']}/{digest}"


def scenario_cache_paths(args, terrain: str, focus: int, spread: int, seed: int, grid: int, flora: str, human_count: int) -> tuple[str, str, str, str]:
    arrival_minutes_per_tick = args.arrival_minutes_per_tick
    if arrival_minutes_per_tick is None:
        arrival_minutes_per_tick = args.seconds_per_tick / 60.0
    cache_dir = Path("Saved") / "Benchmarks" / "_scenarios" / \
        f"terrain={terrain}" / f"flora={flora}" / f"focus=f{focus}" / \
        f"spread=v{spread}" / f"seed=s{seed}" / f"grid=g{grid}" / \
        f"ticks={args.max_ticks}" / f"spt={path_token(args.seconds_per_tick)}" / \
        f"arrival={path_token(arrival_minutes_per_tick)}" / \
        f"cell={path_token(args.cell_size_uu)}" / f"humans={human_count}"
    return (
        (cache_dir / "fire.csv").as_posix(),
        (cache_dir / "humans.csv").as_posix(),
        (cache_dir / "terrain.csv").as_posix(),
        (cache_dir / "fuel.csv").as_posix(),
    )


def method_ids(algorithms: Iterable[str], regimes: Iterable[str]) -> List[str]:
    out: List[str] = []
    for regime in regimes:
        normalized_regime = "static_uniform" if regime in {"static", "static_uniform"} else "dynamic"
        for algorithm in algorithms:
            key = (algorithm, normalized_regime)
            if key not in METHODS:
                raise SystemExit(f"unsupported algorithm/regime: {algorithm}/{regime}")
            out.append(METHODS[key])
    return out


def make_run_id(method_id: str, terrain: str, focus: int, spread: int, seed: int, grid: int, flora: str, drone_processing: str) -> str:
    suffix = f"{method_id}_{terrain}_f{focus}_v{spread}_s{seed}_g{grid}"
    if flora != "medium":
        suffix += f"_{flora}"
    if drone_processing != "sequential":
        suffix += f"_{drone_processing}"
    return suffix


def make_run_subdir(method_id: str, terrain: str, focus: int, spread: int, seed: int, grid: int, flora: str, drone_processing: str) -> str:
    path = Path("runs") / f"terrain={terrain}" / f"flora={flora}" / \
        f"focus=f{focus}" / f"spread=v{spread}" / f"seed=s{seed}" / \
        f"grid=g{grid}"
    if drone_processing != "sequential":
        path /= f"drone={drone_processing}"
    path /= f"method={method_id}"
    if method_id.startswith("oracle_"):
        path /= f"oracle_policy=v{ORACLE_POLICY_VERSION}"
    return str(path).replace("\\", "/")


def make_config(args, method_id: str, terrain: str, focus: int, spread: int, seed: int, grid: int, flora: str, drone_processing: str) -> Dict:
    run_id = make_run_id(method_id, terrain, focus, spread, seed, grid, flora, drone_processing)
    run_subdir = make_run_subdir(method_id, terrain, focus, spread, seed, grid, flora, drone_processing)
    human_count = args.humans if args.humans > 0 else persons_for_grid(grid)
    render_terrain_visuals = args.render_terrain_visuals
    if render_terrain_visuals is None:
        render_terrain_visuals = args.sensor == "yolo"
    arrival_minutes_per_tick = args.arrival_minutes_per_tick
    if arrival_minutes_per_tick is None:
        arrival_minutes_per_tick = args.seconds_per_tick / 60.0
    fire_cache_path, humans_cache_path, terrain_cache_path, fuel_cache_path = scenario_cache_paths(
        args, terrain, focus, spread, seed, grid, flora, human_count)
    config = {
        "run_id": run_id,
        "benchmark_run_subdir": run_subdir,
        "method_id": method_id,
        "oracle_policy_version": ORACLE_POLICY_VERSION,
        "max_sim_ticks": args.max_ticks,
        "exit_on_max_ticks": True,
        "record_metrics": True,
        "suppress_benchmark_viewport": not args.render_benchmark_viewport,
        "max_ticks_per_second": 0,
        "seconds_per_tick": args.seconds_per_tick,
        "arrival_minutes_per_tick": arrival_minutes_per_tick,
        "grid_width": grid,
        "grid_height": grid,
        "cell_size_uu": args.cell_size_uu,
        "drone_count": args.drones,
        "drone_speed_uu_per_tick": args.drone_speed_uu_per_tick,
        "parallel_drone_processing": drone_processing == "parallel_cpu",
        "drone_processing_mode": drone_processing,
        "drone_max_view_distance_uu": args.drone_max_view_distance_uu,
        "terrain_profile": terrain,
        "render_terrain_visuals": render_terrain_visuals,
        "flora_density": flora,
        "fire_seed": seed,
        "fire_generator_num_focus_points": focus,
        "fire_generator_spread_variant": spread,
        "reuse_generated_scenario_csvs": args.scenario_cache,
        "generated_fire_csv_output_path": fire_cache_path if args.scenario_cache else "Saved/GeneratedFire.csv",
        "generated_humans_csv_output_path": humans_cache_path if args.scenario_cache else "Saved/GeneratedHumans.csv",
        "generated_terrain_csv_output_path": terrain_cache_path if args.scenario_cache else "Saved/GeneratedTerrain.csv",
        "generated_fuel_csv_output_path": fuel_cache_path if args.scenario_cache else "Saved/GeneratedFuel.csv",
        "fire_csv_path": fire_cache_path if args.scenario_cache else "",
        "humans_csv_path": humans_cache_path if args.scenario_cache else "",
        "generate_fire_csv": True,
        "auto_generate_scenario": True,
        "human_count": human_count,
        "human_max_ticks": args.max_ticks,
        "human_seed": seed,
        "human_mesh_scale": args.human_mesh_scale,
        "human_visual_mesh_set": getattr(args, "human_visual_mesh_set", "mixed"),
        "human_pose_mode": getattr(args, "human_pose_mode", "automatic"),
        "partition_recompute_every_n_ticks": args.partition_recompute_every_n_ticks,
        "grid_stats_every_n_ticks": args.grid_stats_every_n_ticks,
        "grid_stats_readback_lag_ticks": args.grid_stats_readback_lag_ticks,
        "recall_deadline_ticks": args.recall_deadline_ticks,
        "danger_exposure_threshold": args.danger_threshold,
        "tracking_window_ticks": args.tracking_window_ticks,
        "record_wall_clock_timing": args.record_wall_clock_timing,
        "metrics_detail_mode": args.metrics_detail,
        "enable_drone_captures": args.sensor == "yolo",
        "capture_resolution": args.drone_capture_resolution,
        "enable_vision_bridge": args.sensor == "yolo",
        "vision_server_host": args.vision_server_host,
        "vision_server_port": args.vision_server_port,
        "vision_cache_mode": args.vision_cache_mode,
        "vision_model_tag": args.vision_model_tag,
        "save_yolo_debug_images": args.save_yolo_debug_images,
        "yolo_max_error_images": args.yolo_max_error_images,
        "yolo_max_correct_images": args.yolo_max_correct_images,
        "yolo_person_confidence_threshold": args.yolo_person_confidence_threshold,
        "drone_capture_auto_exposure": args.drone_capture_auto_exposure,
        "drone_capture_exposure_bias": args.drone_capture_exposure_bias,
        "drone_capture_manual_ev100": args.drone_capture_manual_ev100,
        "drone_capture_bloom_intensity": args.drone_capture_bloom_intensity,
        "save_drone_capture_pngs": args.save_drone_capture_pngs,
        "drone_capture_pitch_degrees": args.drone_capture_pitch_degrees,
        "drone_fov_degrees": args.drone_fov_degrees,
    }
    # Keep legacy configs byte-compatible with saved requests created before
    # camera render profiles existed. UE treats an omitted field as legacy.
    camera_render_profile = getattr(args, "camera_render_profile", "legacy")
    if camera_render_profile != "legacy":
        config["camera_render_profile"] = camera_render_profile
    if args.sensor == "yolo":
        # Version 6 counts GT-in-view by projected alive-person anchors and
        # filters the ground footprint through the true camera frustum.
        # Version 5 adds tick-driven human poses and the Mixamo/mannequin pool.
        # Version 4 pins HISM foliage to LOD0 for repeatable foliage edges.
        # Version 3 uses the mesh/texture/shadow profile, fixed sky time and
        # deterministic fire copies; sensor DFAO/refraction are disabled.
        # Changed pixels can change detections/trajectories; old recordings and
        # completion receipts must not satisfy the new sensor configuration.
        config["vision_sensor_contract_version"] = 6
        config["yolo_online_metric_version"] = "geometric_alive_one_to_one_cell_neighborhood_v2"
        # Presentation changes also invalidate recorded pixels and detections.
        config["environment_presentation_version"] = 12
    if args.num_real_camera_drones is not None:
        config["num_real_camera_drones"] = args.num_real_camera_drones
    elif args.sensor == "yolo":
        config["num_real_camera_drones"] = args.drones
    config["vision_cache_dir"] = make_vision_cache_dir(config, args.map, args.output_namespace, args.vision_cache_dir)
    return config


def build_command(args, cfg_path: Path) -> List[str]:
    cmd = [args.editor, args.uproject]
    if args.map:
        cmd.append(args.map)
    cmd += [
        "-game",
        "-RenderOffscreen",
        "-unattended",
        "-nosplash",
        "-nosound",
        f"-SimConfig={cfg_path}",
    ]
    if args.suppress_ue_logs:
        cmd += [
            "-nolog",
            "-abslog=NUL",
            "-LogCmds=LogTemp off",
        ]
    if args.nullrhi:
        cmd.append("-nullrhi")
    return cmd


def build_batch_command(args, manifest_path: Path) -> List[str]:
    """Like build_command but runs a whole -SimConfigManifest batch in one process."""
    cmd = [args.editor, args.uproject]
    if args.map:
        cmd.append(args.map)
    cmd += [
        "-game",
        "-RenderOffscreen",
        "-unattended",
        "-nosplash",
        "-nosound",
        f"-SimConfigManifest={manifest_path}",
    ]
    if getattr(args, "stop_file", None):
        cmd.append(f"-SimBatchStopFile={args.stop_file}")
    if args.suppress_ue_logs:
        cmd += [
            "-nolog",
            "-abslog=NUL",
            "-LogCmds=LogTemp off",
        ]
    if args.nullrhi:
        cmd.append("-nullrhi")
    return cmd


def summary_path(bench_dir: Path, plan: RunPlan) -> Path:
    return bench_dir / plan.run_dir / "summary.csv"


# These affect output location or presentation, not the recorded experiment.
# Timing/detail and debug-image requests remain significant: a summary-only run
# must not satisfy a later request for trajectories, profiling, or capture files.
RESUME_IGNORED_CONFIG_KEYS = frozenset({
    "run_id", "benchmark_run_subdir", "exit_on_max_ticks",
    "max_ticks_per_second", "suppress_benchmark_viewport",
})


def resume_signature(config: Dict, map_name: str) -> Dict:
    ignored_keys = RESUME_IGNORED_CONFIG_KEYS
    if config.get("record_wall_clock_timing", False):
        # A viewport or tick-rate change invalidates a performance measurement.
        ignored_keys = ignored_keys - {"max_ticks_per_second", "suppress_benchmark_viewport"}
    return {
        "schema_version": 1,
        "map": map_name,
        "config": {key: value for key, value in config.items()
                   if key not in ignored_keys},
    }


def windows_extended_path(path: Path) -> Path:
    if sys.platform != "win32":
        return path
    # Windows' legacy MAX_PATH limit includes the terminating NUL, so a normal
    # path of 260 characters fails even when its parent directories exist.
    absolute = os.path.abspath(os.fspath(path))
    if len(absolute) < 260 or absolute.startswith("\\\\?\\"):
        return path
    if absolute.startswith("\\\\"):
        absolute = "\\\\?\\UNC\\" + absolute[2:]
    else:
        absolute = "\\\\?\\" + absolute
    return Path(absolute)


def request_receipt_path(bench_dir: Path, plan: RunPlan) -> Path:
    # Only Python's run receipt reaches MAX_PATH in the frozen matrix. Unreal
    # continues to receive ordinary paths for its shorter config and summary.
    return windows_extended_path(bench_dir / plan.run_dir / "benchmark_request.json")


def has_compatible_request(bench_dir: Path, plan: RunPlan) -> bool:
    # Never compare against _configs: planning rewrites those files even for a
    # dry run. This receipt belongs to the execution that produced summary.csv.
    try:
        receipt = json.loads(request_receipt_path(bench_dir, plan).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return (isinstance(receipt, dict) and receipt.get("status") == "complete"
            and receipt.get("request") == resume_signature(plan.requested_config, plan.map_name))


def prepare_run_outputs(bench_dir: Path, plan: RunPlan) -> None:
    # Invalidate the old completion BEFORE recording the new request. Otherwise
    # a failed launch could pair an old same-horizon summary with a new config.
    summary_path(bench_dir, plan).unlink(missing_ok=True)
    receipt = request_receipt_path(bench_dir, plan)
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_text(json.dumps({"status": "running", "request": resume_signature(
        plan.requested_config, plan.map_name)}, indent=2, sort_keys=True), encoding="utf-8")


def read_summary_row(bench_dir: Path, plan: RunPlan, *, require_completed_request: bool = True) -> Dict[str, str] | None:
    if require_completed_request and not has_compatible_request(bench_dir, plan):
        return None
    path = summary_path(bench_dir, plan)
    if not path.exists() or path.stat().st_size <= 0:
        return None
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    row = rows[0]
    try:
        final_tick = int(row.get("final_tick", ""))
    except (TypeError, ValueError):
        return None
    if final_tick != plan.expected_final_tick:
        return None
    row["run_id"] = plan.run_id
    row["run_dir"] = plan.run_dir
    return row


def complete_run_outputs(bench_dir: Path, plan: RunPlan) -> Dict[str, str] | None:
    """Called only after a zero process exit; EndPlay alone is not success."""
    row = read_summary_row(bench_dir, plan, require_completed_request=False)
    if row is not None:
        request_receipt_path(bench_dir, plan).write_text(json.dumps({
            "status": "complete", "request": resume_signature(plan.requested_config, plan.map_name),
        }, indent=2, sort_keys=True), encoding="utf-8")
    return row


def write_csv_atomic(path: Path, rows: List[Dict[str, str]], preferred_fields: List[str]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(preferred_fields)
    seen = set(fieldnames)
    for row in rows:
        for key in row:
            if key not in seen:
                fieldnames.append(key)
                seen.add(key)

    tmp_path = path.with_name(f"{path.name}.{time.time_ns()}.tmp")
    with tmp_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        # Flush contents before replacement to reduce progress-file loss on restart.
        f.flush()
        os.fsync(f.fileno())

    for attempt in range(20):
        try:
            tmp_path.replace(path)
            return path
        except PermissionError:
            if attempt == 19:
                # A reader may hold the public index without Windows delete sharing.
                # Keep progress durable without stopping simulations or touching that reader.
                if path.name in {"run_index.csv", "aggregate.csv"}:
                    fallback = path.parent / "_progress_snapshots" / path.name
                    fallback.parent.mkdir(parents=True, exist_ok=True)
                    try:
                        tmp_path.replace(fallback)
                    except PermissionError:
                        # Even the fallback can be held by a progress reader. A unique
                        # snapshot needs no replacement and keeps workers running.
                        fallback = fallback.with_name(f"{path.stem}.{time.time_ns()}.csv")
                        tmp_path.replace(fallback)
                        snapshots = sorted(fallback.parent.glob(f"{path.stem}.*.csv"),
                                           key=lambda item: item.stat().st_mtime_ns)
                        for old_snapshot in snapshots[:-2]:
                            try:
                                old_snapshot.unlink()
                            except PermissionError:
                                pass  # A reader will eventually release this old snapshot.
                    except OSError:
                        tmp_path.unlink(missing_ok=True)
                        raise
                    if str(path) not in _CSV_FALLBACK_WARNED:
                        print(f"WARNING: progress file locked: {path}; current progress saved to {fallback}", flush=True)
                        _CSV_FALLBACK_WARNED.add(str(path))
                    return fallback
                try:
                    tmp_path.unlink(missing_ok=True)
                finally:
                    raise
            time.sleep(0.1)
    return path


def write_aggregate(bench_dir: Path, planned: List[RunPlan], rows_by_run_id: Dict[str, Dict[str, str]]) -> Path:
    rows = [rows_by_run_id[p.run_id] for p in planned if p.run_id in rows_by_run_id]
    return write_csv_atomic(bench_dir / "aggregate.csv", rows, ["run_id", "run_dir"])


def write_run_index(bench_dir: Path, planned: List[RunPlan], statuses: Dict[str, str]) -> Path:
    rows: List[Dict[str, str]] = []
    for plan in planned:
        summary = summary_path(bench_dir, plan)
        rows.append({
            "index": str(plan.index),
            "status": statuses.get(plan.run_id, "pending"),
            "run_id": plan.run_id,
            "run_dir": plan.run_dir,
            "summary_path": str(summary),
            "config_path": str(plan.cfg_path),
            "method_id": plan.method_id,
            "terrain": plan.terrain,
            "flora": plan.flora,
            "focus": str(plan.focus),
            "spread": str(plan.spread),
            "seed": str(plan.seed),
            "grid": str(plan.grid),
            "drone_processing": plan.drone_processing,
            "scenario_key": plan.scenario_key,
            "expected_final_tick": str(plan.expected_final_tick),
        })
    return write_csv_atomic(
        bench_dir / "run_index.csv",
        rows,
        [
            "index", "status", "run_id", "run_dir", "summary_path", "config_path",
            "method_id", "terrain", "flora", "focus", "spread", "seed", "grid", "drone_processing", "scenario_key",
            "expected_final_tick",
        ],
    )


def duplicate_values(values: Iterable[str]) -> List[str]:
    seen = set()
    duplicates = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
        else:
            seen.add(value)
    return sorted(duplicates)


def compact_values(values: Iterable, limit: int = 12) -> str:
    unique = sorted({str(value) for value in values})
    if not unique:
        return "-"
    if len(unique) <= limit:
        return " ".join(unique)
    return f"{len(unique)} values ({unique[0]} .. {unique[-1]})"


def make_inprocess_batches(plans: List[RunPlan], batch_size: int) -> List[List[RunPlan]]:
    # Keep runs that share a scenario cache in one process (no cross-process
    # cache race), then pack those groups into same-grid batches of <= batch-size.
    # Groups arrive interleaved by grid in the full sweep, so keep one open
    # batch per grid instead of flushing at every grid change. This amortises
    # editor startup without ever mixing grid dimensions in a manifest.
    groups: Dict[str, List[RunPlan]] = {}
    group_order: List[str] = []
    for plan in plans:
        if plan.scenario_key not in groups:
            group_order.append(plan.scenario_key)
            groups[plan.scenario_key] = []
        groups[plan.scenario_key].append(plan)

    batches: List[List[RunPlan]] = []
    open_by_grid: Dict[int, List[RunPlan]] = {}
    grid_order: List[int] = []
    cap = max(1, batch_size)
    for scenario_key in group_order:
        group = groups[scenario_key]
        group_grid = group[0].grid
        if any(plan.grid != group_grid for plan in group):
            raise ValueError(f"scenario_key spans multiple grid sizes: {scenario_key}")

        if group_grid not in open_by_grid:
            open_by_grid[group_grid] = []
            grid_order.append(group_grid)

        current = open_by_grid[group_grid]
        if current and len(current) + len(group) > cap:
            batches.append(current)
            current = []
            open_by_grid[group_grid] = current

        if len(group) > cap:
            batches.append(list(group))
            continue

        current.extend(group)
        if len(current) == cap:
            batches.append(current)
            open_by_grid[group_grid] = []

    for grid in grid_order:
        current = open_by_grid.get(grid, [])
        if current:
            batches.append(current)
    return batches


def validate_inprocess_batches(plans: List[RunPlan], batches: List[List[RunPlan]]) -> Dict[str, object]:
    expected_run_ids = [p.run_id for p in plans]
    assigned_run_ids = [p.run_id for batch in batches for p in batch]
    expected = set(expected_run_ids)
    assigned = set(assigned_run_ids)

    scenario_to_batch: Dict[str, int] = {}
    split_scenarios = set()
    for batch_idx, batch in enumerate(batches):
        for scenario_key in {p.scenario_key for p in batch}:
            previous = scenario_to_batch.get(scenario_key)
            if previous is not None and previous != batch_idx:
                split_scenarios.add(scenario_key)
            scenario_to_batch[scenario_key] = batch_idx

    duplicate_assigned = duplicate_values(assigned_run_ids)
    missing = sorted(expected - assigned)
    extra = sorted(assigned - expected)
    assignment_ok = (
        len(assigned_run_ids) == len(expected_run_ids)
        and not duplicate_assigned
        and not missing
        and not extra
    )
    same_grid_ok = all(len({p.grid for p in batch}) <= 1 for batch in batches)
    scenario_group_ok = not split_scenarios
    return {
        "assignment_ok": assignment_ok,
        "same_grid_ok": same_grid_ok,
        "scenario_group_ok": scenario_group_ok,
        "duplicate_assigned": duplicate_assigned,
        "missing": missing,
        "extra": extra,
        "split_scenarios": sorted(split_scenarios),
    }


def print_plan_summary(args, planned: List[RunPlan], statuses: Dict[str, str]) -> None:
    pending = [p for p in planned if args.force or statuses.get(p.run_id) != "complete"]
    scenario_count = len({p.scenario_key for p in planned})
    pending_scenario_count = len({p.scenario_key for p in pending})
    print("\nPlan check:")
    print(f"  unique run_id values: {len({p.run_id for p in planned})}/{len(planned)}")
    print(f"  scenario groups:      {scenario_count} total, {pending_scenario_count} pending")
    print(f"  seeds:                {compact_values(p.seed for p in planned)}")
    print(f"  grids:                {compact_values(p.grid for p in planned)}")
    print(f"  terrains:             {compact_values(p.terrain for p in planned)}")
    print(f"  methods:              {compact_values(p.method_id for p in planned)}")
    print(f"  duplicate run_ids:    none")

    if not args.in_process:
        if args.jobs > 1:
            print(f"  jobs scheduler:       up to {args.jobs} standalone process(es); same scenario_key is serialized")
        return

    batches = make_inprocess_batches(pending, args.batch_size)
    validation = validate_inprocess_batches(pending, batches)
    print(f"  in-process:           {len(pending)} pending run(s) -> {len(batches)} batch(es)")
    print(f"  concurrent batches:   up to {args.jobs}")
    print(f"  max batch size:       {max(1, args.batch_size)}")
    print(f"  batch assignment:     {'PASS' if validation['assignment_ok'] else 'FAIL'} (each pending run exactly once)")
    print(f"  scenario grouping:    {'PASS' if validation['scenario_group_ok'] else 'FAIL'} (same scenario_key never split)")
    print(f"  same-grid batches:    {'PASS' if validation['same_grid_ok'] else 'FAIL'}")
    if not validation["assignment_ok"]:
        print(f"  duplicate assigned:   {compact_values(validation['duplicate_assigned'][:8])}")
        print(f"  missing assigned:     {compact_values(validation['missing'][:8])}")
        print(f"  extra assigned:       {compact_values(validation['extra'][:8])}")
    if not validation["scenario_group_ok"]:
        print(f"  split scenarios:      {compact_values(validation['split_scenarios'][:8])}")
    if batches:
        sizes = [len(batch) for batch in batches]
        scenario_sizes = [len({p.scenario_key for p in batch}) for batch in batches]
        print(f"  batch runs min/max:   {min(sizes)}/{max(sizes)}")
        print(f"  batch scen min/max:   {min(scenario_sizes)}/{max(scenario_sizes)}")
        shown = 0
        for idx, batch in enumerate(batches, 1):
            if len(batches) > 16 and idx > 8 and idx <= len(batches) - 3:
                if shown == 8:
                    print(f"  ... {len(batches) - 11} batch(es) omitted ...")
                    shown += 1
                continue
            print(
                f"  batch {idx:>4}: {len(batch):>3} run(s), "
                f"{len({p.scenario_key for p in batch}):>3} scenario(s), "
                f"seeds {compact_values((p.seed for p in batch), limit=6)}, "
                f"grids {compact_values((p.grid for p in batch), limit=6)}, "
                f"terrains {compact_values((p.terrain for p in batch), limit=6)}"
            )
            shown += 1


def run_command(cmd: List[str], timeout: int) -> int:
    process = subprocess.Popen(cmd)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        raise
    except KeyboardInterrupt:
        process.terminate()
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise


def stop_processes(processes: Iterable[subprocess.Popen]) -> None:
    """Stop active Unreal processes when the user interrupts a sweep."""
    active = [process for process in processes if process.poll() is None]
    for process in active:
        try:
            process.terminate()
        except OSError:
            # It may have exited between poll() and terminate().
            pass
    for process in active:
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except OSError:
                pass
            process.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--editor", required=True, help="Path to UnrealEditor-Cmd.exe")
    parser.add_argument("--uproject", required=True, help="Path to Thesis.uproject")
    parser.add_argument("--map", default="NewMap")
    parser.add_argument("--seeds", nargs="+", default=[str(27 + i) for i in range(25)],
                        help="Space/comma-separated seeds")
    parser.add_argument("--terrains", nargs="+", default=TERRAINS)
    parser.add_argument("--focus-points", nargs="+", default=["1", "2", "3"], help="Space/comma-separated focus counts")
    parser.add_argument("--spread-variants", nargs="+", default=["0", "1", "2"], help="Space/comma-separated spread variants")
    parser.add_argument("--algorithms", nargs="+", default=["mine", "random", "oracle"])
    parser.add_argument("--regimes", nargs="+", default=["static_uniform", "dynamic"])
    parser.add_argument("--max-ticks", type=int, default=8000,
                        help="Simulation horizon. Default 8000 with --seconds-per-tick 3.6 = 8 hours.")
    parser.add_argument("--seconds-per-tick", type=float, default=3.6,
                        help="Physical duration represented by one simulation tick.")
    parser.add_argument("--arrival-minutes-per-tick", type=float, default=None,
                        help="Fire CSV minutes per tick. Default = seconds_per_tick / 60.")
    parser.add_argument("--drones", type=int, default=4)
    parser.add_argument("--grid", nargs="+", type=int, default=[200],
                        help="One or more grid sizes, e.g. --grid 100 200 300.")
    parser.add_argument("--cell-size-uu", type=float, default=3600.0,
                        help="Default 3600 UU = 36 m, matched to the 8-hour calibration.")
    parser.add_argument("--humans", type=int, default=0, help="0 = scale with grid area")
    parser.add_argument("--human-mesh-scale", type=float, default=1.0,
                        help="Visual-only scale for human meshes in drone captures.")
    parser.add_argument("--human-visual-mesh-set", choices=["mixed", "mixamo", "mannequin"], default="mixed")
    parser.add_argument("--human-pose-mode", choices=["automatic", "varied", "standing", "walking", "running",
                                                   "waving", "crouching", "seated", "lying"], default="automatic")
    parser.add_argument("--flora", nargs="+", choices=["none", "sparse", "medium", "dense"], default=["medium"],
                        help="One or more flora densities to run. Use 'none medium' for without/with-flora sweeps.")
    parser.add_argument("--sensor", choices=["perfect", "yolo"], default="perfect")
    parser.add_argument("--num-real-camera-drones", type=int, default=None,
                        help="Number of drones with a real camera. YOLO defaults to all drones; "
                             "set a smaller number only for an explicit mixed-sensor experiment.")
    parser.add_argument("--drone-capture-resolution", type=int, default=640,
                        help="Square camera capture size in pixels (default: 640).")
    parser.add_argument("--camera-render-profile", choices=CAMERA_RENDER_PROFILES, default="legacy",
                        help="Camera-only rendering profile (default: legacy). Fast profiles change captured appearance.")
    parser.add_argument("--render-benchmark-viewport", action="store_true",
                        help="Render the overview viewport/HUD during benchmark runs. By default only "
                             "explicit drone captures render; GPU belief compute remains enabled.")
    parser.add_argument("--render-terrain-visuals", dest="render_terrain_visuals", action="store_true",
                        default=None, help="Render the terrain mesh even in perfect-sensor runs.")
    parser.add_argument("--no-render-terrain-visuals", dest="render_terrain_visuals", action="store_false",
                        help="Hide the terrain mesh from all cameras while keeping terrain data active.")
    parser.add_argument("--nullrhi", action="store_true",
                        help="Disable the RHI. Do not use for normal benchmarks: the belief pipeline dispatches GPU compute shaders.")
    parser.add_argument("--vision-cache-mode", choices=["off", "record", "replay"], default="replay")
    parser.add_argument("--vision-server-host", default="127.0.0.1")
    parser.add_argument("--vision-server-port", type=int, default=8765)
    parser.add_argument("--vision-cache-dir", default=None,
                        help="Relative cache root beneath Saved/VisionCache; method/scenario/hash subfolders "
                             "are always appended. Default: benchmarks/<output-namespace or default>. "
                             "Use the same explicit root to replay from another output namespace.")
    parser.add_argument("--vision-model-tag", default="yolov8n",
                        help="Metadata label included in cache identity (default: yolov8n). This does not "
                             "configure the vision server; match its model/settings and change the tag "
                             "when weights or inference settings change.")
    parser.add_argument("--save-yolo-debug-images", dest="save_yolo_debug_images", action="store_true", default=True,
                        help="Save raw+YOLO-box+expected-ground-truth images for total YOLO misses (needs Record/Off cache mode).")
    parser.add_argument("--no-yolo-debug-images", dest="save_yolo_debug_images", action="store_false")
    parser.add_argument("--yolo-max-error-images", type=int, default=200,
                        help="Cap on saved total-miss debug frames per run (0 = unlimited).")
    parser.add_argument("--yolo-max-correct-images", type=int, default=0,
                        help="Cap on saved true-positive debug frames per run (0 = disabled).")
    parser.add_argument("--yolo-person-confidence-threshold", type=float, default=0.25)
    parser.add_argument("--drone-capture-auto-exposure", dest="drone_capture_auto_exposure",
                        action="store_true", default=False,
                        help="Opt in to auto-exposure. Default is manual exposure for stable capture brightness.")
    parser.add_argument("--drone-capture-manual-exposure", dest="drone_capture_auto_exposure",
                        action="store_false",
                        help="Use a fixed manual exposure instead; tune with --drone-capture-exposure-bias.")
    parser.add_argument("--drone-capture-exposure-bias", type=float, default=0.0,
                        help="Exposure compensation in EV (lower = darker).")
    parser.add_argument("--drone-capture-bloom-intensity", type=float, default=0.15)
    parser.add_argument("--save-drone-capture-pngs", action="store_true", help="Save every sensor frame for visual diagnostics (adds PNG encoding and disk I/O).")
    parser.add_argument("--drone-capture-manual-ev100", type=float, default=12.0,
                        help="Fixed manual exposure reference; compensation is applied on top.")
    parser.add_argument("--drone-capture-pitch-degrees", type=float, default=-60.0,
                        help="Capture camera pitch; -90 = straight down (less sky in frame).")
    parser.add_argument("--drone-fov-degrees", type=float, default=90.0,
                        help="Horizontal camera/sensor FOV. Lower values are optical zoom, pitch unchanged.")
    parser.add_argument("--drone-speed-uu-per-tick", type=float, default=3600.0,
                        help="Default 3600 UU/tick = 36 m/tick; with 3.6 s/tick this is 10 m/s.")
    parser.add_argument("--drone-max-view-distance-uu", type=float, default=14400.0,
                        help="Default 4 calibrated cells of view range.")
    parser.add_argument("--drone-processing", choices=["sequential", "parallel_cpu"], default="sequential",
                        help="Drone update implementation. sequential is the current baseline.")
    parser.add_argument("--compare-drone-processing", action="store_true",
                        help="Generate both sequential and parallel_cpu variants for each non-oracle run.")
    parser.add_argument("--partition-recompute-every-n-ticks", type=int, default=50)
    parser.add_argument("--grid-stats-every-n-ticks", type=int, default=0,
                        help="GPU grid-stats metrics cadence. Default 0 disables the blocking staleness/coverage readback.")
    parser.add_argument("--grid-stats-readback-lag-ticks", type=int, default=5,
                        help="Ticks to wait between grid-stats dispatch and blocking consumption.")
    parser.add_argument("--recall-deadline-ticks", type=int, default=500)
    parser.add_argument("--danger-threshold", type=float, default=0.5)
    parser.add_argument("--tracking-window-ticks", type=int, default=50)
    parser.add_argument("--record-wall-clock-timing", action="store_true",
                        help="Opt in to volatile wall-clock timing metrics, including per-run phase_timings.csv. "
                             "Leave off for byte-identical gates.")
    parser.add_argument("--profile-functions", action="store_true",
                        help="Shortcut for optimization runs: enables --record-wall-clock-timing so each run writes "
                             "phase_timings.csv and avg/max phase columns in summary.csv.")
    parser.add_argument("--metrics-detail", choices=["full", "summary"], default="full",
                        help="full writes ticks.csv+drones.csv; summary keeps paper summary/person metrics and sparse events only.")
    parser.add_argument("--paper-mode", action="store_true",
                        help="Shortcut for paper throughput runs: summary metrics and no wall-clock timing.")
    parser.add_argument("--jobs", type=int, default=1,
                        help="Number of UnrealEditor-Cmd processes to run concurrently. Use with paper-mode/summary metrics.")
    parser.add_argument("--suppress-ue-logs", dest="suppress_ue_logs", action="store_true", default=True,
                        help="Disable project LogTemp output in benchmark Unreal processes (default).")
    parser.add_argument("--keep-ue-logs", dest="suppress_ue_logs", action="store_false",
                        help="Keep normal Unreal/LogTemp output for debugging benchmark failures.")
    parser.add_argument("--no-scenario-cache", dest="scenario_cache", action="store_false", default=True,
                        help="Disable shared fire/human CSV cache across methods. Default caches each scenario once.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--plan-only", action="store_true",
                        help="Write configs and print a compact uniqueness/batch plan, then exit without launching Unreal.")
    parser.add_argument("--write-manifest", default=None,
                        help="Write the planned config.json paths (one per line) to this file for "
                             "in-process batching (UE -SimConfigManifest), then exit without launching.")
    parser.add_argument("--in-process", action="store_true",
                        help="Run scenarios in batches inside one Unreal process each, reloading the "
                             "level between scenarios to amortise editor startup. Composes with --jobs "
                             "(a rolling pool of that many concurrent batch processes). Validate first "
                             "with Tools/verify_inprocess.py (must be byte-identical).")
    parser.add_argument("--batch-size", type=int, default=64,
                        help="Max scenarios per in-process batch. Runs sharing a scenario cache stay in "
                             "the same batch to avoid cross-process cache races.")
    parser.add_argument("--output-namespace", default="",
                        help="Optional safe subdirectory under Saved/Benchmarks for isolated configs, "
                             "results, aggregate.csv, and run_index.csv. Intended for profiling/job sweeps.")
    parser.add_argument("--stop-file", default=None,
                        help="If this file exists, stop launching new scenarios at the next run boundary; "
                             "active scenarios finish normally and completed receipts remain resumable.")
    parser.add_argument("--fail-fast", action="store_true",
                        help="After the first failed scenario/process, stop launching work and preserve pending runs for resume.")
    parser.add_argument("--force", action="store_true",
                        help="Re-run even if a run's summary.csv already exists. Default skips "
                             "completed runs, so re-running the same command resumes the sweep.")
    parser.add_argument("--timeout", type=int, default=7200)
    args = parser.parse_args()
    if args.vision_cache_dir is not None:
        args.vision_cache_dir = normalize_vision_cache_root(args.vision_cache_dir)
    validate_inprocess_sensor_profile(args.sensor, args.in_process, args.camera_render_profile)
    args.vision_model_tag = args.vision_model_tag.strip()
    if not args.vision_model_tag:
        raise SystemExit("--vision-model-tag must not be empty")
    if not 1 <= args.vision_server_port <= 65535:
        raise SystemExit("--vision-server-port must be between 1 and 65535")
    if args.profile_functions:
        args.record_wall_clock_timing = True
    if args.paper_mode:
        args.metrics_detail = "summary"
        args.record_wall_clock_timing = False
    if args.max_ticks <= 0:
        raise SystemExit("--max-ticks must be positive")
    if args.drone_capture_resolution <= 0:
        raise SystemExit("--drone-capture-resolution must be positive")
    if args.num_real_camera_drones is not None and not 0 <= args.num_real_camera_drones <= args.drones:
        raise SystemExit("--num-real-camera-drones must be between zero and --drones")
    if args.jobs <= 0:
        raise SystemExit("--jobs must be positive")
    if args.seconds_per_tick <= 0:
        raise SystemExit("--seconds-per-tick must be positive")
    if args.arrival_minutes_per_tick is not None and args.arrival_minutes_per_tick <= 0:
        raise SystemExit("--arrival-minutes-per-tick must be positive")
    if args.cell_size_uu <= 0:
        raise SystemExit("--cell-size-uu must be positive")
    if args.drone_speed_uu_per_tick <= 0:
        raise SystemExit("--drone-speed-uu-per-tick must be positive")
    if args.drone_max_view_distance_uu <= 0:
        raise SystemExit("--drone-max-view-distance-uu must be positive")
    if args.grid_stats_every_n_ticks < 0:
        raise SystemExit("--grid-stats-every-n-ticks must be >= 0")
    if args.grid_stats_readback_lag_ticks <= 0:
        raise SystemExit("--grid-stats-readback-lag-ticks must be positive")

    output_namespace = normalize_output_namespace(args.output_namespace)
    project_dir = Path(args.uproject).resolve().parent
    benchmark_root = project_dir / "Saved" / "Benchmarks"
    bench_dir = benchmark_root / output_namespace if output_namespace else benchmark_root
    cfg_dir = bench_dir / "_configs"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    if output_namespace:
        print(f"Benchmark output namespace: {output_namespace} -> {bench_dir}")

    seeds = parse_int_list(args.seeds)
    focus_points = parse_int_list(args.focus_points)
    spreads = parse_int_list(args.spread_variants)
    methods = method_ids(args.algorithms, args.regimes)

    planned: List[RunPlan] = []
    # Terrain is interleaved at the scenario-point level: all methods of a given
    # (seed, focus, spread, terrain) run consecutively, then the terrain advances.
    # Stopping early therefore leaves balanced, fully-paired coverage across
    # terrains instead of finishing one terrain before touching the next.
    flora_values = args.flora if isinstance(args.flora, list) else [args.flora]
    grid_values = args.grid if isinstance(args.grid, list) else [args.grid]
    drone_processing_values = ["sequential", "parallel_cpu"] if args.compare_drone_processing else [args.drone_processing]

    for seed in seeds:
        for focus in focus_points:
            for spread in spreads:
                for grid in grid_values:
                    for flora in flora_values:
                        for terrain in args.terrains:
                            for method_id in methods:
                                for drone_processing in drone_processing_values:
                                    if args.compare_drone_processing and drone_processing != "sequential" and method_id.startswith("oracle_"):
                                        continue
                                    cfg = make_config(args, method_id, terrain, focus, spread, seed, grid, flora, drone_processing)
                                    run_dir = cfg["benchmark_run_subdir"]
                                    if output_namespace:
                                        cfg["benchmark_run_subdir"] = f"{output_namespace}/{run_dir}"
                                    cfg_path = cfg_dir / Path(run_dir) / "config.json"
                                    cfg_path.parent.mkdir(parents=True, exist_ok=True)
                                    cfg_path.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
                                    scenario_key = cfg["generated_fire_csv_output_path"] if args.scenario_cache else cfg["run_id"]
                                    planned.append(RunPlan(
                                        index=len(planned) + 1,
                                        run_id=cfg["run_id"],
                                        run_dir=run_dir,
                                        cfg_path=cfg_path,
                                        cmd=build_command(args, cfg_path),
                                        method_id=method_id,
                                        terrain=terrain,
                                        flora=flora,
                                        focus=focus,
                                        spread=spread,
                                        seed=seed,
                                        grid=grid,
                                        drone_processing=drone_processing,
                                        scenario_key=scenario_key,
                                        expected_final_tick=args.max_ticks,
                                        requested_config=cfg,
                                        map_name=args.map,
                                    ))

    duplicate_run_ids = duplicate_values(p.run_id for p in planned)
    duplicate_run_dirs = duplicate_values(p.run_dir for p in planned)
    duplicate_cfg_paths = duplicate_values(str(p.cfg_path) for p in planned)
    if duplicate_run_ids or duplicate_run_dirs or duplicate_cfg_paths:
        print("ERROR: benchmark plan contains duplicate output identities.")
        if duplicate_run_ids:
            print("Duplicate run_id values:")
            for value in duplicate_run_ids[:20]:
                print(f"  {value}")
        if duplicate_run_dirs:
            print("Duplicate run_dir values:")
            for value in duplicate_run_dirs[:20]:
                print(f"  {value}")
        if duplicate_cfg_paths:
            print("Duplicate config paths:")
            for value in duplicate_cfg_paths[:20]:
                print(f"  {value}")
        print("Check for repeated seeds or duplicated matrix axis values.")
        return 2

    def is_complete(plan: RunPlan) -> bool:
        return read_summary_row(bench_dir, plan) is not None

    total = len(planned)
    rows_by_run_id: Dict[str, Dict[str, str]] = {}
    statuses: Dict[str, str] = {}
    for plan in planned:
        row = read_summary_row(bench_dir, plan)
        if row is not None:
            rows_by_run_id[plan.run_id] = row
            statuses[plan.run_id] = "complete"
        else:
            statuses[plan.run_id] = "pending"

    mismatched_existing = sum(
        1 for plan in planned
        if statuses.get(plan.run_id) != "complete" and summary_path(bench_dir, plan).exists()
    )
    if mismatched_existing:
        print(
            f"Ignoring {mismatched_existing} existing summary file(s) without a compatible "
            f"benchmark_request.json and final_tick={args.max_ticks}. Legacy results without "
            "execution receipts are not assumed compatible."
        )
        if not args.force and not args.plan_only and not args.dry_run:
            print("ERROR: existing results would be overwritten. Use a fresh --output-namespace "
                  "to preserve them, or --force to explicitly replace them.")
            return 2

    already = sum(1 for plan in planned if statuses.get(plan.run_id) == "complete")
    if args.force:
        print(f"{total} run(s) planned; {already} already complete, {total} to run because --force is set.")
    else:
        note = " (use --force to redo them)" if already else ""
        print(f"{total} run(s) planned; {already} already complete, {total - already} to run{note}.")

    if args.write_manifest:
        manifest_runs = [p for p in planned if args.force or statuses.get(p.run_id) != "complete"]
        grids = sorted({p.grid for p in manifest_runs})
        if len(grids) > 1:
            print(f"WARNING: manifest spans grid sizes {grids}; in-process batches must share one grid size.")
        manifest_path = Path(args.write_manifest)
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        with manifest_path.open("w", encoding="utf-8") as mf:
            for p in manifest_runs:
                mf.write(str(p.cfg_path) + "\n")
        print(f"Wrote {len(manifest_runs)} config path(s) to manifest {manifest_path}")
        return 0

    if args.plan_only:
        print_plan_summary(args, planned, statuses)
        agg_path = write_aggregate(bench_dir, planned, rows_by_run_id)
        index_path = write_run_index(bench_dir, planned, statuses)
        print(f"Aggregate: {agg_path}")
        print(f"Run index: {index_path}")
        print("Plan-only mode: no Unreal processes launched.")
        return 0

    if not args.dry_run:
        agg_path = write_aggregate(bench_dir, planned, rows_by_run_id)
        index_path = write_run_index(bench_dir, planned, statuses)
        print(f"Live aggregate: {agg_path}")
        print(f"Run index:      {index_path}")

    failed = []
    ran = 0
    skipped = 0

    if args.dry_run:
        print_plan_summary(args, planned, statuses)
        for idx, plan in enumerate(planned, 1):
            print(f"[{idx}/{total}] {plan.run_id}")
            print(f"    run_dir: {plan.run_dir}")
            print("    " + " ".join(str(part) for part in plan.cmd))
        return 0

    def finish_plan(plan: RunPlan, returncode: int, timed_out: bool) -> None:
        nonlocal ran
        # In-process batches can stop cleanly at a scenario boundary. Plans
        # after that boundary have a temporary running receipt from batch
        # preparation but no completed summary; leave them resumable.
        valid_row = read_summary_row(bench_dir, plan, require_completed_request=False)
        if args.stop_file and Path(args.stop_file).exists() and valid_row is None:
            statuses[plan.run_id] = "pending"
            request_receipt_path(bench_dir, plan).unlink(missing_ok=True)
            print(f"[{plan.index}/{total}] PENDING {plan.run_id} (batch stopped at scenario boundary)")
            return
        if timed_out:
            print(f"[{plan.index}/{total}] TIMEOUT {plan.run_id}")
            statuses[plan.run_id] = "failed"
            failed.append(plan.run_id)
            return
        if returncode != 0:
            print(f"[{plan.index}/{total}] ERROR {plan.run_id}: exit code {returncode}; "
                  "an EndPlay summary does not make a failed run complete")
            statuses[plan.run_id] = "failed"
            failed.append(plan.run_id)
            return

        row = complete_run_outputs(bench_dir, plan) if returncode == 0 else None
        if row is not None:
            rows_by_run_id[plan.run_id] = row
            statuses[plan.run_id] = "complete"
            ran += 1
        else:
            statuses[plan.run_id] = "failed"
            print(
                f"[{plan.index}/{total}] ERROR {plan.run_id}: no valid summary with "
                f"final_tick={plan.expected_final_tick} at {summary_path(bench_dir, plan)}"
            )
            failed.append(plan.run_id)

    def prepare_plan(plan: RunPlan) -> bool:
        nonlocal skipped
        if not args.force and is_complete(plan):
            skipped += 1
            statuses[plan.run_id] = "complete"
            print(f"[{plan.index}/{total}] SKIP {plan.run_id} (already complete)")
            return False
        statuses[plan.run_id] = "pending"
        prepare_run_outputs(bench_dir, plan)
        rows_by_run_id.pop(plan.run_id, None)
        return True

    try:
        if args.in_process:
            pending = []
            for plan in planned:
                if args.stop_file and Path(args.stop_file).exists():
                    print(f"Stop requested by {args.stop_file}; no in-process batch will be prepared.")
                    break
                if prepare_plan(plan):
                    pending.append(plan)
            write_aggregate(bench_dir, planned, rows_by_run_id)
            write_run_index(bench_dir, planned, statuses)
            batches = make_inprocess_batches(pending, args.batch_size)
            validation = validate_inprocess_batches(pending, batches)
            if not validation["assignment_ok"] or not validation["scenario_group_ok"] or not validation["same_grid_ok"]:
                print("ERROR: invalid in-process batch assignment; refusing to launch Unreal.")
                print(f"  batch assignment:  {'PASS' if validation['assignment_ok'] else 'FAIL'}")
                print(f"  scenario grouping: {'PASS' if validation['scenario_group_ok'] else 'FAIL'}")
                print(f"  same-grid batches: {'PASS' if validation['same_grid_ok'] else 'FAIL'}")
                if not validation["assignment_ok"]:
                    print(f"  duplicate assigned: {compact_values(validation['duplicate_assigned'][:8])}")
                    print(f"  missing assigned:   {compact_values(validation['missing'][:8])}")
                    print(f"  extra assigned:     {compact_values(validation['extra'][:8])}")
                if not validation["scenario_group_ok"]:
                    print(f"  split scenarios:    {compact_values(validation['split_scenarios'][:8])}")
                return 2
            print(f"In-process: {len(pending)} run(s) in {len(batches)} batch(es), up to {args.jobs} concurrent.")
            manifests_dir = bench_dir / "_manifests"
            manifests_dir.mkdir(parents=True, exist_ok=True)
            running_batches: Dict[int, tuple] = {}   # idx -> (process, plans, started_at)
            next_idx = 0
            while next_idx < len(batches) or running_batches:
                if args.stop_file and Path(args.stop_file).exists() and next_idx < len(batches):
                    print(f"Stop requested by {args.stop_file}; waiting for active batch(es) to finish.")
                    next_idx = len(batches)
                while len(running_batches) < args.jobs and next_idx < len(batches):
                    bplans = batches[next_idx]
                    mpath = manifests_dir / f"batch_{next_idx}.txt"
                    with mpath.open("w", encoding="utf-8") as mf:
                        for p in bplans:
                            mf.write(str(p.cfg_path) + "\n")
                    for p in bplans:
                        statuses[p.run_id] = "running"
                    print(f"[batch {next_idx + 1}/{len(batches)}] {len(bplans)} scenario(s) -> one process")
                    try:
                        proc = subprocess.Popen(build_batch_command(args, mpath))
                    except OSError as exc:
                        print(f"[batch {next_idx + 1}] ERROR: failed to launch: {exc}")
                        for p in bplans:
                            statuses[p.run_id] = "failed"
                            failed.append(p.run_id)
                        next_idx += 1
                        continue
                    running_batches[next_idx] = (proc, bplans, time.monotonic())
                    next_idx += 1
                write_run_index(bench_dir, planned, statuses)
                if not running_batches:
                    continue
                now = time.monotonic()
                done_batches = []
                for bidx, (proc, bplans, started) in list(running_batches.items()):
                    rc = proc.poll()
                    timed_out = False
                    if rc is None and now - started > args.timeout * max(1, len(bplans)):
                        proc.kill()
                        rc = proc.wait()
                        timed_out = True
                    if rc is not None:
                        running_batches.pop(bidx)
                        done_batches.append((bidx, bplans, rc, timed_out))
                if not done_batches:
                    time.sleep(0.5)
                    continue
                for bidx, bplans, rc, timed_out in done_batches:
                    for p in bplans:
                        finish_plan(p, rc, timed_out)
                    print(f"[batch {bidx + 1}/{len(batches)}] done (exit {rc}).")
                if args.fail_fast and failed and next_idx < len(batches):
                    print("Fail-fast enabled; no additional in-process batches will launch.")
                    next_idx = len(batches)
                write_aggregate(bench_dir, planned, rows_by_run_id)
                write_run_index(bench_dir, planned, statuses)
        elif args.jobs == 1:
            for plan in planned:
                if args.stop_file and Path(args.stop_file).exists():
                    print(f"Stop requested by {args.stop_file}; no new scenario will be launched.")
                    break
                if not prepare_plan(plan):
                    continue
                statuses[plan.run_id] = "running"
                write_run_index(bench_dir, planned, statuses)
                write_aggregate(bench_dir, planned, rows_by_run_id)
                print(f"[{plan.index}/{total}] RUN  {plan.run_id}")
                print(f"    -> {bench_dir / plan.run_dir}")
                try:
                    returncode = run_command(plan.cmd, args.timeout)
                    finish_plan(plan, returncode, timed_out=False)
                except subprocess.TimeoutExpired:
                    finish_plan(plan, returncode=-1, timed_out=True)
                write_aggregate(bench_dir, planned, rows_by_run_id)
                write_run_index(bench_dir, planned, statuses)
                if args.fail_fast and failed:
                    print("Fail-fast enabled; remaining scenarios stay pending for resume.")
                    break
        else:
            pending: List[RunPlan] = []
            for plan in planned:
                if args.stop_file and Path(args.stop_file).exists():
                    print(f"Stop requested by {args.stop_file}; no new scenario will be prepared.")
                    break
                if prepare_plan(plan):
                    pending.append(plan)
            write_aggregate(bench_dir, planned, rows_by_run_id)
            write_run_index(bench_dir, planned, statuses)

            running: Dict[str, ActiveRun] = {}
            active_scenarios = set()
            print(f"Parallel launch enabled: up to {args.jobs} Unreal process(es).")
            if args.record_wall_clock_timing:
                print("WARNING: --record-wall-clock-timing with --jobs > 1 contaminates mean_tick_ms.")

            while pending or running:
                if args.stop_file and Path(args.stop_file).exists() and pending:
                    print(f"Stop requested by {args.stop_file}; waiting for active scenario(s) to finish.")
                    pending.clear()
                while len(running) < args.jobs and pending:
                    pending_index = None
                    for i, candidate in enumerate(pending):
                        if candidate.scenario_key not in active_scenarios:
                            pending_index = i
                            break
                    if pending_index is None:
                        break

                    plan = pending.pop(pending_index)
                    statuses[plan.run_id] = "running"
                    print(f"[{plan.index}/{total}] RUN  {plan.run_id}")
                    print(f"    -> {bench_dir / plan.run_dir}")
                    try:
                        process = subprocess.Popen(plan.cmd)
                    except OSError as exc:
                        print(f"[{plan.index}/{total}] ERROR {plan.run_id}: failed to launch: {exc}")
                        statuses[plan.run_id] = "failed"
                        failed.append(plan.run_id)
                        continue

                    running[plan.run_id] = ActiveRun(plan=plan, process=process, started_at=time.monotonic())
                    active_scenarios.add(plan.scenario_key)

                write_run_index(bench_dir, planned, statuses)

                if not running:
                    continue

                now = time.monotonic()
                completed: List[tuple[RunPlan, int, bool]] = []
                for run_id, active in list(running.items()):
                    returncode = active.process.poll()
                    timed_out = False
                    if returncode is None and now - active.started_at > args.timeout:
                        active.process.kill()
                        returncode = active.process.wait()
                        timed_out = True
                    if returncode is not None:
                        running.pop(run_id)
                        active_scenarios.discard(active.plan.scenario_key)
                        completed.append((active.plan, returncode, timed_out))

                if not completed:
                    time.sleep(0.25)
                    continue

                for plan, returncode, timed_out in completed:
                    finish_plan(plan, returncode, timed_out)
                if args.fail_fast and failed:
                    print("Fail-fast enabled; no additional scenarios will launch.")
                    pending.clear()
                write_aggregate(bench_dir, planned, rows_by_run_id)
                write_run_index(bench_dir, planned, statuses)
    except KeyboardInterrupt:
        interrupted_processes = []
        if "running" in locals():
            interrupted_processes.extend(active.process for active in running.values())
        if "running_batches" in locals():
            interrupted_processes.extend(
                process for process, _plans, _started in running_batches.values()
            )
        stop_processes(interrupted_processes)
        print(f"\nInterrupted. This session: {ran} ran, {skipped} skipped. "
              f"Re-run the same command to resume where you left off.")
        if not args.dry_run:
            for run_id, status in list(statuses.items()):
                if status == "running":
                    plan = next((candidate for candidate in planned if candidate.run_id == run_id), None)
                    completed_row = (read_summary_row(bench_dir, plan, require_completed_request=False)
                                     if plan is not None else None)
                    if completed_row is not None:
                        complete_run_outputs(bench_dir, plan)
                        rows_by_run_id[run_id] = completed_row
                        statuses[run_id] = "complete"
                        ran += 1
                    else:
                        statuses[run_id] = "interrupted"
            agg_path = write_aggregate(bench_dir, planned, rows_by_run_id)
            index_path = write_run_index(bench_dir, planned, statuses)
            print(f"Live aggregate written: {agg_path}")
            print(f"Run index written:      {index_path}")
        return 130

    if not args.dry_run:
        agg_path = write_aggregate(bench_dir, planned, rows_by_run_id)
        index_path = write_run_index(bench_dir, planned, statuses)
        print(f"\nAggregate path: {agg_path}")
        print(f"Run index path: {index_path}")
        remaining = sum(1 for status in statuses.values() if status in {"pending", "interrupted"})
        print(f"Summary: {ran} ran this session, {skipped} already complete, "
              f"{len(failed)} failed, {remaining} pending, {total} total.")

    if failed:
        print(f"\nFailed runs: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
