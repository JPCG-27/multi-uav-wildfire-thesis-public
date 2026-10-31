#!/usr/bin/env python3
"""Prepare or, after a separate explicit authorization, run all current-build arms.

Default behavior prepares immutable launch configs and a manifest only. Unreal
execution requires both --run-matrix and --allow-unreal, plus a focused
validation receipt that explicitly covers the current module and all methods.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any

import run_fov_scoring_full_matrix as scoring


PROJECT = Path(__file__).resolve().parents[1]
BASE_ROOT = PROJECT / "Saved" / "Benchmarks" / "fov_scoring_ablation_current_build_20261021"
ROOT_DEFAULT = PROJECT / "Saved" / "Benchmarks" / "current_build_main_and_fov_20261021"
ENGINE_DEFAULT = scoring.ENGINE_DEFAULT
INDEX_DEFAULT = PROJECT / "Saved" / "Benchmarks" / "run_index.csv"
FOCUSED_DEFAULT = PROJECT / "Saved" / "Benchmarks" / "fov_scoring_ablation_20261020" / "focused_validation.json"
TICK_CAP = scoring.TICK_CAP
WORKERS = 3
EXPECTED_RUNS = 12960
EXPECTED_SCENARIOS = 1620
EXPECTED_SCENARIO_REGIMES = 3240
EXPECTED_FOV_PAIRS = 3240
METHODS = {
    "mine_static": {"information_source": "belief", "region_regime": "static_uniform", "planner_strategy": "GreedyFov", "partition_strategy": "Uniform"},
    "mine_dynamic": {"information_source": "belief", "region_regime": "dynamic", "planner_strategy": "GreedyFov", "partition_strategy": "ZoneRegionGrowing"},
    "random_static": {"information_source": "belief", "region_regime": "static_uniform", "planner_strategy": "Random", "partition_strategy": "Uniform"},
    "random_dynamic": {"information_source": "belief", "region_regime": "dynamic", "planner_strategy": "Random", "partition_strategy": "Random"},
    "oracle_static": {"information_source": "oracle", "region_regime": "static_uniform", "planner_strategy": "OracleTarget", "partition_strategy": "Uniform"},
    "oracle_dynamic": {"information_source": "oracle", "region_regime": "dynamic", "planner_strategy": "OracleTarget", "partition_strategy": "OracleAllocation"},
}
PATH_ONLY_DIFFERENCES = {
    "run_id", "benchmark_run_subdir", "drone_planner_scoring_mode",
    "generated_fire_csv_output_path", "generated_humans_csv_output_path",
    "generated_terrain_csv_output_path", "generated_fuel_csv_output_path",
}
METHOD_DIFFERENCES = PATH_ONLY_DIFFERENCES | {
    "method_id", "oracle_lookahead_ticks", "information_source",
    "region_regime", "planner_strategy", "partition_strategy",
}
CONTROLLED_METHOD_OVERRIDES = {
    "random_dynamic": {"partition_strategy": "ZoneRegionGrowing"},
}


def sha256(path: Path) -> str | None:
    return scoring.sha256(path)


def relpath(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def clean_method_overrides(cfg: dict[str, Any]) -> None:
    # The canonical method_id is the single source for method semantics. Avoid
    # carrying stale explicit overrides across mine/random/oracle arms.
    for key in ("information_source", "region_regime", "planner_strategy", "partition_strategy"):
        cfg.pop(key, None)


def make_arm_config(template: dict[str, Any], *, root: Path, scenario_id: str,
                    regime: str, arm: str, method_id: str) -> dict[str, Any]:
    cfg = dict(template)
    clean_method_overrides(cfg)
    run_id = f"mainfov_{arm}_{scenario_id}_{regime}"
    scratch = Path("Saved") / "Benchmarks" / root.name / "runtime_scratch" / run_id
    cfg.update({
        "run_id": run_id,
        "benchmark_run_subdir": (Path(root.name) / "runs" / run_id).as_posix(),
        "method_id": method_id,
        "max_sim_ticks": TICK_CAP,
        "exit_on_max_ticks": True,
        "record_metrics": True,
        "drone_fov_degrees": 90.0,
        "drone_planner_scoring_mode": "target_cell" if arm == "mine_target_cell" else "full_fov",
        "fire_csv_path": template["fire_csv_path"],
        "humans_csv_path": template["humans_csv_path"],
        "generate_fire_csv": False,
        "auto_generate_scenario": False,
        "reuse_generated_scenario_csvs": False,
        "generated_fire_csv_output_path": (scratch / "fire.csv").as_posix(),
        "generated_humans_csv_output_path": (scratch / "humans.csv").as_posix(),
        "generated_terrain_csv_output_path": (scratch / "terrain.csv").as_posix(),
        "generated_fuel_csv_output_path": (scratch / "fuel.csv").as_posix(),
    })
    if method_id == "random_dynamic":
        # MethodSpec's canonical random_dynamic uses a Random partition. For an
        # allocator-controlled planner baseline, override only the partition to
        # match mine_dynamic's ZoneRegionGrowing allocator; keep Random planner
        # and belief information inherited from the canonical method_id.
        cfg["partition_strategy"] = "ZoneRegionGrowing"
    if method_id.startswith("oracle_"):
        cfg["oracle_policy_version"] = 4
        cfg["oracle_lookahead_ticks"] = 500
    return cfg


def check_method_contract(cfg: dict[str, Any], method_id: str) -> None:
    if cfg.get("method_id") != method_id:
        raise ValueError(f"Method id mismatch: expected {method_id}, got {cfg.get('method_id')}")
    if cfg.get("drone_fov_degrees") != 90.0:
        raise ValueError(f"FOV must remain 90 degrees: {method_id}")
    if cfg.get("oracle_policy_version") != 4:
        raise ValueError("All current-build configs must record oracle policy v4")
    if cfg.get("generate_fire_csv") is not False or cfg.get("auto_generate_scenario") is not False:
        raise ValueError("Worker configs must fail closed instead of regenerating shared scenarios")
    actual_overrides = {key: cfg[key] for key in
                        ("information_source", "region_regime", "planner_strategy", "partition_strategy")
                        if key in cfg}
    expected_overrides = CONTROLLED_METHOD_OVERRIDES.get(method_id, {})
    if actual_overrides != expected_overrides:
        raise ValueError(f"Unexpected method overrides for {method_id}: {actual_overrides} != {expected_overrides}")
    if cfg.get("drone_processing_mode") != "parallel_cpu":
        raise ValueError("Matrix must preserve main-study parallel_cpu processing")
    if cfg.get("max_sim_ticks") != TICK_CAP:
        raise ValueError("Matrix tick cap differs from retained main study")
    if cfg.get("drone_planner_scoring_mode") not in {"full_fov", "target_cell"}:
        raise ValueError("Unknown scoring mode")
    expected_mode = "target_cell" if method_id.startswith("mine_") and cfg.get("run_id", "").startswith("mainfov_mine_target_cell_") else "full_fov"
    if cfg["drone_planner_scoring_mode"] != expected_mode:
        raise ValueError(f"Scoring mode {cfg['drone_planner_scoring_mode']} not allowed for {method_id}")
    if method_id.startswith("oracle_") and cfg.get("oracle_lookahead_ticks") != 500:
        raise ValueError("Oracle lookahead must be explicitly pinned to 500 ticks")


def normalize_config(cfg: dict[str, Any], allowed: set[str]) -> dict[str, Any]:
    normalized = dict(cfg)
    for key in allowed:
        normalized.pop(key, None)
    return normalized


def copy_source_snapshot(base_manifest: dict[str, Any], root: Path,
                         runner_path: Path) -> tuple[dict[str, Any], dict[str, str | None]]:
    base_snapshot = BASE_ROOT / "source_snapshot"
    if not base_snapshot.is_dir():
        raise FileNotFoundError(f"Current-build source snapshot missing: {base_snapshot}")
    snapshot = root / "source_snapshot"
    shutil.copytree(base_snapshot, snapshot)
    runner_dest = snapshot / "Tools" / runner_path.name
    runner_dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(runner_path, runner_dest)
    provenance_dir = root / "build_provenance"
    shutil.copytree(BASE_ROOT / "build_provenance", provenance_dir)
    base_hashes = dict(base_manifest["source_hashes_sha256_at_prepare"])
    base_hashes["Tools/run_fov_current_build_all_arms.py"] = sha256(runner_path)
    archive_records = []
    total_bytes = 0
    for path in snapshot.rglob("*"):
        if path.is_file():
            nbytes = path.stat().st_size
            total_bytes += nbytes
            archive_records.append({"path": relpath(path, root), "bytes": nbytes, "sha256": sha256(path)})
    for path in provenance_dir.rglob("*"):
        if path.is_file():
            archive_records.append({"path": relpath(path, root), "bytes": path.stat().st_size, "sha256": sha256(path)})
    provenance = {
        "inherited_from_prepared_current_fov_root": str(BASE_ROOT),
        "base_manifest_sha256": sha256(BASE_ROOT / "manifest.json"),
        "base_module_sha256": base_hashes.get("module_binary"),
        "source_hashes_sha256_at_prepare": base_hashes,
        "source_snapshot_directory": "source_snapshot",
        "build_provenance_directory": "build_provenance",
        "archived_project_input_count": sum(1 for p in snapshot.rglob("*") if p.is_file()),
        "archived_project_input_bytes": total_bytes,
        "archived_files": archive_records,
        "archive_hashes_verified": True,
        "external_build_tools": "Hashes retained from base manifest; installed engine/toolchain binaries are not copied.",
        "limitation": "The Oct-20 focused validation covers the current module and its listed gameplay files, plus same-arm FOV/random checks; no current-build Oracle launch validation exists. Full execution therefore requires a later focused receipt that explicitly covers all method IDs and the 90-degree sensor contract.",
    }
    return provenance, base_hashes


def prepare(args: argparse.Namespace) -> Path:
    root = args.output_root.resolve()
    benchmark_root = (PROJECT / "Saved" / "Benchmarks").resolve()
    try:
        root.relative_to(benchmark_root)
    except ValueError as exc:
        raise ValueError(f"Output root must be inside {benchmark_root}") from exc
    if root.exists():
        raise FileExistsError(f"Fresh output root required; refusing to reuse: {root}")
    base_manifest_path = BASE_ROOT / "manifest.json"
    base = read_json(base_manifest_path)
    scoring.validate_prepared(BASE_ROOT, 2)
    if base.get("run_mode") != "preparation_only" or len(base.get("runs", [])) != 6480:
        raise ValueError("Current-build FOV source matrix is not the expected untouched preparation")
    if base.get("sensor_fov_degrees") != 90.0:
        raise ValueError("Current-build FOV source matrix does not retain 90-degree sensing")
    if sha256(args.engine.resolve()) != base["source_hashes_sha256_at_prepare"].get("engine_binary"):
        raise ValueError("Engine binary differs from current-build FOV preparation")
    if sha256(PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll") != base["source_hashes_sha256_at_prepare"].get("module_binary"):
        raise ValueError("Current module differs from current-build FOV preparation")
    # Refuse to prepare against source/assets that have drifted since the paired
    # current-build FOV arm was generated.
    for name, expected in base["source_hashes_sha256_at_prepare"].items():
        if name == "module_binary":
            path = PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"
        elif name == "engine_binary":
            path = args.engine.resolve()
        elif name.startswith("external:"):
            path = Path(name.removeprefix("external:"))
        else:
            path = PROJECT / name
        if sha256(path) != expected:
            raise ValueError(f"Current source/build input drifted since paired FOV prep: {name}")
    if sha256(args.main_index.resolve()) != base.get("main_run_index_sha256"):
        raise ValueError("Main run index differs from current-build FOV preparation")
    for scenario_id, inputs in base["scenario_cache_inputs_deduplicated"].items():
        for kind in ("fire", "humans", "terrain", "fuel"):
            record = inputs[kind]
            if sha256(Path(record["path"])) != record["sha256"]:
                raise ValueError(f"Cached input drifted since current-build FOV prep: {scenario_id}/{kind}")

    root.mkdir(parents=True, exist_ok=False)
    (root / "configs").mkdir()
    (root / "runs").mkdir()
    base_runs = {r["pair_id"]: {} for r in base["runs"]}
    for row in base["runs"]:
        base_runs[row["pair_id"]][row["condition"]] = row
    if len(base_runs) != EXPECTED_FOV_PAIRS or any(set(v) != {"full", "target"} for v in base_runs.values()):
        raise ValueError("Current-build FOV source matrix lacks exact full/target pairs")

    index_rows: dict[tuple[str, str], dict[str, str]] = {}
    with args.main_index.resolve().open(newline="", encoding="utf-8-sig") as handle:
        main_rows = list(csv.DictReader(handle))
    for row in main_rows:
        if row["method_id"] in METHODS:
            sid = scoring.scenario_id(row)
            index_rows[(sid, row["method_id"])] = row
    if len(index_rows) != 9720:
        raise ValueError(f"Expected 9,720 retained benchmark method rows, got {len(index_rows)}")

    run_rows: list[dict[str, Any]] = []
    commands: list[list[str]] = []
    configs: dict[str, dict[str, Any]] = {}
    fov_pair_audits: list[dict[str, Any]] = []
    group_settings_audits: list[dict[str, Any]] = []
    for old_pair_id, pair in sorted(base_runs.items()):
        full_row = pair["full"]
        sid = full_row["scenario_id"]
        regime = full_row["regime"]
        scenario_regime_id = f"{sid}_{regime}"
        base_config_path = BASE_ROOT / full_row["launch_config_path"]
        template = read_json(base_config_path)
        method_ids = {
            "mine_full_fov": full_row["method_id"],
            "mine_target_cell": full_row["method_id"],
            "random": f"random_{regime}",
            "oracle": f"oracle_{regime}",
        }
        arms = ("mine_full_fov", "mine_target_cell", "random", "oracle")
        group_configs: dict[str, dict[str, Any]] = {}
        for arm in arms:
            cfg = make_arm_config(template, root=root, scenario_id=sid, regime=regime,
                                  arm=arm, method_id=method_ids[arm])
            check_method_contract(cfg, method_ids[arm])
            run_id = cfg["run_id"]
            config_path = root / "configs" / f"{run_id}.json"
            write_json(config_path, cfg)
            configs[run_id] = cfg
            run_dir_rel = Path("runs") / run_id
            condition = "full" if arm == "mine_full_fov" else "target" if arm == "mine_target_cell" else "baseline"
            row_pair_id = f"{scenario_regime_id}_fov" if arm.startswith("mine_") else f"{scenario_regime_id}_{arm}"
            main_ref = index_rows.get((sid, method_ids[arm]))
            if main_ref is None:
                raise ValueError(f"Missing historical method reference: {sid}/{method_ids[arm]}")
            source_template = {
                "path": str(base_config_path),
                "sha256": sha256(base_config_path),
                "source_matrix_manifest_sha256": sha256(base_manifest_path),
            }
            row = {
                "arm": arm,
                "condition": condition,
                "method_id": method_ids[arm],
                "method_contract": METHODS[method_ids[arm]],
                "controlled_method_overrides": CONTROLLED_METHOD_OVERRIDES.get(method_ids[arm], {}),
                "pair_id": row_pair_id,
                "scenario_regime_id": scenario_regime_id,
                "scenario_id": sid,
                "regime": regime,
                "terrain": full_row["terrain"],
                "flora": full_row["flora"],
                "focus": full_row["focus"],
                "spread": full_row["spread"],
                "seed": full_row["seed"],
                "grid": full_row["grid"],
                "grid_width": full_row["grid_width"],
                "grid_height": full_row["grid_height"],
                "tick_cap": TICK_CAP,
                "completion": "planned",
                "run_id": run_id,
                "run_dir": run_dir_rel.as_posix(),
                "runtime_config_path": (run_dir_rel / "config.json").as_posix(),
                "launch_config_path": relpath(config_path, root),
                "launch_config_sha256": sha256(config_path),
                "summary_path": (run_dir_rel / "summary.csv").as_posix(),
                "engine_log_path": (run_dir_rel / "unreal.engine.log").as_posix(),
                "console_log_path": (run_dir_rel / "unreal.console.log").as_posix(),
                "runtime_scratch_rel": (Path("Saved") / "Benchmarks" / root.name / "runtime_scratch" / run_id).as_posix(),
                "scenario_inputs": full_row["scenario_inputs"],
                "historical_main_reference": {
                    "run_id": main_ref["run_id"],
                    "method_id": main_ref["method_id"],
                    "status": main_ref["status"],
                    "summary_path": main_ref["summary_path"],
                    "config_path": main_ref["config_path"],
                    "role": "historical descriptive reference only; producer provenance not established",
                },
                "template_config": source_template,
            }
            command = scoring.engine_command(args.engine.resolve(), args.uproject.resolve(), args.map,
                                             config_path.resolve(), root / run_dir_rel)
            row["command"] = command
            run_rows.append(row)
            commands.append(command)
            group_configs[arm] = cfg

        # Full and target are a true same-current-build pair; random and oracle
        # are separate non-scorer baselines, not assumed equivalent to each other.
        full_cfg, target_cfg = group_configs["mine_full_fov"], group_configs["mine_target_cell"]
        differences = sorted(k for k in set(full_cfg) | set(target_cfg) if full_cfg.get(k) != target_cfg.get(k))
        if not set(differences).issubset(PATH_ONLY_DIFFERENCES):
            raise ValueError(f"FOV paired config differs beyond scoring/output fields for {scenario_regime_id}: {differences}")
        if normalize_config(full_cfg, PATH_ONLY_DIFFERENCES) != normalize_config(target_cfg, PATH_ONLY_DIFFERENCES):
            raise ValueError(f"FOV pair config mismatch for {scenario_regime_id}")
        fov_pair_audits.append({"pair_id": f"{scenario_regime_id}_fov", "scenario_regime_id": scenario_regime_id,
                                "different_fields": differences, "all_non_mode_settings_identical": True})
        # All four arms share the same scene, timing, perfect sensor, movement,
        # and metrics settings. Only method identity, scorer mode, output paths,
        # and the Oracle lookahead knob may differ by design.
        normalized = [normalize_config(group_configs[a], METHOD_DIFFERENCES) for a in arms]
        if any(x != normalized[0] for x in normalized[1:]):
            raise ValueError(f"Exogenous settings differ across arms for {scenario_regime_id}")
        group_settings_audits.append({"scenario_regime_id": scenario_regime_id,
                                      "arms": list(arms), "shared_exogenous_config": True})

    arm_counts = {arm: sum(r["arm"] == arm for r in run_rows)
                  for arm in ("mine_full_fov", "mine_target_cell", "random", "oracle")}
    if len(run_rows) != EXPECTED_RUNS or len(fov_pair_audits) != EXPECTED_FOV_PAIRS or len(group_settings_audits) != EXPECTED_SCENARIO_REGIMES:
        raise ValueError(f"Unexpected matrix sizes: rows={len(run_rows)}, FOV pairs={len(fov_pair_audits)}, groups={len(group_settings_audits)}")
    if arm_counts != {k: 3240 for k in ("mine_full_fov", "mine_target_cell", "random", "oracle")}:
        raise ValueError(f"Arm counts do not match design: {arm_counts}")

    # Preserve all hashed project inputs and the exact module/build receipt used
    # by the existing current-build FOV matrix, plus this combined runner.
    base_hashes = base["source_hashes_sha256_at_prepare"]
    provenance, source_hashes = copy_source_snapshot(base, root, Path(__file__).resolve())
    main_index_sha = sha256(args.main_index.resolve())
    manifest = {
        "schema_version": 1,
        "experiment": "Current-build main algorithms plus FOV-scoring ablation",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "phase": "preparation_only",
        "run_mode": "preparation_only",
        "project": str(PROJECT),
        "engine": str(args.engine.resolve()),
        "uproject": str(args.uproject.resolve()),
        "map": args.map,
        "main_run_index": str(args.main_index.resolve()),
        "main_run_index_sha256": main_index_sha,
        "main_index_integrity_check": base.get("main_index_integrity_check"),
        "scope": {
            "seeds": list(range(27, 42)),
            "terrains": ["flat", "rolling_hills", "foothills", "mountains"],
            "focus_points": [1, 2, 3], "spread_variants": [0, 1, 2],
            "grid_widths": [100, 200, 300],
            "unique_scenarios": EXPECTED_SCENARIOS,
            "scenario_regime_groups": EXPECTED_SCENARIO_REGIMES,
            "FOV_scoring_pairs_full_vs_target": EXPECTED_FOV_PAIRS,
            "configured_runs": EXPECTED_RUNS,
            "configured_runs_per_arm": arm_counts,
            "tick_cap": TICK_CAP,
        },
        "arms": {
            "mine_full_fov": "mine planner, full FOV-path scorer",
            "mine_target_cell": "mine planner, target-cell-only scorer",
            "random": "Random planner, canonical random static/dynamic method IDs",
            "oracle": "OracleTarget planner, policy v4, canonical oracle static/dynamic method IDs",
        },
        "method_contracts_from_current_source": METHODS,
        "controlled_method_overrides": {
            "random_dynamic": {
                "partition_strategy": "ZoneRegionGrowing",
                "canonical_method_partition": "Random",
                "purpose": "Match mine_dynamic's allocator while keeping the Random planner, belief information, and dynamic regime. This is a controlled planner baseline and intentionally differs from the canonical random_dynamic combination retained in the main matrix.",
            }
        },
        "oracle_processing_note": "Configs retain the main-study parallel_cpu request, but current SimulationManager::CanUseParallelDroneProcessing explicitly forces OracleTarget to game-thread execution with oracle_planner fallback. The three-worker design means three separate Unreal processes; it does not mean Oracle's per-drone logic runs in parallel. Future runtime analysis must verify the effective fallback in each Oracle run snapshot/log.",
        "sensor": {"type": "perfect", "drone_fov_degrees": 90.0,
                   "drone_max_view_distance_uu": 14400.0,
                   "enable_vision_bridge": False, "enable_drone_captures": False},
        "concurrency": {
            "max_unreal_processes": WORKERS,
            "schedule": "bounded process pool; no more than three Unreal processes active at once",
            "runtime_validation": "not validated; no UE launch permitted in this preparation task",
            "scenario_generation_in_workers": False,
            "shared_exogenous_inputs": "cached fire/human trajectories shared read-only; each arm has unique output and scratch paths",
        },
        "oracle_policy": {
            "version": 4, "lookahead_ticks": 500,
            "deduplicate_static_dynamic": False,
            "reason": "No current-build static/dynamic Oracle parity result exists; both regimes remain separate configured arms.",
        },
        "random_policy": {
            "reuse_historical_random_outcomes": False,
            "reason": "Historical producer provenance is unknown; current-build Random is configured as a new arm.",
        },
        "fov_scoring_policy": {
            "repeat_fov_counting": "unchanged current-source behavior in full_fov and target_cell arms; current Drone.cpp source hash is captured",
            "target_cell_is_scoring_only": True,
        },
        "scenario_loading_contract": {
            "fire_and_humans": "explicit cached paths; generation disabled; workers fail closed if unavailable",
            "terrain_and_fuel": "unique per-arm generated output paths; no shared writes",
            "validation_limit": "source/config preparation check only; no UE launch validated the all-arm input flags or three-worker behavior",
        },
        "current_build_provenance": provenance,
        "source_hashes_sha256_at_prepare": source_hashes,
        "base_current_fov_manifest": str(BASE_ROOT / "manifest.json"),
        "base_current_fov_manifest_sha256": sha256(BASE_ROOT / "manifest.json"),
        "scenario_cache_inputs_deduplicated": base["scenario_cache_inputs_deduplicated"],
        "scenario_cache_input_totals": base["scenario_cache_input_totals"],
        "fov_pair_config_audit": fov_pair_audits,
        "all_arm_shared_config_audit": group_settings_audits,
        "runs": run_rows,
        "commands": commands,
        "historical_archive_mutated": False,
        "launch_policy": "Preparation only by default. Future execution requires --run-matrix and --allow-unreal plus a fresh focused validation receipt covering all method IDs, current module, 90-degree sensor, and same-build parity needed by this protocol.",
    }
    write_json(root / "manifest.json", manifest)
    with (root / "commands.txt").open("w", encoding="utf-8", newline="\n") as handle:
        for command in commands:
            handle.write(subprocess.list2cmdline(command) + "\n")
    write_json(root / "experiment.json", {
        "schema_version": 1, "experiment": manifest["experiment"], "manifest": "manifest.json",
        "configured_runs": EXPECTED_RUNS, "unique_scenarios": EXPECTED_SCENARIOS,
        "scenario_regime_groups": EXPECTED_SCENARIO_REGIMES,
        "FOV_scoring_pairs": EXPECTED_FOV_PAIRS, "arms": arm_counts,
        "oracle_static_dynamic_deduplication": False, "workers": WORKERS,
        "sensor_fov_degrees": 90.0, "execution_started": False,
    })
    write_json(root / "readiness_validation.json", {
        "preparation_valid": True, "execution_validated": False,
        "reason_execution_validated": "No UE launch was allowed; Oracle current-build behavior and three-process scheduling are not runtime-validated.",
        "run_count": EXPECTED_RUNS, "fov_pairs": EXPECTED_FOV_PAIRS,
        "arm_counts": arm_counts, "workers": WORKERS,
    })
    return root / "manifest.json"


def validate_prepared(root: Path) -> dict[str, Any]:
    manifest = read_json(root / "manifest.json")
    if manifest.get("run_mode") != "preparation_only" or manifest.get("phase") != "preparation_only":
        raise ValueError("Manifest is not a preparation-only plan")
    rows = manifest.get("runs", [])
    if len(rows) != EXPECTED_RUNS:
        raise ValueError(f"Expected {EXPECTED_RUNS} runs, found {len(rows)}")
    counts = {arm: sum(r.get("arm") == arm for r in rows) for arm in
              ("mine_full_fov", "mine_target_cell", "random", "oracle")}
    if counts != {arm: 3240 for arm in counts}:
        raise ValueError(f"Arm counts mismatch: {counts}")
    if manifest["concurrency"]["max_unreal_processes"] != WORKERS:
        raise ValueError("Worker count mismatch")
    if manifest["sensor"]["drone_fov_degrees"] != 90.0:
        raise ValueError("Sensor FOV changed")
    if len(manifest["fov_pair_config_audit"]) != EXPECTED_FOV_PAIRS:
        raise ValueError("FOV pair audit count mismatch")
    for pair in manifest["fov_pair_config_audit"]:
        if not pair["all_non_mode_settings_identical"]:
            raise ValueError(f"FOV config pair failed: {pair['pair_id']}")
    outputs = set()
    for row in rows:
        out = root / row["run_dir"]
        if out.exists():
            raise FileExistsError(f"Output path exists; refusing overwrite: {out}")
        if row["run_dir"] in outputs:
            raise ValueError(f"Duplicate output path {row['run_dir']}")
        outputs.add(row["run_dir"])
        scratch = PROJECT / row["runtime_scratch_rel"]
        if scratch.exists():
            raise FileExistsError(f"Runtime scratch path exists; refusing reuse: {scratch}")
        cfg_path = root / row["launch_config_path"]
        if not cfg_path.is_file() or sha256(cfg_path) != row["launch_config_sha256"]:
            raise ValueError(f"Missing/changed launch config: {cfg_path}")
        cfg = read_json(cfg_path)
        check_method_contract(cfg, row["method_id"])
        if cfg.get("drone_fov_degrees") != 90.0:
            raise ValueError(f"FOV mismatch {cfg_path}")
    for name, expected in manifest["source_hashes_sha256_at_prepare"].items():
        if name == "module_binary": path = PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"
        elif name == "engine_binary": path = Path(manifest["engine"])
        elif name.startswith("external:"): path = Path(name.removeprefix("external:"))
        else: path = PROJECT / name
        if sha256(path) != expected:
            raise ValueError(f"Source/build input differs from preparation: {name}")
    for archived in manifest["current_build_provenance"]["archived_files"]:
        path = root / archived["path"]
        if not path.is_file() or path.stat().st_size != archived["bytes"] or sha256(path) != archived["sha256"]:
            raise ValueError(f"Archived provenance file missing/changed: {archived['path']}")
    return manifest


def validate_launch(manifest: dict[str, Any], validation_path: Path, engine: Path) -> None:
    validation = read_json(validation_path)
    required_methods = set(METHODS)
    required = (
        validation.get("passed") is True,
        validation.get("sensor_fov_degrees") == 90.0,
        validation.get("current_module_matches_sources") is True,
        validation.get("module_sha256", "").lower() == (sha256(PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll") or "").lower(),
        validation.get("same_arm_repeat_byte_identical") is True,
        required_methods.issubset(set(validation.get("covered_method_ids", []))),
        validation.get("three_worker_runtime_validated") is True,
    )
    if not all(required):
        raise RuntimeError("Execution blocked: focused receipt must pass current-module, 90-degree, all-method, repeatability, and three-worker validation gates")
    if sha256(engine) != manifest["current_build_provenance"]["source_hashes_sha256_at_prepare"].get("engine_binary"):
        raise RuntimeError("Engine binary changed since preparation")
    for name, expected in manifest["source_hashes_sha256_at_prepare"].items():
        if name == "module_binary": path = PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"
        elif name == "engine_binary": path = engine
        elif name.startswith("external:"): path = Path(name.removeprefix("external:"))
        else: path = PROJECT / name
        if sha256(path) != expected:
            raise RuntimeError(f"Source/build input changed since preparation: {name}")
    if sha256(Path(manifest["main_run_index"])) != manifest["main_run_index_sha256"]:
        raise RuntimeError("Main run index changed since preparation")
    for scenario, inputs in manifest["scenario_cache_inputs_deduplicated"].items():
        for kind in ("fire", "humans", "terrain", "fuel"):
            record = inputs[kind]
            if sha256(Path(record["path"])) != record["sha256"]:
                raise RuntimeError(f"Cached scenario input changed: {scenario}/{kind}")


def finish_run(row: dict[str, Any], rc: int, root: Path) -> bool:
    summary = root / row["summary_path"]
    tick = None
    if summary.is_file():
        with summary.open(newline="", encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
        if rows and rows[-1].get("final_tick"):
            tick = int(float(rows[-1]["final_tick"]))
    row["return_code"] = rc
    row["summary_final_tick"] = tick
    row["summary_sha256"] = sha256(summary)
    row["completion"] = "complete" if rc == 0 and tick == TICK_CAP else "failed"
    if row["completion"] != "complete":
        row["failure_reason"] = "Nonzero UE exit or summary failed to reach 8,000 ticks"
        return False
    scratch = PROJECT / row["runtime_scratch_rel"]
    for name in ("fire.csv", "humans.csv", "terrain.csv", "fuel.csv"):
        (scratch / name).unlink(missing_ok=True)
    try: scratch.rmdir()
    except OSError: pass
    return True


def execute(args: argparse.Namespace) -> int:
    if not args.allow_unreal:
        raise RuntimeError("Execution requires both --run-matrix and --allow-unreal")
    root = args.output_root.resolve()
    manifest = validate_prepared(root)
    validate_launch(manifest, args.focused_validation.resolve(), args.engine.resolve())
    manifest["run_mode"] = "actual_unreal_simulator"
    manifest["phase"] = "running"
    manifest["focused_validation_path"] = str(args.focused_validation.resolve())
    manifest["focused_validation_sha256"] = sha256(args.focused_validation.resolve())
    write_json(root / "manifest.json", manifest)
    pending = iter(manifest["runs"])
    active: dict[subprocess.Popen, tuple[dict[str, Any], float, Any]] = {}
    completed = 0
    failed = False
    state_path = root / "execution_events.jsonl"
    while active or not failed:
        while not failed and len(active) < WORKERS:
            try: row = next(pending)
            except StopIteration: break
            run_dir = root / row["run_dir"]
            run_dir.mkdir(parents=True, exist_ok=False)
            scratch = PROJECT / row["runtime_scratch_rel"]
            scratch.mkdir(parents=True, exist_ok=False)
            log = (run_dir / "unreal.console.log").open("w", encoding="utf-8", errors="replace")
            proc = subprocess.Popen(row["command"], cwd=PROJECT, stdout=log, stderr=subprocess.STDOUT)
            row["completion"] = "running"
            row["pid"] = proc.pid
            row["started_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            active[proc] = (row, time.time(), log)
        if not active: break
        time.sleep(1)
        for proc, (row, started, log) in list(active.items()):
            rc = proc.poll()
            if rc is None and time.time() - started > args.timeout:
                proc.terminate()
                try: rc = proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    proc.kill(); rc = proc.wait()
                rc = 124
                row["failure_reason"] = f"Timed out after {args.timeout}s"
            if rc is not None:
                log.close()
                row["elapsed_seconds"] = round(time.time() - started, 3)
                row["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                if not finish_run(row, rc, root): failed = True
                completed += 1
                with state_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps({"run_id": row["run_id"], "completion": row["completion"], "return_code": rc}) + "\n")
                del active[proc]
        if completed % 50 == 0 and completed:
            write_json(root / "manifest.json", manifest)
        if failed:
            for proc, (row, _, log) in list(active.items()):
                proc.terminate(); row["completion"] = "interrupted_after_peer_failure"
                row["return_code"] = proc.wait(timeout=60); log.close(); del active[proc]
            write_json(root / "manifest.json", manifest)
            return 1
    manifest["phase"] = "complete"
    manifest["all_runs_complete_at_tick_budget"] = all(
        r.get("completion") == "complete" and r.get("summary_final_tick") == TICK_CAP
        for r in manifest["runs"])
    write_json(root / "manifest.json", manifest)
    return 0 if manifest["all_runs_complete_at_tick_budget"] else 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-root", type=Path, default=ROOT_DEFAULT)
    p.add_argument("--engine", type=Path, default=ENGINE_DEFAULT)
    p.add_argument("--uproject", type=Path, default=PROJECT / "Thesis.uproject")
    p.add_argument("--main-index", type=Path, default=INDEX_DEFAULT)
    p.add_argument("--map", default="NewMap")
    p.add_argument("--workers", type=int, default=WORKERS)
    p.add_argument("--timeout", type=int, default=7200)
    p.add_argument("--focused-validation", type=Path, default=FOCUSED_DEFAULT)
    p.add_argument("--check", action="store_true", help="Validate prepared configs without launching UE")
    p.add_argument("--run-matrix", action="store_true", help="Run the prepared 12,960-job matrix")
    p.add_argument("--allow-unreal", action="store_true", help="Second explicit gate required for any UE launch")
    args = p.parse_args()
    if args.workers != WORKERS: p.error("Worker count is fixed at three for this design")
    if args.check and args.run_matrix: p.error("--check cannot be combined with --run-matrix")
    if args.check:
        m = validate_prepared(args.output_root.resolve())
        print(f"Validated preparation only: {len(m['runs'])} configs, three-worker schedule, 90-degree sensor")
        return 0
    if args.run_matrix: return execute(args)
    if args.allow_unreal: p.error("--allow-unreal alone cannot launch; --run-matrix is also required")
    path = prepare(args)
    print(f"Prepared {EXPECTED_RUNS} configs across four arms; no Unreal process was started: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
