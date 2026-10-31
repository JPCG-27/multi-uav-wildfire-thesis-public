#!/usr/bin/env python3
"""Prepare or explicitly execute the 90-degree FOV-scoring main-matrix ablation.

With no execution flag this prepares configs and an auditable manifest only.
Running Unreal requires both --run-full-matrix and --allow-unreal.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any


PROJECT = Path(__file__).resolve().parents[1]
ENGINE_DEFAULT = Path(r"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe")
INDEX_DEFAULT = PROJECT / "Saved" / "Benchmarks" / "run_index.csv"
ROOT_DEFAULT = PROJECT / "Saved" / "Benchmarks" / "fov_scoring_ablation_full_matrix_20261020"
FOCUSED_DEFAULT = PROJECT / "Saved" / "Benchmarks" / "fov_scoring_ablation_20261020" / "focused_validation.json"
METHODS = {"static": "mine_static", "dynamic": "mine_dynamic"}
ALLOWED_PAIR_DIFFERENCES = {
    "run_id", "benchmark_run_subdir", "drone_planner_scoring_mode",
    "generated_fire_csv_output_path", "generated_humans_csv_output_path",
    "generated_terrain_csv_output_path", "generated_fuel_csv_output_path",
}
SOURCE_FILES = [
    "Source/Thesis/Drone/Drone.cpp",
    "Source/Thesis/Drone/Drone.h",
    "Source/Thesis/Benchmark/SimRunConfig.cpp",
    "Source/Thesis/Benchmark/SimRunConfig.h",
    "Source/Thesis/World/SimulationManager.cpp",
    "Source/Thesis/World/SimulationManager.h",
    "Tools/run_benchmarks.py",
    "Tools/run_fov_scoring_full_matrix.py",
]
COMPILED_SOURCES = [p for p in SOURCE_FILES if p.startswith("Source/")]
MATRIX_SEEDS = list(range(27, 42))
MATRIX_TERRAINS = ("flat", "rolling_hills", "foothills", "mountains")
MATRIX_FOCI = (1, 2, 3)
MATRIX_SPREADS = (0, 1, 2)
MATRIX_GRIDS = (100, 200, 300)
TICK_CAP = 8000


def sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def current_build_input_files(engine: Path) -> list[Path]:
    """Enumerate code, shader, config, map, and build inputs for provenance."""
    paths: set[Path] = {PROJECT / "Thesis.uproject"}
    for root, suffixes in (
        (PROJECT / "Source", {".cpp", ".h", ".hpp", ".inl", ".cs"}),
        (PROJECT / "Shaders", {".usf", ".ush"}),
        (PROJECT / "Config", {".ini", ".xml"}),
    ):
        if root.exists():
            paths.update(p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in suffixes)
    content = PROJECT / "Content"
    if content.exists():
        # Maps contain placed actors and serialized Blueprint defaults. Hash every
        # map plus likely behavior-bearing Blueprint assets without hashing all art.
        paths.update(p for p in content.rglob("*.umap") if p.is_file())
        paths.update(p for p in content.rglob("*.uasset")
                     if p.is_file() and any(token in p.name.lower()
                                            for token in ("drone", "simmanager", "simulationmanager")))
    engine_root = engine.parents[2]
    for external in (
        engine_root / "Build" / "BatchFiles" / "Build.bat",
        engine_root / "Binaries" / "DotNET" / "UnrealBuildTool" / "UnrealBuildTool.dll",
    ):
        if external.is_file():
            paths.add(external)
    compiler = shutil.which("cl.exe")
    if compiler:
        paths.add(Path(compiler).resolve())
    return sorted(paths, key=lambda p: str(p).casefold())


def safe_token(text: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in text)


def scenario_id(row: dict[str, str]) -> str:
    return (f"{row['terrain']}_f{int(row['focus'])}_v{int(row['spread'])}_"
            f"s{int(row['seed'])}_g{int(row['grid'])}")


def pair_id(row: dict[str, str]) -> str:
    regime = "static" if row["method_id"] == "mine_static" else "dynamic"
    return f"{scenario_id(row)}_{regime}"


def load_index(path: Path) -> tuple[list[dict[str, str]], dict[str, Any]]:
    if not path.is_file():
        raise FileNotFoundError(f"Main run index missing: {path}")
    with path.open(newline="", encoding="utf-8-sig") as handle:
        all_rows = list(csv.DictReader(handle))
    if len(all_rows) != 9720:
        raise ValueError(f"Expected 9,720 authoritative main-index rows, found {len(all_rows)}")
    selected = [r for r in all_rows if r.get("method_id") in METHODS.values()]
    dims = {
        "rows": len(selected),
        "status_counts": {s: sum(r["status"] == s for r in selected)
                          for s in sorted({r["status"] for r in selected})},
        "seeds": sorted({int(r["seed"]) for r in selected}),
        "terrains": sorted({r["terrain"] for r in selected}),
        "foci": sorted({int(r["focus"]) for r in selected}),
        "spreads": sorted({int(r["spread"]) for r in selected}),
        "grids": sorted({int(r["grid"]) for r in selected}),
        "methods": sorted({r["method_id"] for r in selected}),
        "scenario_keys": len({r["scenario_key"] for r in selected}),
    }
    expected = {
        "rows": 3240,
        "seeds": MATRIX_SEEDS,
        "terrains": sorted(MATRIX_TERRAINS),
        "foci": list(MATRIX_FOCI),
        "spreads": list(MATRIX_SPREADS),
        "grids": list(MATRIX_GRIDS),
        "methods": sorted(METHODS.values()),
        "scenario_keys": 1620,
    }
    for key, value in expected.items():
        if dims[key] != value:
            raise ValueError(f"Main index dimension {key} mismatch: {dims[key]!r} != {value!r}")
    if dims["status_counts"] != {"complete": 3240}:
        raise ValueError(f"Expected all 3,240 main mine rows complete: {dims['status_counts']}")
    lookup: dict[tuple[str, str], dict[str, str]] = {}
    for row in selected:
        key = (scenario_id(row), row["method_id"])
        if key in lookup:
            raise ValueError(f"Duplicate main-index row for {key}")
        lookup[key] = row
    if len(lookup) != 3240:
        raise ValueError(f"Expected 3,240 unique scenario/regime rows, found {len(lookup)}")
    return selected, dims


def config_from_main(row: dict[str, str], condition: str, root: Path) -> tuple[dict[str, Any], Path]:
    source_config = Path(row["config_path"])
    if not source_config.is_file():
        raise FileNotFoundError(f"Main-study config missing: {source_config}")
    cfg = json.loads(source_config.read_text(encoding="utf-8"))
    expected_method = row["method_id"]
    if cfg.get("method_id") != expected_method:
        raise ValueError(f"Method mismatch in {source_config}: {cfg.get('method_id')} != {expected_method}")
    if cfg.get("max_sim_ticks") != TICK_CAP or cfg.get("drone_fov_degrees") != 90.0:
        raise ValueError(f"Unexpected main effective tick/FOV settings in {source_config}")

    sid = scenario_id(row)
    regime = "static" if expected_method == "mine_static" else "dynamic"
    pid = pair_id(row)
    stem = f"{sid}_{regime}_{condition}"
    run_id = f"fovmatrix_{stem}"
    run_subdir = Path(root.name) / "runs" / run_id
    scratch_rel = Path("Saved") / "Benchmarks" / root.name / "runtime_scratch" / run_id
    cfg.update({
        "run_id": run_id,
        "benchmark_run_subdir": run_subdir.as_posix(),
        "method_id": expected_method,
        "max_sim_ticks": TICK_CAP,
        "exit_on_max_ticks": True,
        "record_metrics": True,
        "drone_fov_degrees": 90.0,
        "drone_planner_scoring_mode": "full_fov" if condition == "full" else "target_cell",
        # Explicit cached inputs plus disabled generation make each worker
        # fail closed. The main setup still exports terrain/fuel CSVs; use
        # unique per-run scratch paths to avoid cross-process write races.
        "fire_csv_path": cfg["fire_csv_path"],
        "humans_csv_path": cfg["humans_csv_path"],
        "generate_fire_csv": False,
        "auto_generate_scenario": False,
        "reuse_generated_scenario_csvs": False,
        # These are the archived effective planner/sensor defaults used by
        # the earlier 90-degree focused validation; pin them for auditability.
        "drone_replan_cadence_ticks": 5,
        "drone_commitment_margin": 0.20,
        "sensor_fire_positive_llr": 3.0,
        "sensor_fire_negative_llr": -0.3,
        "sensor_person_positive_llr": 2.5,
        "sensor_person_negative_llr": -0.2,
        "priority_weight_staleness": 1.0,
        "priority_weight_danger": 0.35,
        "priority_weight_person": 0.30,
        "partition_recompute_every_n_ticks": 50,
        "recall_deadline_ticks": 500,
        "tracking_window_ticks": 50,
        "generated_fire_csv_output_path": (scratch_rel / "fire.csv").as_posix(),
        "generated_humans_csv_output_path": (scratch_rel / "humans.csv").as_posix(),
        "generated_terrain_csv_output_path": (scratch_rel / "terrain.csv").as_posix(),
        "generated_fuel_csv_output_path": (scratch_rel / "fuel.csv").as_posix(),
    })
    config_path = root / "configs" / f"{stem}.json"
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return cfg, config_path


def engine_command(engine: Path, uproject: Path, map_name: str,
                   cfg_path: Path, run_dir: Path) -> list[str]:
    return [str(engine), str(uproject), map_name, "-game", "-RenderOffscreen", "-unattended",
            "-nop4", "-nosplash", "-nosound", "-stdout", "-FullStdOutLogOutput",
            f"-abslog={run_dir / 'unreal.engine.log'}", f"-SimConfig={cfg_path}"]


def pair_audits(rows: list[dict[str, str]], root: Path) -> list[dict[str, Any]]:
    grouped: dict[str, dict[str, dict[str, Any]]] = {}
    for row in rows:
        condition = row["condition"]
        config = json.loads((root / row["launch_config_path"]).read_text(encoding="utf-8"))
        grouped.setdefault(row["pair_id"], {})[condition] = config
    audits = []
    for pid, pair in sorted(grouped.items()):
        if set(pair) != {"full", "target"}:
            raise ValueError(f"Incomplete scoring pair in prepared configs: {pid}")
        a, b = pair["full"], pair["target"]
        different = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
        norm = []
        for cfg in (a, b):
            cfg = dict(cfg)
            for key in ALLOWED_PAIR_DIFFERENCES:
                cfg[key] = f"<{key}>"
            norm.append(cfg)
        ok = norm[0] == norm[1] and set(different).issubset(ALLOWED_PAIR_DIFFERENCES)
        if not ok:
            raise ValueError(f"Paired config mismatch for {pid}: {different}")
        audits.append({"pair_id": pid, "different_fields": different,
                       "allowed_fields": sorted(ALLOWED_PAIR_DIFFERENCES),
                       "all_other_settings_identical": True})
    return audits


def prepare(args: argparse.Namespace) -> Path:
    root = args.output_root.resolve()
    benchmark_root = (PROJECT / "Saved" / "Benchmarks").resolve()
    try:
        root.relative_to(benchmark_root)
    except ValueError as exc:
        raise ValueError(f"Output root must be inside {benchmark_root}") from exc
    if root.exists():
        raise FileExistsError(f"Fresh output root required; refusing to reuse: {root}")

    selected, dimensions = load_index(args.main_index.resolve())
    root.mkdir(parents=True, exist_ok=False)
    (root / "configs").mkdir()
    (root / "runs").mkdir()

    by_scenario_method = {(scenario_id(r), r["method_id"]): r for r in selected}
    sorted_scenarios = sorted({scenario_id(r): r for r in selected}.items())
    cache_inputs: dict[str, dict[str, Any]] = {}
    for sid, row in sorted_scenarios:
        base = by_scenario_method[(sid, "mine_dynamic")]
        cfg = json.loads(Path(base["config_path"]).read_text(encoding="utf-8"))
        input_record: dict[str, Any] = {"scenario_id": sid, "scenario_key": base["scenario_key"]}
        for kind, key in (("fire", "fire_csv_path"), ("humans", "humans_csv_path"),
                          ("terrain", "generated_terrain_csv_output_path"),
                          ("fuel", "generated_fuel_csv_output_path")):
            path = (PROJECT / cfg[key]).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"Refusing matrix prep: cached {kind} input missing: {path}")
            input_record[kind] = {"path": str(path), "sha256": sha256(path), "bytes": path.stat().st_size,
                                  "worker_use": "read_only" if kind in ("fire", "humans") else "provenance_only_reconstructed_to_unique_scratch"}
        cache_inputs[sid] = input_record

    run_rows: list[dict[str, Any]] = []
    commands: list[list[str]] = []
    for base in sorted(selected, key=lambda r: (scenario_id(r), r["method_id"])):
        sid = scenario_id(base)
        regime = "static" if base["method_id"] == "mine_static" else "dynamic"
        pid = f"{sid}_{regime}"
        for condition in ("full", "target"):
            cfg, cfg_path = config_from_main(base, condition, root)
            run_dir_rel = Path("runs") / cfg["run_id"]
            run_dir = root / run_dir_rel
            row = {
                "condition": condition,
                "phase": "full_matrix",
                "regime": regime,
                "method_id": base["method_id"],
                "pair_id": pid,
                "scenario_id": sid,
                "terrain": base["terrain"],
                "flora": base["flora"],
                "focus": int(base["focus"]),
                "spread": int(base["spread"]),
                "seed": int(base["seed"]),
                "grid": int(base["grid"]),
                "grid_width": int(base["grid"]),
                "grid_height": int(base["grid"]),
                "tick_cap": TICK_CAP,
                "completion": "planned",
                "run_id": cfg["run_id"],
                "run_dir": run_dir_rel.as_posix(),
                "runtime_config_path": (run_dir_rel / "config.json").as_posix(),
                "launch_config_path": cfg_path.relative_to(root).as_posix(),
                "scenario_inputs": cache_inputs[sid],
                "launch_config_sha256": sha256(cfg_path),
                "summary_path": (run_dir_rel / "summary.csv").as_posix(),
                "console_log_path": (run_dir_rel / "unreal.console.log").as_posix(),
                "engine_log_path": (run_dir_rel / "unreal.engine.log").as_posix(),
                "runtime_scratch_rel": (Path("Saved") / "Benchmarks" / root.name / "runtime_scratch" / cfg["run_id"]).as_posix(),
            }
            run_rows.append(row)
            commands.append(engine_command(args.engine.resolve(), args.uproject.resolve(),
                                           args.map, cfg_path.resolve(), run_dir))

    # Fail closed on duplicates, missing scenario input hashes, bad matrix size,
    # or any setting difference besides mode and arm-specific run/output paths.
    pair_audit = pair_audits(run_rows, root)
    if len(run_rows) != 6480 or len(pair_audit) != 3240 or len(cache_inputs) != 1620:
        raise ValueError(f"Unexpected prepared matrix counts: runs={len(run_rows)}, pairs={len(pair_audit)}, caches={len(cache_inputs)}")
    index_hash = sha256(args.main_index.resolve())
    provenance_files = current_build_input_files(args.engine.resolve())
    hashes = {path.relative_to(PROJECT).as_posix() if path.is_relative_to(PROJECT) else f"external:{path}": sha256(path)
              for path in provenance_files}
    # Keep the small explicit list used by the earlier audit for compatibility.
    hashes.update({name: sha256(PROJECT / name) for name in SOURCE_FILES})
    hashes["Thesis.uproject"] = sha256(PROJECT / "Thesis.uproject")
    hashes["module_binary"] = sha256(PROJECT / "Binaries" / "Win64" / "UnrealEditor-Thesis.dll")
    hashes["engine_binary"] = sha256(args.engine.resolve())
    manifest = {
        "schema_version": 1,
        "experiment": "Full-matrix 90-degree FOV scoring ablation",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "phase": "preparation_only",
        "run_mode": "preparation_only",
        "project": str(PROJECT),
        "engine": str(args.engine.resolve()),
        "uproject": str(args.uproject.resolve()),
        "map": args.map,
        "main_run_index": str(args.main_index.resolve()),
        "main_run_index_sha256": index_hash,
        "main_index_dimensions": dimensions,
        "scope": {
            "algorithms": ["mine"],
            "regimes": ["static", "dynamic"],
            "seeds": MATRIX_SEEDS,
            "terrains": sorted(MATRIX_TERRAINS),
            "focus_points": list(MATRIX_FOCI),
            "spread_variants": list(MATRIX_SPREADS),
            "grid_widths": list(MATRIX_GRIDS),
            "scenario_count": 1620,
            "paired_comparison_count": 3240,
            "run_count": 6480,
            "runs_per_grid_regime_stratum": 1080,
            "pairs_per_grid_regime_stratum": 540,
            "tick_cap": TICK_CAP,
        },
        "scoring_modes": {"full": "full_fov", "target": "target_cell"},
        "sensor_fov_degrees": 90.0,
        "concurrency": {
            "max_unreal_processes": 2,
            "schedule": "two independent commandlet processes; each scenario arm has unique metrics/log/scratch paths",
            "shared_scenario_inputs": "read-only cached fire and human CSVs; hashes for fire/humans/terrain/fuel deduplicated per scenario_id",
            "scenario_generation_in_workers": False,
        },
        "scenario_loading_contract": {
            "fire": "explicit fire_csv_path is loaded directly; generate_fire_csv=false prevents fallback generation",
            "humans": "explicit humans_csv_path is loaded; auto_generate_scenario=false prevents fallback generation",
            "terrain_and_fuel": "source rebuilds terrain/forest from the archived profile and seed and exports movement-cost/fuel CSVs during setup; per-arm scratch output paths avoid concurrent writes to shared cache files",
            "validation_limit": "preparation-only: source contract and cached-file presence/hashes are audited, but the new fail-closed flags/scratch-path combination has not been UE-launched",
        },
        "effective_main_settings": {
            "preserved_from_main_configs": ["drone_count", "cell_size_uu", "drone_speed_uu_per_tick",
                "drone_max_view_distance_uu", "parallel_drone_processing", "partition_recompute_every_n_ticks",
                "seconds_per_tick", "terrain_profile", "flora_density", "human_count", "human_seed",
                "recall_deadline_ticks", "tracking_window_ticks", "metrics_detail_mode"],
            "verified_common": {"drone_fov_degrees": 90.0, "drone_count": 4,
                "cell_size_uu": 3600.0, "drone_speed_uu_per_tick": 3600.0,
                "drone_max_view_distance_uu": 14400.0, "partition_recompute_every_n_ticks": 50,
                "max_sim_ticks": 8000, "recall_deadline_ticks": 500,
                "tracking_window_ticks": 50, "drone_processing_mode": "parallel_cpu"},
            "planner_cadence_and_commitment": {"drone_replan_cadence_ticks": 5, "drone_commitment_margin": 0.20},
            "sensor_llr": {"fire_positive": 3.0, "fire_negative": -0.3,
                           "person_positive": 2.5, "person_negative": -0.2},
            "priority_weights": {"staleness": 1.0, "danger": 0.35, "person": 0.30},
        },
        "scenario_cache_inputs_deduplicated": cache_inputs,
        "scenario_cache_input_totals": {
            "unique_scenarios": len(cache_inputs),
            "unique_cache_files": sum(1 for _ in cache_inputs) * 4,
            "read_only_fire_human_files": sum(1 for _ in cache_inputs) * 2,
            "total_bytes": sum(x[k]["bytes"] for x in cache_inputs.values() for k in ("fire", "humans", "terrain", "fuel")),
        },
        "source_hashes_sha256_at_prepare": hashes,
        "current_build_provenance": {
            "captured_project_inputs": [p.relative_to(PROJECT).as_posix() if p.is_relative_to(PROJECT) else str(p)
                                         for p in provenance_files],
            "current_module_sha256": hashes["module_binary"],
            "current_editor_cmd_sha256": hashes["engine_binary"],
            "validation_scope": "Hash inventory records all project Source C++/headers/build rules, shaders, ini/xml configuration, maps, behavior-named Blueprint assets, project descriptor, UnrealBuildTool/Build.bat, and cl.exe when discoverable. Focused validation independently confirms only its listed gameplay sources and current module; this preparation does not compile or launch UE.",
        },
        "pair_config_audit": pair_audit,
        "runs": run_rows,
        "commands": commands,
        "random_reuse_policy": "No planner outcomes reused; only identical exogenous cached fire/human trajectories are shared read-only.",
        "historical_archive": str(PROJECT / "Saved" / "Benchmarks" / "fov_scoring_ablation_20261020"),
        "historical_archive_mutated": False,
        "launch_policy": "Preparation only. Execution requires an explicit later --run-full-matrix --allow-unreal invocation and focused validation.",
    }
    manifest_path = root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (root / "commands.txt").open("w", encoding="utf-8", newline="\n") as handle:
        for command in commands:
            handle.write(subprocess.list2cmdline(command) + "\n")
    experiment = {
        "schema_version": 1,
        "experiment": manifest["experiment"],
        "manifest": "manifest.json",
        "run_count": 6480,
        "paired_comparisons": 3240,
        "scenario_count": 1620,
        "scoring_modes": manifest["scoring_modes"],
        "regimes": ["static", "dynamic"],
        "sensor_fov_degrees": 90.0,
        "execution_started": False,
    }
    (root / "experiment.json").write_text(json.dumps(experiment, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def validate_prepared(root: Path, workers: int) -> dict[str, Any]:
    if workers != 2:
        raise ValueError("This full-matrix protocol is fixed to exactly two Unreal workers")
    path = root / "manifest.json"
    if not path.is_file():
        raise FileNotFoundError(f"Prepared manifest missing: {path}")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("scope", {}).get("run_count") != 6480 or len(manifest.get("runs", [])) != 6480:
        raise ValueError("Prepared manifest does not contain all 6,480 runs")
    if len(manifest.get("pair_config_audit", [])) != 3240 or not all(
            p.get("all_other_settings_identical") for p in manifest["pair_config_audit"]):
        raise ValueError("Prepared manifest paired config audit is incomplete or failed")
    if manifest.get("sensor_fov_degrees") != 90.0 or manifest.get("concurrency", {}).get("max_unreal_processes") != 2:
        raise ValueError("Prepared manifest FOV/concurrency contract does not match 90 degrees / two workers")
    if manifest.get("run_mode") != "preparation_only":
        raise ValueError("Prepared manifest is no longer preparation-only")
    for row in manifest["runs"]:
        run_dir = root / row["run_dir"]
        if run_dir.exists():
            raise FileExistsError(f"Prepared output path already exists; refusing reuse: {run_dir}")
        cfg_path = root / row["launch_config_path"]
        if not cfg_path.is_file() or sha256(cfg_path) != row["launch_config_sha256"]:
            raise ValueError(f"Launch config missing/changed: {cfg_path}")
    return manifest


def validate_launch(manifest: dict[str, Any], validation_path: Path,
                    engine: Path) -> dict[str, str | None]:
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    module = PROJECT / "Binaries" / "Win64" / "UnrealEditor-Thesis.dll"
    required = (validation.get("passed") is True,
                validation.get("sensor_fov_degrees") == 90.0,
                validation.get("same_arm_repeat_byte_identical") is True,
                validation.get("current_module_matches_sources") is True,
                validation.get("module_sha256", "").lower() == (sha256(module) or "").lower())
    if not all(required):
        raise RuntimeError("Focused 90-degree/same-arm/current-module validation does not match the active module")
    prepared = manifest.get("source_hashes_sha256_at_prepare", {})
    current: dict[str, str | None] = {}
    for name, expected_hash in prepared.items():
        if name == "module_binary":
            target = module
        elif name == "engine_binary":
            target = engine
        elif name == "Thesis.uproject":
            target = PROJECT / "Thesis.uproject"
        elif name.startswith("external:"):
            target = Path(name.removeprefix("external:"))
        else:
            target = PROJECT / name
        current[name] = sha256(target)
        if current[name] != expected_hash:
            raise RuntimeError(f"Prepared source/input changed before launch: {name}")
    if sha256(PROJECT / "Thesis.uproject") != manifest.get("source_hashes_sha256_at_prepare", {}).get("Thesis.uproject"):
        raise RuntimeError("Thesis.uproject changed after preparation")
    if sha256(engine) != manifest.get("source_hashes_sha256_at_prepare", {}).get("engine_binary"):
        raise RuntimeError("UE commandlet differs from preparation-time executable hash")
    stale = [p for p in COMPILED_SOURCES if (PROJECT / p).is_file()
             and (PROJECT / p).stat().st_mtime > module.stat().st_mtime]
    if stale:
        raise RuntimeError("Compiled source newer than Thesis module: " + ", ".join(stale))
    if sha256(manifest["main_run_index"]) != manifest.get("main_run_index_sha256"):
        raise RuntimeError("Authoritative main run index changed after preparation")
    for sid, inputs in manifest["scenario_cache_inputs_deduplicated"].items():
        for kind in ("fire", "humans", "terrain", "fuel"):
            record = inputs[kind]
            p = Path(record["path"])
            if sha256(p) != record["sha256"]:
                raise RuntimeError(f"Scenario input changed since preparation ({sid}/{kind}): {p}")
    return current


def finish_run(row: dict[str, Any], rc: int, root: Path) -> bool:
    run_dir = root / row["run_dir"]
    summary = run_dir / "summary.csv"
    final_tick = None
    if summary.is_file():
        with summary.open(newline="", encoding="utf-8-sig") as handle:
            values = list(csv.DictReader(handle))
        if values and values[-1].get("final_tick"):
            final_tick = int(float(values[-1]["final_tick"]))
    row["return_code"] = rc
    row["summary_final_tick"] = final_tick
    row["summary_sha256"] = sha256(summary)
    row["summary_present"] = summary.is_file()
    row["completion"] = "complete" if rc == 0 and final_tick == row["tick_cap"] else "failed"
    if row["completion"] != "complete":
        row["failure_reason"] = "UE exit code was nonzero or summary did not reach tick_cap."
        return False
    # Per-run terrain/fuel exports are transient and unique to this arm; remove
    # them only after the actual run has a valid 8,000-tick summary.
    scratch = PROJECT / row["runtime_scratch_rel"]
    for filename in ("fire.csv", "humans.csv", "terrain.csv", "fuel.csv"):
        (scratch / filename).unlink(missing_ok=True)
    try:
        scratch.rmdir()
    except OSError:
        pass
    return True


def execute(args: argparse.Namespace) -> int:
    if not args.allow_unreal:
        raise RuntimeError("Unreal requires --run-full-matrix and --allow-unreal")
    root = args.output_root.resolve()
    manifest = validate_prepared(root, args.workers)
    validation_path = args.focused_validation.resolve()
    if not validation_path.is_file():
        raise FileNotFoundError(f"Focused validation missing: {validation_path}")
    launch_hashes = validate_launch(manifest, validation_path, args.engine.resolve())
    manifest["run_mode"] = "actual_unreal_simulator"
    manifest["phase"] = "full_matrix_running"
    manifest["focused_validation"] = str(validation_path)
    manifest["focused_validation_sha256"] = sha256(validation_path)
    manifest["source_hashes_sha256_at_launch"] = launch_hashes
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    pending = iter(zip(manifest["runs"], manifest["commands"]))
    active: dict[subprocess.Popen, tuple[dict[str, Any], float]] = {}
    failed = False
    while active or not failed:
        while not failed and len(active) < args.workers:
            try:
                row, command = next(pending)
            except StopIteration:
                break
            run_dir = root / row["run_dir"]
            run_dir.mkdir(parents=True, exist_ok=False)
            scratch = PROJECT / row["runtime_scratch_rel"]
            scratch.mkdir(parents=True, exist_ok=False)
            console = run_dir / "unreal.console.log"
            handle = console.open("w", encoding="utf-8", errors="replace")
            try:
                proc = subprocess.Popen(command, cwd=PROJECT, stdout=handle, stderr=subprocess.STDOUT)
                active[proc] = (row, time.time())
                row["completion"] = "running"
                row["started_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                row["pid"] = proc.pid
                row["console_log_path"] = str(console.relative_to(root))
            except Exception:
                handle.close()
                raise
        if not active:
            break
        time.sleep(1)
        for proc, (row, started) in list(active.items()):
            rc = proc.poll()
            if rc is None and time.time() - started > args.timeout:
                proc.terminate()
                try:
                    rc = proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    rc = proc.wait()
                row["failure_reason"] = f"Timed out after {args.timeout} seconds."
                rc = 124
            if rc is not None:
                row["elapsed_seconds"] = round(time.time() - started, 3)
                row["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                if not finish_run(row, rc, root):
                    failed = True
                del active[proc]
                path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        if failed:
            for proc, (row, _) in list(active.items()):
                proc.terminate()
                row["completion"] = "interrupted_after_peer_failure"
                row["return_code"] = proc.wait(timeout=60)
                del active[proc]
            path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            return 1
    manifest["phase"] = "full_matrix_complete"
    manifest["execution_completed_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    manifest["all_runs_complete_at_tick_budget"] = all(
        r.get("completion") == "complete" and r.get("summary_final_tick") == TICK_CAP
        for r in manifest["runs"])
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0 if manifest["all_runs_complete_at_tick_budget"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=ROOT_DEFAULT)
    parser.add_argument("--main-index", type=Path, default=INDEX_DEFAULT)
    parser.add_argument("--engine", type=Path, default=ENGINE_DEFAULT)
    parser.add_argument("--uproject", type=Path, default=PROJECT / "Thesis.uproject")
    parser.add_argument("--focused-validation", type=Path, default=FOCUSED_DEFAULT)
    parser.add_argument("--map", default="NewMap")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=7200)
    parser.add_argument("--run-full-matrix", action="store_true",
                        help="Execute the already prepared 6,480-run matrix in two UE processes.")
    parser.add_argument("--allow-unreal", action="store_true",
                        help="Required additional explicit gate for launching Unreal.")
    parser.add_argument("--check", action="store_true",
                        help="Validate an existing prepared manifest without modifying it or launching Unreal.")
    args = parser.parse_args()
    args.output_root = args.output_root.resolve()
    args.engine = args.engine.resolve()
    args.uproject = args.uproject.resolve()
    args.main_index = args.main_index.resolve()
    args.focused_validation = args.focused_validation.resolve()
    if args.workers != 2:
        parser.error("--workers is fixed at 2 for the prepared full-matrix design")
    if args.check and args.run_full_matrix:
        parser.error("--check cannot be combined with --run-full-matrix")
    if args.check:
        manifest = validate_prepared(args.output_root, args.workers)
        print(f"Validated preparation-only manifest: {len(manifest['runs'])} runs, 2 workers, 90-degree FOV")
        return 0
    if args.run_full_matrix:
        return execute(args)
    if args.allow_unreal:
        parser.error("--allow-unreal alone does not launch; pair it with --run-full-matrix")
    path = prepare(args)
    print(f"Prepared 6,480 configs (3,240 pairs, 1,620 cached scenarios): {path}")
    print("No Unreal process was started. Execution requires an explicit later --run-full-matrix --allow-unreal.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
