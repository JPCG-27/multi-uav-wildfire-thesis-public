#!/usr/bin/env python3
"""Prepare, parity-validate, and execute the sequential 3,240 + 3,240 follow-up.

Preparation and parity validation do not touch the simulator unless ``--validate``
is explicitly supplied. Production is gated by the independent validation receipt.
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
import traceback
from typing import Any

PROJECT = Path(__file__).resolve().parents[1]
BENCH = PROJECT / "Saved/Benchmarks"
CONTROLLER = BENCH / "lawnmower_followup_20261027"
V1_ROOT = BENCH / "lawnmower_pinned_v2"
V1_MANIFEST = BENCH / "lawnmower_v1/manifest_preexecution.json"
V1_PRODUCTION = V1_ROOT / "production_execution/full_3240_20261027/manifest_runtime.json"
PILOT_ROOT = BENCH / "lawnmower_v2_pilot_run_20261027_r3"
ARM_SPECS = {
    "v2": {"root": BENCH / "lawnmower_v2_full_20261027", "planner": "LawnmowerV2",
           "prefix": "lawnmower_v2", "version": 2},
    "aligned": {"root": BENCH / "lawnmower_v1_aligned_full_20261027", "planner": "LawnmowerV1Aligned",
                "prefix": "lawnmower_v1_aligned", "version": 1},
}
ENGINE = Path(r"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe")
MODULE = PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"
MAX_PROCESSES = 3
TICKS = 8000
POLL_SECONDS = 2
NO_PROGRESS_SECONDS = 3600
sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_lawnmower_pinned_validation as validation  # noqa: E402


def sha(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read(p: Path) -> Any:
    return json.loads(p.read_text(encoding="utf-8-sig"))


def atomic(p: Path, value: Any) -> None:
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(tmp, p)


def utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def close_enough(a: Any, b: Any) -> bool:
    return validation.same_value(a, b)


def source_rows() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    pinned = read(V1_ROOT / "manifest_preexecution.json")
    aligned = read(V1_MANIFEST)
    prod = read(V1_PRODUCTION)
    if len(pinned.get("runs", [])) != 3240 or len(aligned.get("runs", [])) != 3240:
        raise ValueError("Expected complete 3,240-row historical pinned and V1 manifests")
    if prod.get("overall_status") != "passed" or prod.get("expected_runs") != 3240:
        raise ValueError("Historical pinned V1 production is not complete")
    return pinned, aligned, prod


def historical_output_audit(prod: dict[str, Any]) -> dict[str, Any]:
    runs = [j for b in prod["batches"] for j in b["runs"]]
    if len(runs) != 3240:
        raise ValueError("Historical V1 runtime does not enumerate all 3,240 runs")
    files = 0
    for job in runs:
        if job.get("status") != "runtime_evidence_passed" or not job.get("artifact_hashes"):
            raise ValueError(f"Historical V1 output evidence missing: {job.get('run_id')}")
        for name, digest in job["artifact_hashes"].items():
            p = Path(job["run_dir"]) / name
            if not p.is_file() or sha(p) != digest:
                raise ValueError(f"Historical V1 output changed: {p}")
            files += 1
    return {"status": "passed", "runs_checked": len(runs), "files_checked": files,
            "production_manifest_sha256": sha(V1_PRODUCTION)}


def verify_source(pinned: dict[str, Any], v1: dict[str, Any], prod: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    pins = pinned.get("inherited_parameter_pins", {})
    if len(pins) != 49 or pins != read(V1_ROOT / "provenance_pins.json")["field_to_value_mapping"]:
        raise ValueError("Pinned V2 source must supply the matching 49-field runtime map")
    v1_by_source = {r["source_run_id"]: r for r in v1["runs"]}
    completed = {j["run_id"]: j for b in prod["batches"] for j in b["runs"]}
    if len(v1_by_source) != 3240:
        raise ValueError("V1 source manifest run IDs are not unique")
    pinned_by_source = {r["source_run_id"]: r for r in pinned["runs"]}
    if len(pinned_by_source) != 3240 or set(pinned_by_source) != set(v1_by_source):
        raise ValueError("Pinned queue and V1 source cohort do not match")
    return pins, {"v1": v1_by_source, "pinned": pinned_by_source, "completed": completed}


def copied_snapshot(source: Path, dest: Path, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    copied = []
    for src in sorted(set(source.resolve() for source in [*rows])):
        if not src.is_file():
            raise FileNotFoundError(src)
        relative = src.relative_to(PROJECT) if src.is_relative_to(PROJECT) else Path("external_runtime") / src.name
        out = dest / relative
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, out)
        digest = sha(src)
        if sha(out) != digest:
            raise IOError(f"Provenance snapshot copy hash mismatch: {src}")
        copied.append({"path": str(src), "snapshot_path": str(out), "bytes": src.stat().st_size, "sha256": digest})
    return copied


def build_arm(arm: str, pins: dict[str, Any], maps: dict[str, Any], controller: Path) -> dict[str, Any]:
    spec = ARM_SPECS[arm]
    root = spec["root"]
    if root.exists():
        raise FileExistsError(f"Refusing preexisting follow-up arm namespace: {root}")
    root.mkdir(parents=True)
    (root / "configs").mkdir()
    (root / "batch_manifests").mkdir()
    jobs: list[dict[str, Any]] = []
    jobs_by_key: dict[tuple[str, str, int], dict[str, Any]] = {}
    # Use V1 manifest order and its nine historical queue batches verbatim.
    for row in maps["v1"].values():
        pinned_row = maps["pinned"][row["source_run_id"]]
        completed = maps["completed"].get(pinned_row["run_id"])
        if not completed or completed.get("status") != "runtime_evidence_passed":
            raise ValueError(f"Frozen source row is not complete: {pinned_row['run_id']}")
        cfg_path = Path(pinned_row["config_path"])
        if not cfg_path.is_file() or sha(cfg_path) != pinned_row["config_sha256"]:
            raise ValueError(f"Frozen pinned launch config changed: {cfg_path}")
        cfg = read(cfg_path)
        if len(pinned_row.get("expected_pinned_parameters", {})) != 49:
            raise ValueError(f"Missing 49 inherited pins: {row['source_run_id']}")
        for key, value in pins.items():
            if key not in {"environment_presentation_version", "vision_sensor_contract_version", "yolo_online_metric_version"}:
                if key not in cfg or not close_enough(cfg[key], value):
                    raise ValueError(f"Source config does not match pin {key}: {row['source_run_id']}")
        explicit_pin_count = sum(key in cfg for key in pins)
        if explicit_pin_count != 46:
            raise ValueError(f"Expected 46 inherited explicit pins plus three runtime-only assertions for {row['source_run_id']}")
        if row["scenario_id"] != pinned_row["scenario_id"] or row["seed"] != pinned_row["seed"]:
            raise ValueError(f"Historical source key mismatch: {row['source_run_id']}")
        input_hashes = pinned_row.get("input_hashes", {})
        for label, record in input_hashes.items():
            path = Path(record["path"])
            if not path.is_file() or sha(path) != record["sha256"]:
                raise ValueError(f"Frozen scenario input changed: {path}")
        old = dict(cfg)
        rid = row["run_id"].replace("lawnmower_v1_", f"{spec['prefix']}_", 1)
        cfg.update(run_id=rid, benchmark_run_subdir=f"{root.name}/runs/{rid}",
                   planner_strategy=spec["planner"],
                   partition_strategy="SpawnAwareUniform" if row["regime"] == "static" else "ZoneRegionGrowing",
                   method_id=f"{spec['prefix']}_{row['regime']}", max_sim_ticks=TICKS, exit_on_max_ticks=True)
        # Full parity traces; production retains the frozen source's summary mode.
        cfg["metrics_detail_mode"] = "summary"
        cfg["generate_fire_csv"] = False
        cfg["auto_generate_scenario"] = False
        cfg["reuse_generated_scenario_csvs"] = False
        for kind in ("fire", "humans", "terrain", "fuel"):
            cfg[f"generated_{kind}_csv_output_path"] = f"Saved/Benchmarks/{root.name}/runtime_scratch/{rid}/{kind}.csv"
        cp = root / "configs" / f"{rid}.json"
        cp.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        run_dir = root / "runs" / rid
        if run_dir.exists():
            raise FileExistsError(run_dir)
        diffs = {k: {"pinned_v2": old.get(k), "followup": cfg.get(k)}
                 for k in sorted(set(old) | set(cfg)) if old.get(k) != cfg.get(k)}
        allowed = {"run_id", "benchmark_run_subdir", "planner_strategy", "partition_strategy", "method_id",
                   "generated_fire_csv_output_path", "generated_humans_csv_output_path",
                   "generated_terrain_csv_output_path", "generated_fuel_csv_output_path"}
        if set(diffs) - allowed:
            raise ValueError(f"Unexpected config differences for {rid}: {sorted(set(diffs)-allowed)}")
        job = {"run_id": rid, "source_run_id": pinned_row["run_id"], "v1_source_run_id": row["run_id"],
               "scenario_id": row["scenario_id"], "seed": row["seed"], "grid": int(row["grid"]),
               "regime": row["regime"], "expected_pinned_parameters": pins, "input_hashes": input_hashes,
               "source_config_path": str(cfg_path.resolve()), "source_config_sha256": pinned_row["config_sha256"],
               "config_path": str(cp.resolve()), "config_sha256": sha(cp), "config_differences": diffs,
               "run_dir": str(run_dir.resolve()), "runtime_config_path": str((run_dir / "config.json").resolve()),
               "phase": "production", "status": "pending"}
        jobs.append(job)
        jobs_by_key[(row["scenario_id"], row["regime"], int(row["grid"]))] = job

    source_batches = maps["v1"].values()
    del source_batches
    # Preserve queue order from the pinned manifest, rebasing each entry to our configs.
    pinned_manifest = read(V1_ROOT / "manifest_preexecution.json")
    pinned_by_source = maps["pinned"]
    job_by_source = {j["source_run_id"]: j for j in jobs}
    batches = []
    for ix, batch in enumerate(pinned_manifest["batches"], 1):
        # Historical batch run IDs identify the pinned V2 records; batch order and boundaries stay intact.
        batch_jobs = [job_by_source[rid] for rid in batch["run_ids"]]
        entries = [j["config_path"] for j in batch_jobs]
        manifest_path = root / "batch_manifests" / Path(batch["manifest_path"]).name
        manifest_path.write_text("\n".join(entries) + "\n", encoding="utf-8")
        cmd = [x for x in batch["command"] if not x.startswith("-abslog=") and not x.startswith("-SimConfigManifest=")]
        cmd.extend([f"-abslog={(root / 'logs' / f'production_b{ix:02d}.engine.log').resolve()}",
                    f"-SimConfigManifest={manifest_path.resolve()}"])
        batches.append({"batch_id": f"{arm}_b{ix:02d}", "size": len(batch_jobs),
                        "run_ids": [j["run_id"] for j in batch_jobs], "jobs": batch_jobs, "runs": batch_jobs,
                        "manifest_path": str(manifest_path.resolve()), "manifest_sha256": sha(manifest_path),
                        "manifest_entries": entries, "command": cmd, "phase": "production",
                        "console_log": str((root / "logs" / f"production_b{ix:02d}.console.log").resolve()),
                        "status": "pending"})

    # Six fresh-process parity cases and eight batch rows (six primary + two G300 repeats).
    representative: dict[tuple[int, str], dict[str, Any]] = {}
    for j in jobs:
        key = (j["grid"], j["regime"])
        if key not in representative or int(j["seed"]) < int(representative[key]["seed"]):
            representative[key] = j
    if set(representative) != {(g, r) for g in (100, 200, 300) for r in ("static", "dynamic")}:
        raise ValueError(f"Historical source lacks parity strata for {arm}")
    parity_jobs: list[dict[str, Any]] = []
    parity_batches = []
    fresh_map = {}
    for grid, regime in sorted(representative):
        source_job = representative[(grid, regime)]
        fresh = _parity_clone(arm, spec, root, source_job, "fresh", False, None)
        parity_jobs.append(fresh); fresh_map[(grid, regime)] = fresh
    for grid in (100, 200, 300):
        group = []
        for regime in ("static", "dynamic"):
            src = representative[(grid, regime)]
            primary = _parity_clone(arm, spec, root, src, "batch", False, fresh_map[(grid, regime)]["run_id"])
            parity_jobs.append(primary); group.append(primary)
            if grid == 300:
                repeat = _parity_clone(arm, spec, root, src, "batch_repeat", True, fresh_map[(grid, regime)]["run_id"])
                repeat["repeat_of"] = primary["run_id"]
                parity_jobs.append(repeat); group.append(repeat)
        entries = [j["config_path"] for j in group]
        mp = root / "parity" / "batch_manifests" / f"g{grid}.txt"
        mp.parent.mkdir(parents=True, exist_ok=True)
        mp.write_text("\n".join(entries) + "\n", encoding="utf-8")
        cmd = [str(ENGINE), str(PROJECT / "Thesis.uproject"), "NewMap", "-game", "-RenderOffscreen",
               "-unattended", "-nop4", "-nosplash", "-nosound", "-stdout", "-FullStdOutLogOutput",
               f"-abslog={(root / 'parity' / 'logs' / f'batch_g{grid}.engine.log').resolve()}",
               f"-SimConfigManifest={mp.resolve()}"]
        parity_batches.append({"batch_id": f"{arm}_parity_batch_g{grid}", "size": len(group), "jobs": group,
                              "manifest_path": str(mp.resolve()), "manifest_sha256": sha(mp), "manifest_entries": entries,
                              "command": cmd, "console_log": str((root / 'parity' / 'logs' / f'batch_g{grid}.console.log').resolve()),
                              "status": "pending"})
    for j in parity_jobs:
        if j["cohort_role"] in ("parity_fresh",):
            j["command"] = _single_command(root, j)
            j["console_log"] = str((root / "parity" / "logs" / f"{j['run_id']}.console.log").resolve())
    return {"schema_version": 1, "arm": "v1_aligned" if arm == "aligned" else arm,
            "arm_id": "v1_aligned" if arm == "aligned" else arm,
            "namespace": root.name, "root": str(root.resolve()),
            "planner": spec["planner"], "method_ids": [f"{spec['prefix']}_static", f"{spec['prefix']}_dynamic"],
            "production_expected_runs": 3240, "ticks": TICKS, "max_concurrent_processes": MAX_PROCESSES,
            "inherited_parameter_pins": pins, "production_batches": batches,
            "parity_jobs": parity_jobs, "parity_batches": parity_batches,
            "source_pinned_manifest_sha256": sha(V1_ROOT / "manifest_preexecution.json"),
            "source_manifest_sha256": sha(V1_ROOT / "manifest_preexecution.json"),
            "source_v1_manifest_sha256": sha(V1_MANIFEST), "source_production_manifest_sha256": sha(V1_PRODUCTION)}


def _parity_clone(arm: str, spec: dict[str, Any], root: Path, source: dict[str, Any], role: str,
                  repeat: bool, baseline: str | None) -> dict[str, Any]:
    suffix = f"g{source['grid']}_{source['regime']}"
    rid = f"{arm}_parity_{role}_{suffix}" + ("_repeat" if repeat else "")
    cfg = read(Path(source["config_path"]))
    cfg["run_id"] = rid
    cfg["benchmark_run_subdir"] = f"{root.name}/parity/runs/{rid}"
    cfg["metrics_detail_mode"] = "full"
    cp = root / "parity" / "configs" / f"{rid}.json"
    cp.parent.mkdir(parents=True, exist_ok=True)
    cp.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    run_dir = root / "parity" / "runs" / rid
    return {"run_id": rid, "source_run_id": source["source_run_id"], "scenario_id": source["scenario_id"],
            "seed": source["seed"], "grid": source["grid"], "regime": source["regime"], "phase": "parity",
            "cohort_role": "parity_fresh" if role == "fresh" else ("parity_batch_repeat" if repeat else "parity_batch"),
            "comparison_key": f"seed{source['seed']}_g{source['grid']}_{source['regime']}",
            "repeat": repeat, "repeat_of": baseline if repeat else None,
            "parity_baseline_run_id": baseline if role.startswith("batch") else None,
            "expected_pinned_parameters": source["expected_pinned_parameters"], "input_hashes": source["input_hashes"],
            "source_config_path": source["config_path"], "source_config_sha256": source["config_sha256"],
            "config_path": str(cp.resolve()), "config_sha256": sha(cp), "run_dir": str(run_dir.resolve()),
            "runtime_config_path": str((run_dir / "config.json").resolve()), "status": "pending"}


def _single_command(root: Path, job: dict[str, Any]) -> list[str]:
    return [str(ENGINE), str(PROJECT / "Thesis.uproject"), "NewMap", "-game", "-RenderOffscreen",
            "-unattended", "-nop4", "-nosplash", "-nosound", "-stdout", "-FullStdOutLogOutput",
            f"-abslog={(root / 'parity' / 'logs' / (job['run_id'] + '.engine.log')).resolve()}",
            f"-SimConfig={Path(job['config_path']).resolve()}"]


def _snapshot_files() -> list[Path]:
    paths = [PROJECT / "Thesis.uproject", MODULE, ENGINE, V1_ROOT / "manifest_preexecution.json",
             V1_ROOT / "provenance_pins.json", V1_PRODUCTION, V1_ROOT / "final_completion_receipt.json",
             V1_MANIFEST, PROJECT / "Tools/verify_lawnmower_pinned_v2.py",
             PROJECT / "Tools/run_lawnmower_pinned_validation.py", PROJECT / "Tools/run_lawnmower_pinned_production.py",
             PROJECT / "Tools/run_lawnmower_followup_production.py", PROJECT / "Tools/verify_lawnmower_followup_validation.py"]
    for folder in (PROJECT / "Source", PROJECT / "Config", PROJECT / "Shaders", PROJECT / "Plugins"):
        paths.extend(p for p in folder.rglob("*") if p.is_file())
    for root in (V1_ROOT, BENCH / "lawnmower_v1"):
        for folder in (root / "configs", root / "batch_manifests"):
            if folder.exists(): paths.extend(p for p in folder.rglob("*") if p.is_file())
    return sorted(set(p.resolve() for p in paths if p.is_file()))


def prepare(controller: Path) -> dict[str, Any]:
    if controller.exists():
        raise FileExistsError(f"Refusing existing controller namespace: {controller}")
    pinned, v1, prod = source_rows()
    pins, maps = verify_source(pinned, v1, prod)
    historical = historical_output_audit(prod)
    controller.mkdir(parents=True)
    snap = controller / "provenance_snapshot"
    snap.mkdir()
    frozen = copied_snapshot(PROJECT, snap, _snapshot_files())
    source_jobs = []
    for arm in ("v2", "aligned"):
        arm_plan = build_arm(arm, pins, maps, controller)
        arm_plan["prepared_module_sha256"] = sha(MODULE)
        arm_plan["module_sha256"] = sha(MODULE)
        arm_manifest = Path(arm_plan["root"]) / "manifest_preexecution.json"
        atomic(arm_manifest, arm_plan)
        arm_plan["arm_preexecution_manifest_path"] = str(arm_manifest.resolve())
        arm_plan["arm_preexecution_manifest_sha256"] = sha(arm_manifest)
        source_jobs.append(arm_plan)
    # Include launch config hashes in the frozen provenance inventory after creation.
    for arm_plan in source_jobs:
        root = Path(arm_plan["root"])
        for job in arm_plan["production_batches"]:
            for row in job["jobs"]:
                frozen.append({"path": row["config_path"], "snapshot_path": str(snap / "generated_configs" / arm_plan["arm"] / f"{row['run_id']}.json"),
                               "bytes": Path(row["config_path"]).stat().st_size, "sha256": row["config_sha256"]})
                out = Path(frozen[-1]["snapshot_path"]); out.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(row["config_path"], out)
        for row in arm_plan["parity_jobs"]:
            frozen.append({"path": row["config_path"], "snapshot_path": str(snap / "generated_configs" / arm_plan["arm"] / f"{row['run_id']}.json"),
                           "bytes": Path(row["config_path"]).stat().st_size, "sha256": row["config_sha256"]})
            out = Path(frozen[-1]["snapshot_path"]); out.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(row["config_path"], out)
        for batch in arm_plan["production_batches"] + arm_plan["parity_batches"]:
            src = Path(batch["manifest_path"])
            dest = snap / "generated_manifests" / arm_plan["arm"] / src.name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, dest)
            frozen.append({"path": str(src.resolve()), "snapshot_path": str(dest.resolve()),
                           "bytes": src.stat().st_size, "sha256": batch["manifest_sha256"]})
    base = {"schema_version": 1, "phase": "preparation_only", "launch_authorized": False,
            "created_utc": utc(), "controller_root": str(controller.resolve()),
            "arm_roots": {k: str(v["root"].resolve()) for k, v in ARM_SPECS.items()},
            "source_pinned_manifest_path": str((V1_ROOT / "manifest_preexecution.json").resolve()),
            "source_pinned_manifest_sha256": sha(V1_ROOT / "manifest_preexecution.json"),
            "source_v1_manifest_path": str(V1_MANIFEST.resolve()), "source_v1_manifest_sha256": sha(V1_MANIFEST),
            "source_production_manifest_path": str(V1_PRODUCTION.resolve()),
            "source_production_manifest_sha256": sha(V1_PRODUCTION), "historical_v1_output_audit_at_prepare": historical,
            "inherited_parameter_pins": pins, "production_runs_per_arm": 3240, "ticks": TICKS,
            "batch_count_per_arm": 9, "max_batch_size": 500, "max_concurrent_processes": MAX_PROCESSES,
            "engine_path": str(ENGINE), "engine_sha256": sha(ENGINE), "module_path": str(MODULE),
            "prepared_module_sha256": sha(MODULE), "provenance_snapshot": frozen,
            "arms": source_jobs, "validation_receipt_path": str((controller / "validation_receipt.json").resolve()),
            "validation_execution_path": str((controller / "validation_execution.json").resolve())}
    atomic(controller / "manifest_preexecution.json", base)
    return base


def verify_frozen(manifest: dict[str, Any], module_hash: str) -> dict[str, Any]:
    if sha(MODULE) != module_hash or module_hash != manifest.get("prepared_module_sha256"):
        raise ValueError("Current compiled module differs from the module frozen at preparation")
    if not ENGINE.is_file() or sha(ENGINE) != manifest.get("engine_sha256"):
        raise ValueError("UE executable differs from the executable frozen at preparation")
    checked = 0
    for item in manifest.get("provenance_snapshot", []):
        src, snap = Path(item["path"]), Path(item["snapshot_path"])
        if not src.is_file() or src.stat().st_size != item["bytes"] or sha(src) != item["sha256"]:
            raise ValueError(f"Frozen source/config/build/helper changed: {src}")
        if not snap.is_file() or snap.stat().st_size != item["bytes"] or sha(snap) != item["sha256"]:
            raise ValueError(f"Provenance snapshot changed: {snap}")
        checked += 1
    for arm in manifest.get("arms", []):
        for batch in arm.get("production_batches", []) + arm.get("parity_batches", []):
            path = Path(batch["manifest_path"])
            if not path.is_file() or sha(path) != batch["manifest_sha256"]:
                raise ValueError(f"Frozen queue manifest changed: {path}")
            rows = path.read_text(encoding="utf-8-sig").splitlines()
            if rows != [j["config_path"] for j in batch["jobs"]]:
                raise ValueError(f"Frozen queue manifest entries changed: {path}")
        for batch in arm.get("production_batches", []):
            for job in batch.get("jobs", []):
                for rec in job.get("input_hashes", {}).values():
                    path = Path(rec["path"])
                    if not path.is_file() or sha(path) != rec["sha256"]:
                        raise ValueError(f"Frozen input changed: {path}")
    if checked < 100:
        raise ValueError(f"Incomplete provenance snapshot: {checked} files")
    return {"status": "passed", "files_checked": checked, "module_sha256": module_hash}


def verify_job_inputs(job: dict[str, Any]) -> None:
    cp = Path(job["config_path"])
    if not cp.is_file() or sha(cp) != job["config_sha256"]:
        raise ValueError(f"Frozen launch config changed: {cp}")
    for name, rec in job.get("input_hashes", {}).items():
        p = Path(rec["path"])
        if not p.is_file() or sha(p) != rec["sha256"]:
            raise ValueError(f"Frozen {name} input changed: {p}")


def validate_runtime(job: dict[str, Any], arm: str) -> None:
    runtime = validation.validate_runtime_config(job)
    cfg = read(Path(job["config_path"]))
    for key in ("method_id", "planner_strategy", "partition_strategy", "region_regime"):
        if runtime.get(key) != cfg.get(key):
            raise ValueError(f"Runtime selector differs for {job['run_id']}: {key}")
    if int(runtime.get("max_sim_ticks", 0)) != TICKS:
        raise ValueError(f"Runtime tick cap is not 8,000 for {job['run_id']}")
    for key, expected in job["expected_pinned_parameters"].items():
        if key not in runtime or not close_enough(runtime[key], expected):
            raise ValueError(f"Runtime inherited pin differs for {job['run_id']}: {key}")
    if arm == "v2":
        expected = {"lawnmower_v2_algorithm_version": 2, "lawnmower_v2_lane_spacing_cells": 4,
                    "lawnmower_v2_lane_spacing_rule": "max1_twice_floor_range_cells_sin_half_fov",
                    "lawnmower_v2_scan_order": "spawn_side_inward_global_lanes_serpentine",
                    "lawnmower_v2_progress_policy": "shared_completed_waypoints_server_slot_global_epoch",
                    "lawnmower_v2_initial_waypoint": "nearest_unfinished_owned_lane_direct_flight"}
    else:
        expected = {"lawnmower_v1_aligned_algorithm_version": 1, "lawnmower_v1_aligned_lane_spacing_cells": 1,
                    "lawnmower_v1_aligned_scan_order": "spawn_side_inward_one_cell_serpentine",
                    "lawnmower_v1_aligned_initial_waypoint": "owned_spawn_cell_direct_flight",
                    "lawnmower_v1_aligned_waypoint_policy": "arrival_only_direct_flight_v1_progress",
                    "lawnmower_v1_aligned_static_partition": "SpawnAwareUniform"}
    for key, expected_value in expected.items():
        if runtime.get(key) != expected_value:
            raise ValueError(f"Runtime algorithm metadata mismatch {job['run_id']} {key}: {runtime.get(key)!r}")
    job["runtime_config_sha256"] = sha(Path(job["runtime_config_path"]))
    job["runtime_pins_checked_utc"] = utc()


def validate_done(job: dict[str, Any], arm: str) -> None:
    validate_runtime(job, arm)
    validation.validate_completion(job)
    run_dir = Path(job["run_dir"])
    summary = run_dir / "summary.csv"
    if not summary.is_file() or summary.stat().st_size == 0:
        raise ValueError(f"Missing summary.csv: {job['run_id']}")
    with summary.open(newline="", encoding="utf-8-sig") as f:
        records = list(csv.DictReader(f))
    if not records or int(float(records[-1]["final_tick"])) != TICKS:
        raise ValueError(f"Run did not produce an 8,000-tick summary: {job['run_id']}")
    job["summary_final_tick"] = TICKS
    job["artifact_hashes"] = {p.name: sha(p) for p in run_dir.iterdir() if p.is_file()}
    job["status"] = "runtime_evidence_passed"
    job["finished_utc"] = utc()


def launch_wave(jobs: list[dict[str, Any]], arm: str, max_workers: int, timeout: int,
                runtime_path: Path, phase_name: str, live_status_path: Path | None = None,
                spawn_guard: Any | None = None) -> None:
    pending = iter(jobs)
    active: dict[subprocess.Popen, tuple[list[dict[str, Any]], float, Any]] = {}
    failed = False
    last_progress: dict[str, float] = {}
    last_manifest_write = 0.0
    try:
        while active or not failed:
            while not failed and len(active) < max_workers:
                try:
                    group = next(pending)
                except StopIteration:
                    break
                for job in group:
                    verify_job_inputs(job)
                    d = Path(job["run_dir"])
                    if d.exists():
                        raise FileExistsError(f"Refusing existing run directory: {d}")
                    d.mkdir(parents=True)
                logpath = Path(group[0].get("console_log") or group[0]["_console_log"])
                logpath.parent.mkdir(parents=True, exist_ok=True)
                log = logpath.open("w", encoding="utf-8", errors="replace")
                cmd = group[0]["command"]
                try:
                    if spawn_guard:
                        spawn_guard()
                    proc = subprocess.Popen(cmd, cwd=PROJECT, stdout=log, stderr=subprocess.STDOUT,
                                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0) |
                                            getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
                except BaseException:
                    log.close()
                    raise
                bid = group[0].get("batch_id", group[0]["run_id"])
                last_progress[bid] = time.monotonic()
                for j in group:
                    j.update(status="running", pid=proc.pid, started_utc=utc())
                active[proc] = (group, time.monotonic(), log)
            if not active:
                break
            time.sleep(POLL_SECONDS)
            batch_finished = False
            for proc, (group, start, log) in list(active.items()):
                try:
                    for job in group:
                        if job["status"] == "running" and Path(job["runtime_config_path"]).is_file():
                            try:
                                validate_runtime(job, arm)
                                job["status"] = "runtime_pins_passed"
                                last_progress[group[0].get("batch_id", group[0]["run_id"])] = time.monotonic()
                            except json.JSONDecodeError:
                                pass
                    now = time.monotonic()
                    bid = group[0].get("batch_id", group[0]["run_id"])
                    rc = proc.poll()
                    # In a UE batch, the next effective-config snapshot means the previous
                    # run has finished writing its artifacts. Validate it immediately.
                    for idx, job in enumerate(group[:-1]):
                        if job["status"] == "runtime_pins_passed" and Path(group[idx + 1]["runtime_config_path"]).is_file():
                            validate_done(job, arm)
                            last_progress[bid] = time.monotonic()
                    if rc is None and (now - start > timeout or now - last_progress[bid] > NO_PROGRESS_SECONDS):
                        raise TimeoutError(f"{phase_name} {bid} exceeded timeout/no-progress limit")
                except BaseException as exc:
                    for j in group:
                        j.update(status="runtime_mismatch", failure_reason=str(exc))
                    failed = True
                    if proc.poll() is None:
                        proc.terminate()
                    try:
                        rc = proc.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        proc.kill(); rc = proc.wait()
                if rc is None:
                    continue
                batch_finished = True
                log.close()
                for j in group:
                    j["return_code"] = rc
                if rc == 0 and not failed:
                    try:
                        for j in group:
                            old_hashes = j.get("artifact_hashes")
                            validate_done(j, arm)
                            if old_hashes is not None and old_hashes != j["artifact_hashes"]:
                                raise ValueError(f"Artifacts changed after completion was recorded: {j['run_id']}")
                    except BaseException as exc:
                        failed = True
                        for j in group:
                            j.update(status="evidence_failed", failure_reason=str(exc))
                else:
                    failed = True
                    for j in group:
                        if j["status"] == "running":
                            j["status"] = "process_failed"
                        j.setdefault("failure_reason", f"UE exited with {rc}")
                del active[proc]
            flat_jobs = [j for group in jobs for j in group]
            if batch_finished or time.monotonic() - last_manifest_write >= 30:
                current_state = {"phase": phase_name, "arm": arm, "updated_utc": utc(), "jobs": flat_jobs}
                atomic(runtime_path, current_state)
                last_manifest_write = time.monotonic()
            if live_status_path:
                atomic(live_status_path, {"updated_utc": utc(), "phase": phase_name, "arm": arm,
                       "current_arm": "v1_aligned" if arm == "aligned" else arm,
                       "next_arm": "v1_aligned" if arm == "v2" and "production" in phase_name else None,
                       "active_processes": len(active), "max_concurrent_processes": max_workers,
                       "runs_completed": sum(j.get("status") == "runtime_evidence_passed" for j in flat_jobs),
                       "runs_total": len(flat_jobs), "running": [g[0].get("batch_id", g[0]["run_id"]) for g, _, _ in active.values()]})
            if failed:
                break
    finally:
        for proc, (group, start, log) in list(active.items()):
            try:
                if proc.poll() is None:
                    proc.terminate()
                try:
                    rc = proc.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    proc.kill(); rc = proc.wait()
                for j in group:
                    if j["status"] in ("running", "runtime_pins_passed"):
                        j.update(status="stopped_after_peer_failure", return_code=rc,
                                 failure_reason="stopped after peer/controller failure", finished_utc=utc())
            finally:
                log.close()
        flat_jobs = [j for group in jobs for j in group]
        final_status = "passed" if not failed and all(j.get("status") == "runtime_evidence_passed" for j in flat_jobs) else "failed"
        atomic(runtime_path, {"phase": phase_name, "arm": arm, "updated_utc": utc(),
                              "jobs": flat_jobs, "status": final_status})
        if live_status_path:
            flat_jobs = [j for group in jobs for j in group]
            atomic(live_status_path, {"updated_utc": utc(), "phase": phase_name, "arm": arm,
                   "current_arm": "v1_aligned" if arm == "aligned" else arm,
                   "active_processes": 0, "runs_completed": sum(j.get("status") == "runtime_evidence_passed" for j in flat_jobs),
                   "runs_total": len(flat_jobs), "status": final_status})
    if failed:
        raise RuntimeError(f"{phase_name} failed for {arm}; outputs are preserved")


def start_lock(controller: Path) -> Path:
    validation.assert_no_project_editor_process()
    lock = controller / ".followup_execution.lock"
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as e:
        raise RuntimeError(f"Follow-up execution lock exists: {lock}") from e
    os.write(fd, f"pid={os.getpid()} started={utc()}\n".encode())
    os.close(fd)
    return lock


def _check_live_build(manifest: dict[str, Any], module_hash: str) -> None:
    if not MODULE.is_file() or sha(MODULE) != module_hash:
        raise ValueError("Compiled module changed during queued execution")
    if not ENGINE.is_file() or sha(ENGINE) != manifest.get("engine_sha256"):
        raise ValueError("UE executable changed during queued execution")


def all_parity_groups(plan: dict[str, Any]) -> list[list[dict[str, Any]]]:
    fresh = [[j] for j in plan["parity_jobs"] if j["cohort_role"] == "parity_fresh"]
    batch = [b["jobs"] for b in plan["parity_batches"]]
    return fresh + batch


def run_validation(manifest: dict[str, Any], controller: Path, module_hash: str, timeout: int) -> None:
    audit_before = historical_output_audit(read(V1_PRODUCTION))
    frozen_before = verify_frozen(manifest, module_hash)
    lock = start_lock(controller)
    execution = {"schema_version": 1, "phase": "parity_validation", "status": "running",
                 "module_sha256": module_hash, "started_utc": utc(), "arms": {}}
    atomic(controller / "validation_execution.json", execution)
    try:
        for arm in ("v2", "aligned"):
            plan = next(x for x in manifest["arms"] if x["arm"] == ("v1_aligned" if arm == "aligned" else arm))
            jobs = plan["parity_jobs"]
            for group in all_parity_groups(plan):
                if len(group) == 1:
                    group[0]["command"] = _single_command(Path(plan["root"]), group[0])
                    group[0]["console_log"] = str((Path(plan["root"]) / "parity/logs" / f"{group[0]['run_id']}.console.log").resolve())
            for b in plan["parity_batches"]:
                for j in b["jobs"]:
                    j["console_log"] = b["console_log"]
                    j["batch_id"] = b["batch_id"]
                    j["command"] = b["command"]
            # Each one-run fresh process at most 3 concurrent; then batch mode uses one UE process per grid.
            fresh = [[j] for j in jobs if j["cohort_role"] == "parity_fresh"]
            launch_wave(fresh, arm, MAX_PROCESSES, timeout, Path(plan["root"]) / "parity_runtime.json",
                        f"{arm}_parity_fresh", spawn_guard=lambda: _check_live_build(manifest, module_hash))
            launch_wave([b["jobs"] for b in plan["parity_batches"]], arm, MAX_PROCESSES, timeout,
                        Path(plan["root"]) / "parity_runtime.json", f"{arm}_parity_batch",
                        spawn_guard=lambda: _check_live_build(manifest, module_hash))
            execution["arms"][arm] = {"status": "passed", "module_sha256": module_hash,
                                      "jobs": jobs, "batch_groups": plan["parity_batches"]}
            atomic(controller / "validation_execution.json", execution)
            atomic(Path(plan["root"]) / "manifest_runtime.json",
                   {"phase": "parity_validation", "arm": "v1_aligned" if arm == "aligned" else arm,
                    "arm_id": "v1_aligned" if arm == "aligned" else arm,
                    "status": "passed", "overall_status": "passed", "module_sha256": module_hash,
                    "parity_jobs": jobs, "parity_batches": plan["parity_batches"]})
        # Immutable binding file, written once and never changed by production progress.
        execution.update(status="passed", finished_utc=utc(), historical_v1_output_audit_before=audit_before,
                         provenance_audit_before=frozen_before)
        atomic(controller / "validation_execution.json", execution)
        audit_after = historical_output_audit(read(V1_PRODUCTION))
        frozen_after = verify_frozen(manifest, module_hash)
        execution.update(historical_v1_output_audit_after=audit_after, provenance_audit_after=frozen_after)
        atomic(controller / "validation_execution.json", execution)
        verifier = PROJECT / "Tools/verify_lawnmower_followup_validation.py"
        subprocess.run([sys.executable, str(verifier), "--controller-root", str(controller),
                        "--pilot-root", str(PILOT_ROOT), "--v2-root", str(ARM_SPECS["v2"]["root"]),
                        "--v1-aligned-root", str(ARM_SPECS["aligned"]["root"])], cwd=PROJECT, check=True)
        receipt = read(controller / "validation_receipt.json")
        if receipt.get("status") != "passed":
            raise ValueError("Independent parity validation did not pass")
    except BaseException as exc:
        execution.update(status="failed", failure_reason=repr(exc), finished_utc=utc())
        atomic(controller / "validation_execution.json", execution)
        raise
    finally:
        lock.unlink(missing_ok=True)


def require_receipt(controller: Path, manifest: dict[str, Any], module_hash: str) -> dict[str, Any]:
    receipt_path = controller / "validation_receipt.json"
    execution_path = controller / "validation_execution.json"
    if not receipt_path.is_file() or not execution_path.is_file():
        raise FileNotFoundError("Parity validation receipt is missing; production remains gated")
    receipt, execution = read(receipt_path), read(execution_path)
    if execution.get("status") != "passed" or execution.get("module_sha256") != module_hash:
        raise ValueError("Parity execution is incomplete or bound to another build")
    expected = {"controller_preexecution_sha256": sha(controller / "manifest_preexecution.json"),
                "validation_execution_sha256": sha(execution_path), "current_module_sha256": module_hash}
    for k, v in expected.items():
        if receipt.get(k) != v:
            raise ValueError(f"Validation receipt binding mismatch: {k}")
    for arm in manifest["arms"]:
        aid = arm["arm"]
        receipt_key = "v1_aligned" if aid == "v1_aligned" else aid
        recorded = receipt.get("arm_manifests", {}).get(receipt_key, {})
        pre_path = Path(arm["arm_preexecution_manifest_path"])
        runtime_path = pre_path.parent / "manifest_runtime.json"
        if (recorded.get("preexecution_sha256") != sha(pre_path) or
                not runtime_path.is_file() or recorded.get("runtime_sha256") != sha(runtime_path) or
                recorded.get("module_sha256") != module_hash):
            raise ValueError(f"Validation receipt arm-manifest binding mismatch: {aid}")
    if receipt.get("status") != "passed":
        raise ValueError("Independent parity receipt does not pass")
    return receipt


def run_production(manifest: dict[str, Any], controller: Path, module_hash: str, timeout: int) -> None:
    verify_frozen(manifest, module_hash)
    receipt = require_receipt(controller, manifest, module_hash)
    audit = historical_output_audit(read(V1_PRODUCTION))
    lock = start_lock(controller)
    runtime = {"schema_version": 1, "phase": "production", "status": "running", "module_sha256": module_hash,
               "validation_receipt_sha256": sha(controller / "validation_receipt.json"),
               "started_utc": utc(), "current_arm": "v2", "queued_arm": "v1_aligned",
               "arms": [{"arm": "v2", "status": "queued", "batches_total": 9},
                        {"arm": "v1_aligned", "status": "queued", "batches_total": 9}]}
    atomic(controller / "production_runtime.json", runtime)
    try:
        for arm in ("v2", "aligned"):
            if arm == "aligned":
                verify_frozen(manifest, module_hash)
                historical_output_audit(read(V1_PRODUCTION))
            plan = next(x for x in manifest["arms"] if x["arm"] == ("v1_aligned" if arm == "aligned" else arm))
            root = Path(plan["root"])
            batches = plan["production_batches"]
            aid = "v1_aligned" if arm == "aligned" else arm
            for rec in runtime["arms"]:
                if rec["arm"] == aid:
                    rec["status"] = "running"; rec["started_utc"] = utc()
            runtime["current_arm"] = aid
            atomic(controller / "production_runtime.json", runtime)
            for batch in batches:
                batch["console_log"] = str((root / "logs" / Path(batch["console_log"]).name).resolve())
                batch["jobs"][0]["command"] = batch["command"]
                for j in batch["jobs"]:
                    j["console_log"] = batch["console_log"]
                    j["batch_id"] = batch["batch_id"]
                    # `launch_wave` accepts a list of process groups; attach the batch command on each row.
                    j["command"] = batch["command"]
            for batch in batches:
                for j in batch["jobs"]:
                    j["_console_log"] = batch["console_log"]
                    j["batch_id"] = batch["batch_id"]
                    j["command"] = batch["command"]
            groups = [b["jobs"] for b in batches]
            launch_wave(groups, arm, MAX_PROCESSES, timeout,
                        root / "production_runtime.json", f"{arm}_production", controller / "live_status.json",
                        spawn_guard=lambda: _check_live_build(manifest, module_hash))
            for batch in batches:
                batch["status"] = "runtime_evidence_passed"
            for rec in runtime["arms"]:
                if rec["arm"] == aid:
                    rec.update(status="passed", finished_utc=utc(), batches_completed=9, production_manifest_path=str((root / "manifest_preexecution.json").resolve()))
            atomic(controller / "production_runtime.json", runtime)
            # The historical 3,240 V1 outputs remain the baseline and are audited between arms.
            historical_output_audit(read(V1_PRODUCTION))
        verify_frozen(manifest, module_hash)
        historical_after = historical_output_audit(read(V1_PRODUCTION))
        runtime.update(status="passed", finished_utc=utc(), historical_v1_output_audit_before=audit,
                       historical_v1_output_audit_after=historical_after, validation_receipt_status=receipt["status"])
        atomic(controller / "production_runtime.json", runtime)
    except BaseException as exc:
        runtime.update(status="failed", finished_utc=utc(), failure_reason=repr(exc))
        atomic(controller / "production_runtime.json", runtime)
        raise
    finally:
        lock.unlink(missing_ok=True)


def load_prepared(controller: Path, module_hash: str) -> dict[str, Any]:
    path = controller / "manifest_preexecution.json"
    if not path.is_file():
        raise FileNotFoundError(f"Prepared controller manifest missing: {path}")
    manifest = read(path)
    if manifest.get("prepared_module_sha256") != module_hash:
        raise ValueError("Current module SHA differs from the preparation; use a new output namespace")
    verify_frozen(manifest, module_hash)
    # Require arm plans on disk to be byte-stable with the controller record.
    for arm in manifest["arms"]:
        if not Path(arm["root"]).is_dir():
            raise ValueError(f"Prepared arm root missing: {arm['root']}")
        arm_manifest = Path(arm["arm_preexecution_manifest_path"])
        if not arm_manifest.is_file() or sha(arm_manifest) != arm["arm_preexecution_manifest_sha256"]:
            raise ValueError(f"Arm preparation manifest changed: {arm_manifest}")
        parity_by_id = {j["run_id"]: j for j in arm["parity_jobs"]}
        for batch in arm["parity_batches"]:
            batch["jobs"] = [parity_by_id[j["run_id"]] for j in batch["jobs"]]
            _verify_batch_manifest(batch)
        for batch in arm["production_batches"]:
            # JSON serialization duplicates the `jobs`/`runs` arrays; rebind to a single canonical list.
            batch["runs"] = batch["jobs"]
            _verify_batch_manifest(batch)
        for batch in arm["production_batches"]:
            for job in batch["jobs"]: verify_job_inputs(job)
        for job in arm["parity_jobs"]: verify_job_inputs(job)
    return manifest


def _verify_batch_manifest(batch: dict[str, Any]) -> None:
    path = Path(batch["manifest_path"])
    if not path.is_file() or sha(path) != batch["manifest_sha256"]:
        raise ValueError(f"Frozen batch manifest changed: {path}")
    entries = path.read_text(encoding="utf-8-sig").splitlines()
    expected = [j["config_path"] for j in batch["jobs"]]
    if entries != expected or len(entries) != batch["size"] or batch["size"] > 500:
        raise ValueError(f"Frozen batch entry list/order/size mismatch: {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    modes = ap.add_mutually_exclusive_group(required=True)
    modes.add_argument("--prepare", action="store_true")
    modes.add_argument("--validate", action="store_true")
    modes.add_argument("--run", action="store_true")
    ap.add_argument("--output", type=Path, default=CONTROLLER)
    ap.add_argument("--module-sha256", required=True)
    ap.add_argument("--timeout-seconds", type=int, default=172800)
    args = ap.parse_args()
    controller = args.output.resolve()
    if not controller.is_relative_to(BENCH.resolve()):
        raise ValueError(f"Controller output must be under {BENCH}")
    if not ENGINE.is_file() or not MODULE.is_file():
        raise FileNotFoundError("UE engine or compiled module is missing")
    module_hash = args.module_sha256.lower()
    if sha(MODULE) != module_hash:
        raise ValueError("Current compiled module does not match --module-sha256")
    if args.prepare:
        manifest = prepare(controller)
        if manifest["prepared_module_sha256"] != module_hash:
            raise ValueError("Module changed during preparation")
        print(json.dumps({"status": "prepared_not_launched", "controller": str(controller),
                          "arms": manifest["arm_roots"], "runs_per_arm": 3240,
                          "parity_jobs_per_arm": 14, "max_concurrent_processes": MAX_PROCESSES}, indent=2))
        return 0
    manifest = load_prepared(controller, module_hash)
    if args.validate:
        run_validation(manifest, controller, module_hash, args.timeout_seconds)
        print(json.dumps({"status": "parity_validation_passed", "controller": str(controller)}, indent=2))
        return 0
    run_production(manifest, controller, module_hash, args.timeout_seconds)
    print(json.dumps({"status": "production_passed", "controller": str(controller)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

