#!/usr/bin/env python3
"""Prepare or run isolated full-length Lawnmower pinned-default validation.

Default mode validates inputs and prints the six-case plan without launching.
Use --run to execute it. Runtime evidence is written under a fresh namespace,
separate from the prepared production queue. RHI stays enabled; no NullRHI.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import time
from typing import Any

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = PROJECT / "Saved/Benchmarks/lawnmower_pinned_v2"
ENGINE = Path(r"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe")
MAX_PROCESSES = 3
TICKS = 8000
EXPECTED_MODULE_SHA256 = "85bc3739ee3dc653615971bb445762087ed26c6a04d2651fed0e8496e2eca9ba"


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def write_atomic(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(temp, path)


def same_value(actual: Any, expected: Any) -> bool:
    if type(actual) is bool or type(expected) is bool:
        return type(actual) is type(expected) and actual == expected
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        try:
            return actual == expected or struct.pack("f", actual) == struct.pack("f", expected)
        except OverflowError:
            return actual == expected
    return actual == expected


def reflected_value_matches(actual: Any, expected: Any) -> bool:
    if isinstance(expected, bool):
        if isinstance(actual, str):
            value = actual.strip().lower()
            if expected:
                return value in ("true", "1")
            # Unreal's reflected property exporter represents false bools as
            # empty text for some UPROPERTY types; preserve false semantics.
            return value in ("", "false", "0")
        return type(actual) is bool and actual is expected
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        try:
            return same_value(float(actual), expected)
        except (TypeError, ValueError):
            return False
    return str(actual) == str(expected)


def assert_no_project_editor_process() -> None:
    if os.name != "nt":
        return
    query = "Get-CimInstance Win32_Process | Where-Object { $_.Name -match '^UnrealEditor(-Cmd)?\\.exe$' -and $_.CommandLine -like '*Thesis.uproject*' } | Select-Object -ExpandProperty ProcessId"
    result = subprocess.run(["powershell", "-NoProfile", "-Command", query], capture_output=True,
                            text=True, check=True)
    pids = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if pids:
        raise RuntimeError(f"Project Unreal process already running (PID(s): {', '.join(pids)})")


def select_rows(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    # One repeatable, lexically first scenario per grid × regime. This covers
    # all requested 100/200/300 grids, both regimes, and the queue's f1 terrain.
    chosen: dict[tuple[int, str], dict[str, Any]] = {}
    for row in sorted(manifest["runs"], key=lambda r: (r["grid"], r["regime"], r["scenario_id"])):
        key = (int(row["grid"]), str(row["regime"]))
        if key not in chosen:
            chosen[key] = row
    wanted = {(grid, regime) for grid in (100, 200, 300) for regime in ("static", "dynamic")}
    if set(chosen) != wanted:
        raise ValueError(f"Pinned manifest must contain exactly the six coverage strata; found {sorted(chosen)}")
    return [chosen[k] for k in sorted(chosen)]


def verify_inputs(row: dict[str, Any]) -> dict[str, Any]:
    verified = {}
    for name, record in row.get("input_hashes", row.get("scenario_inputs", {})).items():
        if not isinstance(record, dict) or "path" not in record:
            continue
        path = Path(record["path"]).resolve()
        if not path.is_file() or sha(path) != record["sha256"]:
            raise ValueError(f"Frozen {name} input missing or changed: {path}")
        verified[name] = {"path": str(path), "sha256": record["sha256"], "bytes": path.stat().st_size}
    if not verified:
        raise ValueError(f"No hashed scenario inputs in {row.get('run_id')}")
    return verified


def make_plan(root: Path, manifest_path: Path, out: Path) -> dict[str, Any]:
    manifest = read(manifest_path)
    jobs = []
    configs = out / "configs"
    for index, row in enumerate(select_rows(manifest), 1):
        source = Path(row["config_path"]).resolve()
        if not source.is_file() or sha(source) != row["config_sha256"]:
            raise ValueError(f"Pinned launch config changed: {source}")
        expected = row.get("expected_pinned_parameters")
        if not isinstance(expected, dict) or len(expected) != 49:
            raise ValueError(f"Expected exactly 49 pinned parameters for {row['run_id']}")
        input_hashes = verify_inputs(row)
        cfg = read(source)
        run_id = f"runtime_{index:02d}_{row['grid']}_{row['regime']}_{row['scenario_id']}"
        cfg["run_id"] = run_id
        cfg["benchmark_run_subdir"] = (out.relative_to(PROJECT / "Saved/Benchmarks") / "runs" / run_id).as_posix()
        cfg["max_sim_ticks"] = TICKS
        cfg["exit_on_max_ticks"] = True
        # Runtime provenance and field settings stay as prepared; only isolate
        # outputs and enforce the requested full-length cap.
        cfg["generate_fire_csv"] = False
        cfg["auto_generate_scenario"] = False
        cfg["reuse_generated_scenario_csvs"] = False
        scratch = Path("Saved/Benchmarks") / root.name / "runtime_validation" / out.name / "runtime_scratch" / run_id
        for field, filename in (("generated_fire_csv_output_path", "fire.csv"),
                                ("generated_humans_csv_output_path", "humans.csv"),
                                ("generated_terrain_csv_output_path", "terrain.csv"),
                                ("generated_fuel_csv_output_path", "fuel.csv")):
            cfg[field] = (scratch / filename).as_posix()
        cp = configs / f"{run_id}.json"
        run_dir = (PROJECT / "Saved/Benchmarks" / cfg["benchmark_run_subdir"]).resolve()
        command = [str(ENGINE), str(PROJECT / "Thesis.uproject"), "NewMap", "-game",
                   "-RenderOffscreen", "-unattended", "-nop4", "-nosplash", "-nosound",
                   "-stdout", "-FullStdOutLogOutput", f"-abslog={(out / 'logs' / (run_id + '.engine.log')).resolve()}",
                   f"-SimConfig={cp.resolve()}"]
        jobs.append({"run_id": run_id, "source_run_id": row["run_id"], "scenario_id": row["scenario_id"],
                     "grid": row["grid"], "regime": row["regime"], "seed": row["seed"],
                     "expected_pinned_parameters": expected, "input_hashes": input_hashes,
                     "source_config_path": str(source), "source_config_sha256": row["config_sha256"],
                     "config_path": str(cp.resolve()), "config_sha256": None,
                     "run_dir": str(run_dir), "runtime_config_path": str(run_dir / "config.json"),
                     "runtime_scratch_dir": str(PROJECT / scratch),
                     "runtime_config_sha256": None, "engine_log": str(out / "logs" / (run_id + ".engine.log")),
                     "console_log": str(out / "logs" / f"{run_id}.console.log"),
                     "command": command, "status": "planned"})
        jobs[-1]["_config"] = cfg
    return {"schema_version": 1, "phase": "runtime_validation", "created_utc": utc(),
            "source_manifest_path": str(manifest_path.resolve()), "source_manifest_sha256": sha(manifest_path),
            "engine_path": str(ENGINE), "engine_sha256": sha(ENGINE) if ENGINE.is_file() else None,
            "project_module_path": str(PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"),
            "project_module_sha256": sha(PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"),
            "tick_cap": TICKS, "max_concurrent_processes": MAX_PROCESSES,
            "rhi_enabled": True, "render_suppression_from_pinned_config": True,
            "runs": jobs, "jobs": jobs}


def utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def run(plan: dict[str, Any], out: Path, timeout: int) -> int:
    jobs = plan["jobs"]
    for job in jobs:
        write(Path(job["config_path"]), job.pop("_config"))
        job["config_sha256"] = sha(Path(job["config_path"]))
    write(out / "manifest_runtime.json", plan)
    pending = iter(jobs)
    active: dict[subprocess.Popen[bytes], tuple[dict[str, Any], float, Any]] = {}
    failed = False
    while active or (not failed):
        while not failed and len(active) < MAX_PROCESSES:
            try:
                job = next(pending)
            except StopIteration:
                break
            run_dir = Path(job["run_dir"])
            if run_dir.exists():
                raise FileExistsError(f"Refusing to overwrite runtime output: {run_dir}")
            run_dir.mkdir(parents=True)
            Path(job["engine_log"]).parent.mkdir(parents=True, exist_ok=True)
            Path(job["console_log"]).parent.mkdir(parents=True, exist_ok=True)
            log = Path(job["console_log"]).open("w", encoding="utf-8", errors="replace")
            proc = subprocess.Popen(job["command"], cwd=PROJECT, stdout=log, stderr=subprocess.STDOUT,
                                    creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
            job.update(status="running", pid=proc.pid, started_utc=utc())
            active[proc] = (job, time.monotonic(), log)
        if not active:
            break
        time.sleep(1)
        for proc, (job, started, log) in list(active.items()):
            rc = proc.poll()
            if not job.get("runtime_pins_checked") and Path(job["runtime_config_path"]).is_file():
                try:
                    validate_runtime_config(job)
                    job["runtime_pins_checked"] = True
                    job["runtime_pins_checked_utc"] = utc()
                except json.JSONDecodeError:
                    pass  # Snapshot is still being written; retry on the next poll.
                except Exception as exc:
                    job["status"] = "early_runtime_mismatch"
                    job["failure_reason"] = str(exc)
                    proc.terminate()
                    try: proc.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        proc.kill(); proc.wait()
                    rc = proc.returncode
                    failed = True
            if rc is None and time.monotonic() - started > timeout:
                proc.terminate()
                try: rc = proc.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    proc.kill(); rc = proc.wait()
                rc = 124
                job["failure_reason"] = f"timeout after {timeout}s"
            if rc is None:
                continue
            log.close()
            job.update(return_code=rc, elapsed_seconds=round(time.monotonic() - started, 3), finished_utc=utc())
            if job.get("status") == "early_runtime_mismatch":
                failed = True
            elif rc == 0:
                try:
                    validate_completion(job)
                    job["status"] = "runtime_evidence_passed"
                except Exception as exc:
                    job["status"] = "evidence_failed"
                    job["failure_reason"] = str(exc)
                    failed = True
            else:
                job["status"] = "process_failed"
                failed = True
            del active[proc]
        write_atomic(out / "live_status.json", {"updated_utc": utc(), "max_processes": MAX_PROCESSES,
              "jobs": [{k: j.get(k) for k in ("run_id", "status", "pid", "return_code", "elapsed_seconds", "failure_reason")} for j in jobs]})
        if failed:
            for proc, (job, started, log) in list(active.items()):
                proc.terminate()
                try: job["return_code"] = proc.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    proc.kill(); job["return_code"] = proc.wait()
                job.update(status="stopped_after_peer_failure", finished_utc=utc(),
                           elapsed_seconds=round(time.monotonic() - started, 3))
                log.close(); del active[proc]
            write_atomic(out / "live_status.json", {"updated_utc": utc(), "max_processes": MAX_PROCESSES,
                  "jobs": [{k: j.get(k) for k in ("run_id", "status", "pid", "return_code", "elapsed_seconds", "failure_reason")} for j in jobs]})
            break
    plan["finished_utc"] = utc()
    plan["overall_status"] = "passed" if all(j["status"] == "runtime_evidence_passed" for j in jobs) else "failed_or_incomplete"
    write(out / "manifest_runtime.json", plan)
    write_atomic(out / "live_status.json", {"updated_utc": utc(), "max_processes": MAX_PROCESSES,
          "overall_status": plan["overall_status"],
          "jobs": [{k: j.get(k) for k in ("run_id", "status", "pid", "return_code", "elapsed_seconds", "failure_reason")} for j in jobs]})
    return 0 if plan["overall_status"] == "passed" else 1


def validate_completion(job: dict[str, Any]) -> None:
    run_dir = Path(job["run_dir"])
    snap = Path(job["runtime_config_path"])
    if not snap.is_file(): raise ValueError("Effective runtime config snapshot missing")
    runtime = validate_runtime_config(job)
    for input_key, config_key in (("fire", "fire_csv_path"), ("humans", "humans_csv_path")):
        record = job["input_hashes"].get(input_key)
        if record and Path(runtime.get(config_key, "")).resolve() != Path(record["path"]).resolve():
            raise ValueError(f"Runtime {config_key} does not point to the frozen {input_key} input")
    for name, record in job["input_hashes"].items():
        path = Path(record["path"])
        if not path.is_file() or sha(path) != record["sha256"]:
            raise ValueError(f"Frozen {name} input changed during execution: {path}")
    summary = run_dir / "summary.csv"
    if not summary.is_file() or summary.stat().st_size == 0:
        raise ValueError("summary.csv missing or empty")
    with summary.open(newline="", encoding="utf-8-sig") as stream:
        summary_rows = list(csv.DictReader(stream))
    if not summary_rows or "final_tick" not in summary_rows[-1]:
        raise ValueError("summary.csv has no final_tick evidence row")
    final_tick = int(float(summary_rows[-1]["final_tick"]))
    if final_tick != TICKS:
        raise ValueError(f"summary.csv final_tick is {final_tick}, expected {TICKS}")
    ticks = run_dir / "ticks.csv"
    tick_rows = 0
    if ticks.is_file():
        with ticks.open(newline="", encoding="utf-8-sig") as stream:
            tick_rows = sum(1 for _ in csv.DictReader(stream))
    job["runtime_config_sha256"] = sha(snap)
    job["artifact_hashes"] = {p.name: sha(p) for p in sorted(run_dir.iterdir()) if p.is_file()}
    job["summary_rows"] = len(summary_rows)
    job["summary_final_tick"] = final_tick
    job["tick_rows"] = tick_rows


def validate_runtime_config(job: dict[str, Any]) -> dict[str, Any]:
    snap = Path(job["runtime_config_path"])
    if not snap.is_file(): raise ValueError("Effective runtime config snapshot missing")
    runtime = read(snap)
    resolved = runtime.get("resolved_manager_properties")
    if not isinstance(resolved, dict):
        raise ValueError("resolved_manager_properties missing or not an object in runtime snapshot")
    for key, expected in job["expected_pinned_parameters"].items():
        if key not in runtime or not same_value(runtime[key], expected):
            raise ValueError(f"Pinned runtime field mismatch: {key}: expected {expected!r}, got {runtime.get(key)!r}")
    launch_cfg = read(Path(job["config_path"]))
    explicit_fields = ("method_id", "information_source", "region_regime", "planner_strategy",
                       "partition_strategy", "max_sim_ticks", "grid_width", "grid_height",
                       "fire_seed", "drone_count", "drone_speed_uu_per_tick",
                       "drone_fov_degrees", "drone_max_view_distance_uu", "sensor_class",
                       "render_terrain_visuals", "enable_fire_visuals", "enable_drone_captures",
                       "enable_vision_bridge", "suppress_benchmark_viewport")
    for key in explicit_fields:
        if key in launch_cfg and (key not in runtime or not same_value(runtime[key], launch_cfg[key])):
            raise ValueError(f"Explicit launch field mismatch at runtime: {key}")
    for key in launch_cfg.keys() & runtime.keys():
        if not same_value(runtime[key], launch_cfg[key]):
            raise ValueError(f"Launch/runtime config intersection mismatch: {key}")
    # These are serialized in the exhaustive reflected manager-property map,
    # rather than as top-level runtime snapshot fields.
    for launch_key, reflected_key in (("human_seed", "HumanGeneratorSeed"),
                                      ("human_count", "HumanGeneratorCount"),
                                      ("generate_fire_csv", "bGenerateFireCsvOnBeginPlay"),
                                      ("auto_generate_scenario", "bAutoGenerateScenarioOnBeginPlay")):
        if launch_key in launch_cfg:
            observed = resolved.get(reflected_key)
            if observed is None or not reflected_value_matches(observed, launch_cfg[launch_key]):
                raise ValueError(f"Resolved manager property mismatch: {reflected_key}")
    if runtime.get("sensor_class") != "/Script/Thesis.PerfectDroneSensor":
        raise ValueError(f"Unexpected effective sensor class: {runtime.get('sensor_class')!r}")
    for input_key, config_key in (("fire", "fire_csv_path"), ("humans", "humans_csv_path")):
        record = job["input_hashes"].get(input_key)
        if record and Path(runtime.get(config_key, "")).resolve() != Path(record["path"]).resolve():
            raise ValueError(f"Runtime {config_key} does not point to the frozen {input_key} input")
    return runtime


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--output", type=Path, help="Must be a new folder inside --root/runtime_validation")
    parser.add_argument("--run", action="store_true", help="Launch the six 8,000-tick UE RHI runs")
    parser.add_argument("--timeout-seconds", type=int, default=172800)
    args = parser.parse_args()
    root = args.root.resolve()
    manifest_path = (args.manifest or root / "manifest_preexecution.json").resolve()
    if not manifest_path.is_file(): raise FileNotFoundError(manifest_path)
    if not ENGINE.is_file(): raise FileNotFoundError(ENGINE)
    module = PROJECT / "Binaries/Win64/UnrealEditor-Thesis.dll"
    if not module.is_file(): raise FileNotFoundError(module)
    module_hash = sha(module)
    if module_hash != EXPECTED_MODULE_SHA256:
        raise ValueError(f"Compiled module hash does not match validated Lawnmower build: {module_hash}")
    if args.output:
        out = args.output.resolve()
    else:
        out = root / "runtime_validation" / time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    runtime_parent = (root / "runtime_validation").resolve()
    if not out.is_relative_to(runtime_parent): raise ValueError(f"Output must be inside {runtime_parent}")
    if out.exists(): raise FileExistsError(f"Refusing to reuse output namespace: {out}")
    lock_path = None
    out.mkdir(parents=True)
    plan = make_plan(root, manifest_path, out)
    if not args.run:
        for job in plan["jobs"]: write(Path(job["config_path"]), job.pop("_config"))
        for job in plan["jobs"]: job["config_sha256"] = sha(Path(job["config_path"]))
        plan["overall_status"] = "prepared_not_launched"
        plan["launch_authorized"] = False
        write(out / "manifest_runtime.json", plan)
        print(json.dumps({"status": plan["overall_status"], "output": str(out),
                          "cases": [{k: j[k] for k in ("run_id", "grid", "regime", "seed")} for j in plan["jobs"]],
                          "launch_required": "rerun with --run"}, indent=2))
        return 0
    assert_no_project_editor_process()
    runtime_parent.mkdir(parents=True, exist_ok=True)
    lock_path = runtime_parent / ".runtime_runner.lock"
    try:
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as exc:
        raise RuntimeError(f"Runtime controller lock exists: {lock_path}") from exc
    os.write(lock_fd, f"pid={os.getpid()} started={utc()}\n".encode("ascii"))
    os.close(lock_fd)
    plan["launch_authorized"] = True
    try:
        result = run(plan, out, args.timeout_seconds)
    finally:
        if lock_path is not None:
            lock_path.unlink(missing_ok=True)
    print(json.dumps({"status": plan["overall_status"], "output": str(out),
                      "jobs": [{k: j.get(k) for k in ("run_id", "status", "return_code", "tick_rows", "failure_reason")} for j in plan["jobs"]]}, indent=2))
    return result


if __name__ == "__main__":
    raise SystemExit(main())
