#!/usr/bin/env python3
"""Recompute recent camera-cohort cell-belief comparisons from packaged rows.

Uses only the three included run-summary CSVs and their completed run indexes.
It performs no Unreal run, cache replay, image inference, or reprojection.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data" / "camera_studies"
T95_DF14 = 2.1447866879169273
FACTORS = ("terrain", "flora", "focus", "spread", "seed", "grid", "regime")
SOURCES = {
    "mixed": ("mixed_ideal_plus_camera_run_index.csv", "mixed_ideal_plus_camera_aggregate.csv"),
    "four_camera": ("four_camera_run_index.csv", "four_camera_aggregate.csv"),
    "three_ideal": ("three_ideal_completed_attempt16_run_index.csv", "three_ideal_cell_belief_run_summaries.csv"),
}


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        result = list(reader)
        if not reader.fieldnames:
            raise ValueError(f"missing CSV header: {path}")
        return result


def num(row: dict[str, str], field: str) -> float:
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid {field} in {row.get('run_id')}: {row.get(field)}") from exc
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field} in {row.get('run_id')}")
    return value


def canonical(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(str(row[k] if k != "regime" else row.get("regime", row.get("method_id", "").removeprefix("mine_")).removeprefix("mine_")) for k in FACTORS)


def t_interval(values: list[float]) -> dict:
    mean = statistics.fmean(values)
    if len(values) != 15:
        raise ValueError(f"expected 15 seed-block means, got {len(values)}")
    half = T95_DF14 * statistics.stdev(values) / math.sqrt(15)
    return {"mean": mean, "ci95_low": mean-half, "ci95_high": mean+half,
            "seed_blocks": 15, "df": 14}


def cohort_rows(name: str) -> tuple[list[dict[str, str]], dict[tuple[str, ...], dict[str, str]]]:
    index_name, summary_name = SOURCES[name]
    ix = rows(DATA / index_name)
    summaries = rows(DATA / summary_name)
    if len(ix) != 3240 or len(summaries) != 3240:
        raise ValueError(f"{name}: expected 3240 index and summary rows, got {len(ix)}/{len(summaries)}")
    if any(r.get("status") != "complete" for r in ix):
        raise ValueError(f"{name}: index contains a non-complete run")
    if len({r["run_id"] for r in ix}) != 3240 or len({r["run_id"] for r in summaries}) != 3240:
        raise ValueError(f"{name}: duplicate run ID")
    ix_by_id = {r["run_id"]: r for r in ix}
    sm_by_id = {r["run_id"]: r for r in summaries}
    if set(ix_by_id) != set(sm_by_id):
        raise ValueError(f"{name}: run index and summary IDs differ")
    pairs: dict[tuple[str, ...], dict[str, str]] = {}
    for rid, ir in ix_by_id.items():
        sr = sm_by_id[rid]
        if int(sr.get("final_tick", sr.get("final_tick", "0"))) != 8000:
            raise ValueError(f"{name}/{rid}: summary does not finish at tick 8000")
        # Three-ideal rows have explicit factors in both sources. Camera aggregate
        # rows encode regime in run_id/method suffix, while index has method_id.
        factor_row = dict(ir)
        factor_row.setdefault("flora", "medium")
        factor_row["regime"] = (ir.get("regime") or ir.get("method_id", "").removeprefix("mine_")).removeprefix("mine_")
        key = canonical(factor_row)
        if key in pairs:
            raise ValueError(f"{name}: duplicate canonical scenario key {key}")
        # Verify factors recorded on the accepted summary where present.
        for field in ("terrain", "focus", "spread", "seed", "grid"):
            if field in sr and str(sr[field]) != str(ir.get(field)):
                raise ValueError(f"{name}/{rid}: summary/index {field} mismatch")
        if "regime" in sr and str(sr["regime"]).removeprefix("mine_") != key[-1]:
            raise ValueError(f"{name}/{rid}: summary/index regime mismatch")
        sr["_seed"] = str(ir["seed"])
        sr["_grid"] = str(ir["grid"])
        sr["_regime"] = key[-1]
        sr["_canonical_key"] = "|".join(key)
        pairs[key] = sr
    return ix, pairs


def aggregate(rows_by_key: dict[tuple[str, ...], dict[str, str]], group: dict[str, str]) -> dict:
    selected = [(k, r) for k, r in rows_by_key.items()
                if all(k[FACTORS.index(field)] == value for field, value in group.items())]
    if not selected:
        raise ValueError(f"empty group {group}")
    seed_values: dict[str, list[float]] = defaultdict(list)
    for key, row in selected:
        seed_values[key[FACTORS.index("seed")]].append(num(row, "detection_fraction"))
    block_means = [statistics.fmean(v) for _, v in sorted(seed_values.items(), key=lambda x: int(x[0]))]
    equal_run = statistics.fmean(num(r, "detection_fraction") for _, r in selected)
    result = {"runs": len(selected), "equal_run_mean": equal_run,
              "equal_run_percent": 100*equal_run,
              "equal_run_seed_block_ci95": t_interval(block_means),
              "matched_seed_blocks": len(block_means)}
    return result


def paired_difference(left: dict[tuple[str, ...], dict[str, str]],
                      right: dict[tuple[str, ...], dict[str, str]],
                      group: dict[str, str]) -> dict:
    if set(left) != set(right):
        raise ValueError(f"canonical scenario sets differ: {len(left)} vs {len(right)}")
    selected = [(k, a, right[k]) for k, a in left.items()
                if all(k[FACTORS.index(field)] == value for field, value in group.items())]
    if not selected:
        raise ValueError(f"empty paired group {group}")
    seed_differences: dict[str, list[float]] = defaultdict(list)
    run_diffs = []
    for key, a, b in selected:
        diff = num(b, "detection_fraction") - num(a, "detection_fraction")
        run_diffs.append(diff)
        seed_differences[key[FACTORS.index("seed")]].append(diff)
    block_means = [statistics.fmean(v) for _, v in sorted(seed_differences.items(), key=lambda x: int(x[0]))]
    ci = t_interval(block_means)
    return {"runs": len(selected), "mean_percentage_points": 100*statistics.fmean(run_diffs),
            "seed_block_ci95_percentage_points": {k: 100*v for k, v in ci.items() if k in ("mean", "ci95_low", "ci95_high")},
            "seed_blocks": ci["seed_blocks"], "df": ci["df"], "paired_by": list(FACTORS)}


def main() -> None:
    indexes, maps = {}, {}
    for name in SOURCES:
        indexes[name], maps[name] = cohort_rows(name)
    matrix_hashes = {}
    for a, b in (("mixed", "four_camera"), ("mixed", "three_ideal"), ("four_camera", "three_ideal")):
        if set(maps[a]) != set(maps[b]):
            raise ValueError(f"{a}/{b} canonical scenario keys do not match")
        matrix_hashes[f"{a}_vs_{b}"] = len(set(maps[a]) & set(maps[b]))

    cohort_summaries = {}
    for name, values in maps.items():
        cohort_summaries[name] = {"overall": aggregate(values, {}),
                                  "by_grid": {g: aggregate(values, {"grid": g}) for g in ("100", "200", "300")},
                                  "by_regime": {r: aggregate(values, {"regime": r}) for r in ("static", "dynamic")},
                                  "by_grid_and_regime": {
                                      f"{g}_{r}": aggregate(values, {"grid": g, "regime": r})
                                      for g in ("100", "200", "300") for r in ("static", "dynamic")}}

    contrasts = {}
    contrast_pairs = (("mixed", "three_ideal", "three_ideal_minus_mixed"),
                      ("mixed", "four_camera", "four_camera_minus_mixed"),
                      ("three_ideal", "four_camera", "four_camera_minus_three_ideal"))
    for left_name, right_name, label in contrast_pairs:
        left, right = maps[left_name], maps[right_name]
        groups = {"all_grids": {"all_grids": {}},
                  "by_grid": {g: {"grid": g} for g in ("100", "200", "300")},
                  "by_regime": {r: {"regime": r} for r in ("static", "dynamic")},
                  "by_grid_and_regime": {f"{g}_{r}": {"grid": g, "regime": r}
                                         for g in ("100", "200", "300") for r in ("static", "dynamic")}}
        contrasts[label] = {bucket: {key: paired_difference(left, right, group)
                                     for key, group in groups[bucket].items()}
                            for bucket in groups}

    output = {
        "schema": "recent_camera_cell_belief_contrasts_v1",
        "scope": "Oct 9 mixed, Oct 11 four-camera and Oct 18 accepted three-ideal cohorts only; no Oct 21 main-cohort comparison",
        "metric": "cell-belief coverage; per-run detection_fraction, equal-run means unless labeled person-weighted",
        "source_summaries": {name: SOURCES[name][1] for name in SOURCES},
        "source_indexes": {name: SOURCES[name][0] for name in SOURCES},
        "index_status_and_count": {name: {"rows": len(indexes[name]), "complete": sum(r.get("status") == "complete" for r in indexes[name])}
                                    for name in SOURCES},
        "canonical_scenario_key": list(FACTORS),
        "canonical_key_matches": matrix_hashes,
        "cohort_cell_belief": cohort_summaries,
        "paired_differences": contrasts,
        "interpretation": "Differences compare matched scenario/factor tuples across distinct recorded cohort dates. They are exploratory cohort contrasts, not a randomized or build-identical estimate of a camera-only effect. The three-ideal versus mixed sensor-based witness/union analysis is separately preserved in three_ideal_analysis.json.",
        "method": "Within each fixed seed, average paired run differences across the selected factor matrix; report mean and Student-t interval over 15 seed-block means (df=14).",
    }
    (DATA / "camera_cell_belief_analysis.json").write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")

    with (DATA / "camera_cell_belief_paired_runs.csv").open("w", newline="", encoding="utf-8") as f:
        fields = ["canonical_key", "terrain", "flora", "focus", "spread", "seed", "grid", "regime",
                  "mixed_cell_belief", "four_camera_cell_belief", "three_ideal_cell_belief",
                  "three_ideal_minus_mixed_pp", "four_camera_minus_mixed_pp", "four_camera_minus_three_ideal_pp"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for key in sorted(maps["mixed"]):
            m, c, i = maps["mixed"][key], maps["four_camera"][key], maps["three_ideal"][key]
            v = {fields[j]: key[j] for j in range(len(FACTORS))}
            v.update({"canonical_key": "|".join(key), "mixed_cell_belief": num(m, "detection_fraction"),
                      "four_camera_cell_belief": num(c, "detection_fraction"),
                      "three_ideal_cell_belief": num(i, "detection_fraction"),
                      "three_ideal_minus_mixed_pp": 100*(num(i, "detection_fraction")-num(m, "detection_fraction")),
                      "four_camera_minus_mixed_pp": 100*(num(c, "detection_fraction")-num(m, "detection_fraction")),
                      "four_camera_minus_three_ideal_pp": 100*(num(c, "detection_fraction")-num(i, "detection_fraction"))})
            w.writerow(v)
    print("Wrote camera_cell_belief_analysis.json and 3,240 paired-run rows.")


if __name__ == "__main__":
    main()
