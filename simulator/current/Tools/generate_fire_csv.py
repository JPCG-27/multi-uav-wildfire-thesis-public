#!/usr/bin/env python3
"""
Generate a semi-realistic directional fire-arrival CSV for the Thesis Unreal project.

The output format matches UFireDataImporter::ImportFirstFireTicksFromCSV:
- Each CSV cell is an arrival time in minutes.
- Blank cells mean "never burns".
- Unreal converts minutes to ticks using ArrivalMinutesPerTick.

Model summary (overhauled):
- Single-point ignition: one ignition point always lights at the very start
  (--first-ignition-tick); extra ignition points (--ignition-count > 1) light
  later in a configurable window.
- Acceleration & scaling: the front starts slow and picks up speed as the
  burning region grows (radiant-heat buildup), controlled by
  --growth-acceleration and shaped by --growth-onset-fraction/--growth-full-fraction.
- Density-based velocity: spread speed is tied to fuel/vegetation density
  (--fuel-csv); the front slows significantly in sparse fuel
  (--density-spread-influence).
- Terrain impedance (barriers): steep slopes derived from the terrain
  movement-cost grid (--terrain-csv) act as natural barriers. Above
  --slope-block-steepness an edge is impassable, so the front must route
  AROUND it; within the steep band below it the front is drastically slowed
  (--slope-slow-max). No rivers/roads layer is invented — only the terrain's
  own slopes.
- No fuel consumption / no burnout: this is a fire-arrival (propagation) model
  only. --burn-percentage is a scenario-size cap, never a fuel-exhaustion rule;
  a cell that has caught fire stays alight for the importer/replay.

Example:
    python Tools/generate_fire_csv.py --width 300 --height 300 --output Fire_300x300.csv --seed 27

Example with separated ignition points:
    python Tools/generate_fire_csv.py --width 300 --height 300 --ignition-count 4 \
        --min-ignition-distance 80 --burn-percentage 20 --output Saved/GeneratedFire.csv
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import random
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

Grid = List[List[Optional[float]]]
Point = Tuple[int, int]
TerrainCostGrid = List[List[float]]
TerrainHeightGrid = Optional[List[List[float]]]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    if abs(edge1 - edge0) < 1e-9:
        return 1.0 if x >= edge1 else 0.0
    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def squared_distance(a: Point, b: Point) -> int:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def load_fuel_csv(path: Optional[Path], width: int, height: int) -> Optional[List[List[float]]]:
    """Per-cell fuel density 0..1 (the simulator's floraDensity export).
    None when no map is given — the generator then assumes uniform fuel."""
    if path is None:
        return None
    import csv as _csv
    fuel = [[0.0] * width for _ in range(height)]
    with path.open(newline="", encoding="utf-8") as f:
        for y, row in enumerate(_csv.reader(f)):
            if y >= height:
                break
            for x, v in enumerate(row):
                if x >= width:
                    break
                v = v.strip()
                if v:
                    fuel[y][x] = clamp(float(v), 0.0, 1.0)
    return fuel


def _parse_terrain_cell(raw: str) -> Tuple[float, Optional[float]]:
    """Return (movement_cost, optional_height_uu).

    Current UE export is one float per cell: movement cost. A future richer
    export can write "height:cost" or "height|cost" without changing the CLI.
    """
    value = raw.strip()
    if not value:
        return 1.0, None

    normalized = value.replace("|", ":")
    if ":" in normalized:
        pieces = [p.strip() for p in normalized.split(":") if p.strip()]
        if len(pieces) >= 2:
            return max(0.1, float(pieces[1])), float(pieces[0])

    return max(0.1, float(value)), None


def load_terrain_csv(
    path: Optional[Path],
    width: int,
    height: int,
) -> Tuple[Optional[TerrainCostGrid], TerrainHeightGrid]:
    """Per-cell terrain movement cost, plus optional height.

    The existing terrain CSV is the human movement-cost grid, which is a
    monotonic function of slope (AProceduralTerrain exports
    cost = clamp(1 + factor*slope, 1, 10)). For fire it both slows the front on
    steep cells and, above a steepness threshold, blocks it entirely so the
    front routes around. If heights are present, uphill spread gets a
    deterministic speed boost and downhill spread gets damped.
    """
    if path is None:
        return None, None

    cost: TerrainCostGrid = [[1.0] * width for _ in range(height)]
    heights: List[List[float]] = [[0.0] * width for _ in range(height)]
    saw_height = False

    with path.open(newline="", encoding="utf-8") as f:
        for y, row in enumerate(csv.reader(f)):
            if y >= height:
                break
            for x, raw in enumerate(row):
                if x >= width:
                    break
                c, h = _parse_terrain_cell(raw)
                cost[y][x] = c
                if h is not None:
                    heights[y][x] = h
                    saw_height = True

    return cost, heights if saw_height else None


def choose_ignition_points(
    width: int,
    height: int,
    ignition_count: int,
    min_ignition_distance: float,
    rng: random.Random,
    fuel: Optional[List[List[float]]] = None,
) -> Tuple[List[Point], float]:
    """
    Pick ignition points using rejection sampling.

    If the requested minimum distance is too strict for the grid/count, the
    function gradually relaxes it. That avoids hard failure in the editor while
    still giving points that are as spread out as possible.

    Returns:
        (points, effective_min_distance_used)
    """
    ignition_count = max(1, ignition_count)

    def fuel_ok(p: Point) -> bool:
        # Bias ignitions into vegetated cells: real fires start where fuel is.
        if fuel is None:
            return True
        f = fuel[p[1]][p[0]]
        return rng.random() < (0.05 + 0.95 * min(1.0, f * 1.5))

    # Keep ignitions away from the border so directional spread has room to develop.
    margin_x = max(2, width // 12)
    margin_y = max(2, height // 12)
    min_x = margin_x
    max_x = max(margin_x, width - margin_x - 1)
    min_y = margin_y
    max_y = max(margin_y, height - margin_y - 1)

    # If the grid is tiny, fall back to full extents.
    if max_x < min_x:
        min_x, max_x = 0, width - 1
    if max_y < min_y:
        min_y, max_y = 0, height - 1

    requested_distance = max(0.0, float(min_ignition_distance))
    current_distance = requested_distance

    # Try several relaxed distance levels before giving up.
    # 0 means only uniqueness is required.
    distance_levels: List[float] = []
    if current_distance > 0.0:
        while current_distance > 1.0:
            distance_levels.append(current_distance)
            current_distance *= 0.85
    distance_levels.append(0.0)

    best_points: List[Point] = []
    best_effective_distance = 0.0

    for distance in distance_levels:
        min_dist_sq = distance * distance
        points: List[Point] = []
        used = set()
        attempts_per_point = 1200
        total_attempts = max(2000, attempts_per_point * ignition_count)

        for _ in range(total_attempts):
            if len(points) >= ignition_count:
                break
            candidate = (rng.randint(min_x, max_x), rng.randint(min_y, max_y))
            if candidate in used:
                continue
            if not fuel_ok(candidate):
                continue
            if all(squared_distance(candidate, p) >= min_dist_sq for p in points):
                points.append(candidate)
                used.add(candidate)

        if len(points) > len(best_points):
            best_points = points
            best_effective_distance = distance

        if len(points) >= ignition_count:
            return points, distance

    # Last fallback: fill any missing points uniquely. This should be rare and
    # only happens when count/distance/grid size are incompatible.
    used = set(best_points)
    while len(best_points) < ignition_count and len(used) < width * height:
        candidate = (rng.randint(0, width - 1), rng.randint(0, height - 1))
        if candidate not in used:
            best_points.append(candidate)
            used.add(candidate)

    return best_points, best_effective_distance


def generate_fire_arrivals(
    width: int,
    height: int,
    seed: int,
    ignition_count: int,
    min_ignition_distance: float,
    start_min: float,
    start_max: float,
    first_ignition_minute: float,
    max_minutes: float,
    target_burn_fraction: float,
    spread_minutes_per_cell: float,
    wind_angle_degrees: Optional[float],
    wind_strength: float,
    spotting_chance: float,
    moisture: float,
    fuel: Optional[List[List[float]]] = None,
    fuel_barrier: float = 0.04,
    terrain_cost: Optional[TerrainCostGrid] = None,
    terrain_height: TerrainHeightGrid = None,
    terrain_cost_influence: float = 0.30,
    slope_spread_influence: float = 0.70,
    cell_size_uu: float = 1000.0,
    growth_acceleration: float = 3.5,
    slope_cost_ceiling: float = 10.0,
    slope_block_steepness: float = 0.75,
    slope_slow_steepness: float = 0.35,
    slope_slow_max: float = 8.0,
    slope_block_rank: float = 0.96,
    slope_slow_rank: float = 0.80,
    slope_block_grade: float = 1.5,
    density_spread_influence: float = 3.0,
    density_slow_exponent: float = 1.3,
    growth_onset_fraction: float = 0.02,
    growth_full_fraction: float = 0.6,
) -> Tuple[Grid, float, Sequence[Point], float]:
    rng = random.Random(seed)
    arrivals: Grid = [[None for _ in range(width)] for _ in range(height)]

    if wind_angle_degrees is None:
        wind_angle_degrees = rng.uniform(0.0, 360.0)

    wind_rad = math.radians(wind_angle_degrees)
    wind_x = math.cos(wind_rad)
    wind_y = math.sin(wind_rad)
    wind_strength = clamp(wind_strength, 0.0, 0.95)
    moisture = clamp(moisture, 0.0, 0.95)

    # Burn target is a scenario-size cap, not a fuel-consumption rule. Fuel
    # density changes travel time; it does not decide that a cell can never burn.
    total_cells = width * height
    target_burned = max(1, int(round(total_cells * clamp(target_burn_fraction, 0.001, 1.0))))
    target_burned = min(target_burned, total_cells)

    def fuel_at(x: int, y: int) -> float:
        return fuel[y][x] if fuel is not None else 0.8

    def terrain_cost_at(x: int, y: int) -> float:
        return terrain_cost[y][x] if terrain_cost is not None else 1.0

    # Steepness in 0..1 from the movement-cost grid. AProceduralTerrain clamps
    # its export to [1, ceiling], so normalising by (ceiling - 1) is stable and
    # independent of the (un-exported) slope->cost factor: 0 = flat, 1 = cliff.
    inv_cost_span = 1.0 / max(1e-6, (slope_cost_ceiling - 1.0))

    def steepness_at(x: int, y: int) -> float:
        return clamp((terrain_cost_at(x, y) - 1.0) * inv_cost_span, 0.0, 1.0)

    slow_max = max(1.0, slope_slow_max)

    # Relative steepness = min-rank percentile of the cost distribution, so the
    # barrier model keys off slope CONTRAST rather than the (clamp-saturated)
    # absolute movement cost. Cells tying at the same cost share the minimum
    # rank, so a map with no contrast (e.g. cost saturated at the ceiling almost
    # everywhere) yields rank ~0 -> no barrier -> the fire is never walled in.
    if terrain_cost is not None:
        import bisect as _bisect
        _sorted_costs = sorted(c for row in terrain_cost for c in row)
        _ncost = max(1, len(_sorted_costs))
        _rank_cache: dict = {}

        def steep_rank(c: float) -> float:
            r = _rank_cache.get(c)
            if r is None:
                r = _bisect.bisect_left(_sorted_costs, c) / _ncost
                _rank_cache[c] = r
            return r
    else:
        def steep_rank(c: float) -> float:
            return 0.0

    # Edge-grade thresholds from real heights (unsaturated, unlike the clamped
    # movement cost). Block the steepest ~(1 - slope_block_rank) of edges and
    # slow the steepest ~(1 - slope_slow_rank), but never below the absolute
    # cliff floor slope_block_grade. Quantiles bound the blocked fraction so even
    # uniformly-steep terrain can't wall the fire in; a discrete cliff still
    # blocks because its edges are a small, steep minority.
    if terrain_height is not None:
        _hh = len(terrain_height)
        _hw = len(terrain_height[0]) if _hh else 0
        _hrun = max(1.0, cell_size_uu)
        _diag = math.sqrt(2.0)
        _edge_grades = []
        for _y in range(_hh):
            for _x in range(_hw):
                _h0 = terrain_height[_y][_x]
                for _dx, _dy, _dd in ((1, 0, 1.0), (0, 1, 1.0), (1, 1, _diag), (-1, 1, _diag)):
                    _nx, _ny = _x + _dx, _y + _dy
                    if 0 <= _nx < _hw and 0 <= _ny < _hh:
                        _edge_grades.append(abs(terrain_height[_ny][_nx] - _h0) / (_hrun * _dd))
        _edge_grades.sort()
        _ne = max(1, len(_edge_grades))

        def _grade_quantile(frac_from_top: float) -> float:
            idx = int(round((1.0 - clamp(frac_from_top, 0.0, 1.0)) * (_ne - 1)))
            return _edge_grades[max(0, min(_ne - 1, idx))]

        _eff_block_grade = max(slope_block_grade, _grade_quantile(clamp(1.0 - slope_block_rank, 0.0, 1.0)))
        _eff_slow_grade = min(_eff_block_grade,
                              max(0.4 * slope_block_grade, _grade_quantile(clamp(1.0 - slope_slow_rank, 0.0, 1.0))))
    else:
        _eff_block_grade = 0.0
        _eff_slow_grade = 0.0

    def slope_terms(x: int, y: int, nx: int, ny: int, dist: float) -> Tuple[bool, float, float, float]:
        """Return (blocked, terrain_delay, slope_speedup, slope_slowdown).

        Two regimes:
          * Heights present (height:cost terrain export) -> physical grade model:
            |rise/run| >= slope_block_grade is a true cliff and blocks the edge;
            below that, uphill speeds the front and downhill damps it. Real grade
            is not clamp-saturated, so this is the accurate barrier signal.
          * Cost only -> relative-contrast model: the exported movement cost
            saturates at the clamp ceiling over rugged terrain, so an absolute
            threshold would wall the whole map in. A hard barrier therefore needs
            a cell to be steep in ABSOLUTE terms (>= slope_block_steepness) AND
            among the steepest on THIS map (rank >= slope_block_rank). The slow
            band is purely relative. A uniform/saturated map has no contrast ->
            no barrier -> the fire spreads freely.
        """
        cost_from = terrain_cost_at(x, y)
        cost_to = terrain_cost_at(nx, ny)
        slope_speedup = 1.0
        slope_slowdown = 1.0

        if terrain_height is not None:
            rise = terrain_height[ny][nx] - terrain_height[y][x]
            run = max(1.0, cell_size_uu * dist)
            grade = rise / run
            abs_grade = abs(grade)
            # True cliff: an edge steeper than the adaptive block grade (the
            # steeper of the absolute cliff floor and the steepest-edges
            # quantile). Least-time solving routes the front around it.
            if abs_grade >= _eff_block_grade:
                return True, 0.0, 1.0, 1.0
            influence = max(0.0, slope_spread_influence)
            cg = clamp(grade, -1.0, 1.0)
            slope_speedup += influence * 5.0 * max(0.0, cg)     # uphill: faster
            slope_slowdown += influence * 4.0 * max(0.0, -cg)   # downhill: slower
            if _eff_block_grade > _eff_slow_grade:
                band = smoothstep(_eff_slow_grade, _eff_block_grade, abs_grade)
            else:
                band = 1.0 if abs_grade >= _eff_slow_grade else 0.0
            return False, 1.0 + (slow_max - 1.0) * band, slope_speedup, slope_slowdown

        abs_steep = max(steepness_at(x, y), steepness_at(nx, ny))
        rel_steep = max(steep_rank(cost_from), steep_rank(cost_to))

        if abs_steep >= slope_block_steepness and rel_steep >= slope_block_rank:
            return True, 0.0, 1.0, 1.0

        if slope_block_rank > slope_slow_rank:
            band = smoothstep(slope_slow_rank, slope_block_rank, rel_steep)
        else:
            band = 1.0 if rel_steep >= slope_slow_rank else 0.0
        return False, 1.0 + (slow_max - 1.0) * band, slope_speedup, slope_slowdown

    # Acceleration: the front starts slow and speeds up as the burning region
    # grows (radiant-heat buildup). Progress = burned / target is proportional
    # to total fire size; the smoothstep keeps it near 1x while small and ramps
    # to (1 + growth_acceleration) as the fire becomes large.
    onset = clamp(growth_onset_fraction, 0.0, 0.99)
    full = clamp(growth_full_fraction, onset + 1e-3, 1.0)

    def growth_speedup() -> float:
        progress = burned / max(1.0, float(target_burned))
        return 1.0 + max(0.0, growth_acceleration) * smoothstep(onset, full, progress)

    ignition_points, effective_min_distance = choose_ignition_points(
        width=width,
        height=height,
        ignition_count=ignition_count,
        min_ignition_distance=min_ignition_distance,
        rng=rng,
        fuel=fuel,
    )

    frontier: List[Tuple[float, int, int]] = []
    used_ignitions = set()
    for ignition_idx, (x, y) in enumerate(ignition_points):
        if (x, y) in used_ignitions:
            continue
        used_ignitions.add((x, y))
        if ignition_idx == 0:
            # Rule 1: there is always exactly one ignition at the very start.
            t = first_ignition_minute
        else:
            t = rng.uniform(max(start_min, first_ignition_minute), max(start_max, first_ignition_minute))
        arrivals[y][x] = t
        heapq.heappush(frontier, (t, x, y))

    # 8-neighbour spread. Directional wind makes downwind moves faster and more likely.
    neighbours = [
        (-1, -1, math.sqrt(2.0)), (0, -1, 1.0), (1, -1, math.sqrt(2.0)),
        (-1,  0, 1.0),                         (1,  0, 1.0),
        (-1,  1, math.sqrt(2.0)), (0,  1, 1.0), (1,  1, math.sqrt(2.0)),
    ]

    burned = len(used_ignitions)
    burned_cells = list(used_ignitions)

    # If random spread stalls before the requested burn percentage, restart
    # spread attempts from already-burning cells. This does not create extra
    # ignition points; it just prevents low-probability early extinction from
    # missing the requested total burn percentage by a large amount. Barriers
    # are still respected on every edge, so this never tunnels through a cliff.
    stall_restarts = 0
    max_stall_restarts = max(50, ignition_count * 6)

    while burned < target_burned:
        if not frontier:
            if not burned_cells or stall_restarts >= max_stall_restarts:
                break
            rx, ry = rng.choice(burned_cells)
            rt = arrivals[ry][rx]
            if rt is not None and rt <= max_minutes:
                heapq.heappush(frontier, (rt, rx, ry))
            stall_restarts += 1
            continue

        current_t, x, y = heapq.heappop(frontier)
        if arrivals[y][x] is None or current_t > arrivals[y][x] + 1e-6:
            continue
        if current_t > max_minutes:
            continue

        for dx, dy, dist in neighbours:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue

            # Terrain barrier first: a steep edge is impassable, so the front
            # cannot cross here and is forced to find a way around.
            blocked, terrain_delay, slope_speedup, slope_slowdown = slope_terms(x, y, nx, ny, dist)
            if blocked:
                continue

            dir_len = math.hypot(dx, dy)
            dir_x = dx / dir_len
            dir_y = dy / dir_len
            alignment = dir_x * wind_x + dir_y * wind_y  # -1 upwind, +1 downwind

            # Sparse fuel slows the front; dense canopy carries it faster. Low
            # density is not a hard stop because barriers are slope-based here;
            # density only changes travel time (rule 3).
            f = fuel_at(nx, ny)

            # Elliptical/time-biased spread: faster downwind, slower upwind,
            # faster through dense fuel; stronger noise -> fingered perimeters.
            speedup = 1.0 + wind_strength * max(alignment, 0.0) * 1.75
            slowdown = 1.0 + wind_strength * max(-alignment, 0.0) * 2.75
            fuel_slow = 1.0 + max(0.0, density_spread_influence) * (
                (1.0 - clamp(f, 0.0, 1.0)) ** max(0.1, density_slow_exponent))
            moisture_slow = 1.0 + 1.65 * moisture
            noise = rng.lognormvariate(0.0, 0.30)
            delay = (
                spread_minutes_per_cell
                * dist
                * slowdown
                * fuel_slow
                * moisture_slow
                * terrain_delay
                * slope_slowdown
                * noise
                / max(speedup * slope_speedup * growth_speedup(), 0.1)
            )
            nt = current_t + delay

            if nt > max_minutes:
                continue

            old = arrivals[ny][nx]
            if old is None or nt < old:
                if old is None:
                    if burned >= target_burned:
                        continue
                    burned += 1
                    burned_cells.append((nx, ny))
                arrivals[ny][nx] = nt
                heapq.heappush(frontier, (nt, nx, ny))

        # Occasional ember spotting, mostly downwind and not too far away.
        # Embers cannot land on a steep barrier cell or on near-bare ground.
        if spotting_chance > 0.0 and rng.random() < spotting_chance:
            distance = rng.randint(3, 12)
            lateral = rng.randint(-3, 3)
            sx = int(round(x + wind_x * distance - wind_y * lateral))
            sy = int(round(y + wind_y * distance + wind_x * lateral))
            if (0 <= sx < width and 0 <= sy < height
                    and fuel_at(sx, sy) >= fuel_barrier
                    and steepness_at(sx, sy) < slope_block_steepness):
                blocked, terrain_delay, slope_speedup, slope_slowdown = slope_terms(
                    x, y, sx, sy, max(1.0, float(distance)))
                if not blocked:
                    nt = current_t + (
                        rng.uniform(spread_minutes_per_cell * 2.0, spread_minutes_per_cell * 6.0)
                        * terrain_delay
                        * slope_slowdown
                        / max(slope_speedup * growth_speedup(), 0.1)
                    )
                    if nt <= max_minutes:
                        old = arrivals[sy][sx]
                        if old is None or nt < old:
                            if old is None:
                                if burned >= target_burned:
                                    continue
                                burned += 1
                                burned_cells.append((sx, sy))
                            arrivals[sy][sx] = nt
                            heapq.heappush(frontier, (nt, sx, sy))

    return arrivals, wind_angle_degrees, ignition_points, effective_min_distance


def remap_arrivals_to_duration(
    arrivals: Grid,
    target_duration_ticks: int,
    arrival_minutes_per_tick: float,
    fill_fraction: float,
    remap_strength: float,
    fast_start_fraction: float,
) -> Optional[Tuple[float, float, float, float, float]]:
    """Stretch/compress arrival times so spread activity lasts through a run.

    Returns (old_last_minute, new_last_minute, target_last_minute,
    fast_start_fraction, fast_start_until_minute), or None when there is
    nothing useful to remap.
    """
    if target_duration_ticks <= 0:
        return None

    minutes_per_tick = max(0.000001, arrival_minutes_per_tick)
    target_last = max(0.000001, target_duration_ticks * minutes_per_tick * clamp(fill_fraction, 0.01, 1.0))

    finite: List[Tuple[float, int, int]] = []
    for y, row in enumerate(arrivals):
        for x, t in enumerate(row):
            if t is not None:
                finite.append((t, x, y))

    if len(finite) <= 1:
        return None

    finite.sort(key=lambda item: (item[0], item[2], item[1]))
    old_first = finite[0][0]
    old_last = finite[-1][0]
    target_last = max(target_last, old_first + minutes_per_tick)
    old_span = max(0.000001, old_last - old_first)
    new_span = max(0.000001, target_last - old_first)
    strength = clamp(remap_strength, 0.0, 1.0)
    fast_fraction = clamp(fast_start_fraction, 0.0, 0.75)

    if target_last <= old_last:
        new_last = old_first
        for old_t, x, y in finite:
            physical_progress = clamp((old_t - old_first) / old_span, 0.0, 1.0)
            new_t = old_first + physical_progress * new_span
            arrivals[y][x] = new_t
            new_last = new_t
        return old_last, new_last, target_last, 0.0, old_first

    anchor_rank = min(len(finite) - 1, max(0, int(round((len(finite) - 1) * fast_fraction))))
    anchor_old_t = finite[anchor_rank][0]
    tail_rank_denominator = max(1, (len(finite) - 1) - anchor_rank)
    tail_old_span = max(0.000001, old_last - anchor_old_t)
    tail_new_span = max(minutes_per_tick, target_last - anchor_old_t)

    new_last = old_first
    for rank, (old_t, x, y) in enumerate(finite):
        if rank <= anchor_rank:
            # Keep the early fire front on physical-model timing. The long-run
            # duration stretch should not make the first visible expansion wait.
            new_t = old_t
        else:
            tail_physical = clamp((old_t - anchor_old_t) / tail_old_span, 0.0, 1.0)
            tail_rank = (rank - anchor_rank) / tail_rank_denominator
            progress = (1.0 - strength) * tail_physical + strength * tail_rank
            new_t = anchor_old_t + progress * tail_new_span
        arrivals[y][x] = new_t
        new_last = new_t

    return old_last, new_last, target_last, fast_fraction, anchor_old_t


def write_csv(arrivals: Grid, output: Path, precision: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for row in arrivals:
            writer.writerow(["" if v is None else f"{v:.{precision}f}" for v in row])


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate directional semi-realistic fire-arrival CSVs (slope-barrier aware).")
    parser.add_argument("--width", type=int, default=300)
    parser.add_argument("--height", type=int, default=300)
    parser.add_argument("--output", type=Path, default=Path("GeneratedFire_300x300.csv"))
    parser.add_argument("--seed", type=int, default=27)
    parser.add_argument("--ignition-count", type=int, default=1)
    parser.add_argument(
        "--num-focus-points",
        type=int,
        default=None,
        help="Benchmark alias for --ignition-count: number of independent fire focus points.",
    )
    parser.add_argument(
        "--spread-variant",
        type=int,
        choices=(0, 1, 2),
        default=0,
        help="Benchmark spread variant. Uses the same base seed but a stable variant seed offset.",
    )
    parser.add_argument(
        "--min-ignition-distance",
        type=float,
        default=0.0,
        help=(
            "Minimum distance in grid cells between initial ignition points. "
            "Use 0 to disable. If too strict, the generator gradually relaxes it."
        ),
    )
    parser.add_argument("--start-min", type=float, default=2.0, help="Earliest ignition arrival minute. Must be > 0.")
    parser.add_argument("--start-max", type=float, default=9.0, help="Latest ignition arrival minute. Must be >= start-min.")
    parser.add_argument("--first-ignition-tick", type=int, default=1,
                        help="First ignition starts on this simulation tick.")
    parser.add_argument("--additional-ignition-start-min-tick", type=int, default=250,
                        help="Earliest start tick for ignition points after the first.")
    parser.add_argument("--additional-ignition-start-max-tick", type=int, default=1500,
                        help="Latest start tick for ignition points after the first.")
    parser.add_argument("--max-minutes", type=float, default=0.0,
                        help="Time horizon in minutes. <= 0 = auto-scale to grid size.")
    burn_group = parser.add_mutually_exclusive_group()
    burn_group.add_argument(
        "--burn-percentage",
        type=float,
        default=None,
        help="Target percentage of cells that should burn, e.g. 14 for 14%%.",
    )
    burn_group.add_argument(
        "--target-burn-fraction",
        type=float,
        default=None,
        help="Target fraction of cells that should burn, e.g. 0.14 for 14%%.",
    )
    burn_group.add_argument(
        "--max-burn-fraction",
        type=float,
        default=None,
        help="Deprecated alias for --target-burn-fraction.",
    )
    parser.add_argument("--spread-minutes-per-cell", type=float, default=40.0)
    parser.add_argument("--spread-ticks-per-cell", type=float, default=15.0,
                        help="Preferred sim-time spread rate. Overrides --spread-minutes-per-cell when set.")
    parser.add_argument(
        "--wind-angle-degrees",
        type=float,
        default=None,
        help="0=east, 90=south in CSV row coordinates. Omit for random.",
    )
    parser.add_argument("--wind-strength", type=float, default=0.65)
    parser.add_argument("--spotting-chance", type=float, default=0.0)
    parser.add_argument("--moisture", type=float, default=0.18)
    parser.add_argument("--precision", type=int, default=2)
    parser.add_argument(
        "--fuel-csv",
        type=Path,
        default=None,
        help="Per-cell fuel density 0..1 (simulator floraDensity export). "
             "Fire spreads fast in dense fuel, stalls in clearings.")
    parser.add_argument("--fuel-barrier", type=float, default=0.04,
                        help="Fuel below this is a near-barrier (ember-jump only).")
    parser.add_argument(
        "--terrain-csv",
        type=Path,
        default=None,
        help="Optional terrain CSV. Plain cells are movement costs; height:cost cells also enable uphill/downhill spread.")
    parser.add_argument("--terrain-cost-influence", type=float, default=0.30,
                        help="How strongly terrain movement cost slows spread on steep/difficult cells.")
    parser.add_argument("--slope-spread-influence", type=float, default=0.70,
                        help="When terrain heights are present, uphill boost/downhill damping strength.")
    parser.add_argument("--growth-acceleration", type=float, default=2.5,
                        help="How much the front speeds up as the burning region grows.")
    parser.add_argument("--cell-size-uu", type=float, default=1000.0,
                        help="Grid cell size in Unreal units, used only with terrain heights.")
    # ---- Slope-as-barrier controls (terrain impedance, rule 4) ----
    parser.add_argument("--slope-cost-ceiling", type=float, default=10.0,
                        help="Movement-cost value treated as a vertical cliff (matches AProceduralTerrain's "
                             "export clamp of 10). Cost maps to steepness 0..1 across [1, ceiling].")
    parser.add_argument("--slope-block-steepness", type=float, default=0.75,
                        help="Edges at/above this normalized steepness (0=flat, 1=cliff) are impassable; the "
                             "front must route around them. Set >= 1.0 to disable hard blocking (slowdown only).")
    parser.add_argument("--slope-slow-steepness", type=float, default=0.35,
                        help="Steepness at which the steep-band slowdown begins.")
    parser.add_argument("--slope-slow-max", type=float, default=8.0,
                        help="Maximum extra delay multiplier reached just below the block threshold "
                             "(drastically reduced speed around barriers).")
    parser.add_argument("--slope-block-rank", type=float, default=0.96,
                        help="Relative-steepness percentile (0..1) a cell must also reach (on top of "
                             "--slope-block-steepness) to be a hard barrier in cost-only mode. Bounds "
                             "blocked area to ~(1-rank) of the map so saturated terrain can't wall the "
                             "fire in.")
    parser.add_argument("--slope-slow-rank", type=float, default=0.80,
                        help="Relative-steepness percentile where the cost-only slow band begins.")
    parser.add_argument("--slope-block-grade", type=float, default=1.5,
                        help="With real terrain heights, |rise/run| at/above this is a true cliff that "
                             "blocks the front (1.5 ~= 56 degrees).")
    # ---- Density-based velocity (rule 3) ----
    parser.add_argument("--density-spread-influence", type=float, default=3.0,
                        help="How strongly low fuel density slows the front. 0 = density-independent speed.")
    parser.add_argument("--density-slow-exponent", type=float, default=1.3,
                        help="Shape of the density slowdown curve (higher = only very sparse fuel slows).")
    # ---- Acceleration shaping (rule 2) ----
    parser.add_argument("--growth-onset-fraction", type=float, default=0.02,
                        help="Burned fraction of the target at which acceleration begins (fire starts slow).")
    parser.add_argument("--growth-full-fraction", type=float, default=0.6,
                        help="Burned fraction of the target at which acceleration is fully ramped in.")
    parser.add_argument("--target-duration-ticks", type=int, default=0,
                        help="If >0, remap arrivals so spread activity lasts through this many sim ticks.")
    parser.add_argument("--arrival-minutes-per-tick", type=float, default=1.0,
                        help="Minutes represented by one sim tick, used with --target-duration-ticks.")
    parser.add_argument("--duration-fill-fraction", type=float, default=0.95,
                        help="Latest generated ignition lands at this fraction of target-duration-ticks.")
    parser.add_argument("--duration-remap-strength", type=float, default=0.0,
                        help="Opt-in post-process remap. 0 disables it; use >0 only for synthetic pacing tests.")
    parser.add_argument("--duration-fast-start-fraction", type=float, default=0.12,
                        help="Fraction of burning cells kept on physical timing before stretching the tail.")
    args = parser.parse_args()

    if args.width <= 0 or args.height <= 0:
        raise SystemExit("width and height must be positive")
    if args.start_min <= 0 or args.start_max <= 0 or args.start_max < args.start_min:
        raise SystemExit("start-min/start-max must be positive and ordered; importer treats <=0 as never burns")
    if args.num_focus_points is not None:
        args.ignition_count = args.num_focus_points

    if args.ignition_count <= 0:
        raise SystemExit("ignition-count must be positive")
    if args.min_ignition_distance < 0:
        raise SystemExit("min-ignition-distance must be >= 0")
    if args.arrival_minutes_per_tick <= 0:
        raise SystemExit("arrival-minutes-per-tick must be positive")
    if args.cell_size_uu <= 0:
        raise SystemExit("cell-size-uu must be positive")
    if args.first_ignition_tick < 1:
        raise SystemExit("first-ignition-tick must be >= 1")
    if args.additional_ignition_start_min_tick < args.first_ignition_tick:
        raise SystemExit("additional-ignition-start-min-tick must be >= first-ignition-tick")
    if args.additional_ignition_start_max_tick < args.additional_ignition_start_min_tick:
        raise SystemExit("additional ignition start ticks must be ordered")
    if args.slope_cost_ceiling <= 1.0:
        raise SystemExit("slope-cost-ceiling must be > 1.0")
    if args.slope_slow_max < 1.0:
        raise SystemExit("slope-slow-max must be >= 1.0")
    if args.slope_block_steepness <= 0.0:
        raise SystemExit("slope-block-steepness must be > 0.0 (use >= 1.0 to disable hard blocking)")

    first_ignition_minute = args.first_ignition_tick * args.arrival_minutes_per_tick
    args.start_min = args.additional_ignition_start_min_tick * args.arrival_minutes_per_tick
    args.start_max = args.additional_ignition_start_max_tick * args.arrival_minutes_per_tick

    target_duration_minutes = 0.0
    if args.target_duration_ticks > 0:
        target_duration_minutes = (
            args.target_duration_ticks
            * args.arrival_minutes_per_tick
            * clamp(args.duration_fill_fraction, 0.01, 1.0)
        )

    if args.max_minutes <= 0:
        if target_duration_minutes > 0.0:
            args.max_minutes = max(args.start_max, target_duration_minutes)
            print(f"Auto max-minutes: {args.max_minutes:.1f} (scaled to target duration)")
        else:
            # Auto horizon: enough minutes for a fire to cross the longer grid axis
            # at the nominal spread rate, with 50% headroom for upwind/fuel slowdown.
            args.max_minutes = args.start_max + args.spread_minutes_per_cell * 1.5 * max(args.width, args.height)
            print(f"Auto max-minutes: {args.max_minutes:.1f} (scaled to grid size)")
    if args.spread_ticks_per_cell is not None:
        if args.spread_ticks_per_cell <= 0:
            raise SystemExit("spread-ticks-per-cell must be positive")
        args.spread_minutes_per_cell = args.spread_ticks_per_cell * args.arrival_minutes_per_tick
    if args.spread_minutes_per_cell <= 0:
        raise SystemExit("spread-minutes-per-cell must be positive")

    if args.burn_percentage is not None:
        target_burn_fraction = args.burn_percentage / 100.0
    elif args.target_burn_fraction is not None:
        target_burn_fraction = args.target_burn_fraction
    elif args.max_burn_fraction is not None:
        target_burn_fraction = args.max_burn_fraction
    else:
        target_burn_fraction = 0.16

    if not (0.0 < target_burn_fraction <= 1.0):
        raise SystemExit("burn percentage/fraction must be greater than 0 and at most 100% / 1.0")

    fuel = load_fuel_csv(args.fuel_csv, args.width, args.height)
    terrain_cost, terrain_height = load_terrain_csv(args.terrain_csv, args.width, args.height)

    effective_seed = args.seed + args.spread_variant * 104729

    arrivals, wind, ignition_points, effective_min_distance = generate_fire_arrivals(
        width=args.width,
        height=args.height,
        seed=effective_seed,
        ignition_count=args.ignition_count,
        min_ignition_distance=args.min_ignition_distance,
        start_min=args.start_min,
        start_max=args.start_max,
        first_ignition_minute=first_ignition_minute,
        max_minutes=args.max_minutes,
        target_burn_fraction=target_burn_fraction,
        fuel=fuel,
        fuel_barrier=args.fuel_barrier,
        spread_minutes_per_cell=args.spread_minutes_per_cell,
        wind_angle_degrees=args.wind_angle_degrees,
        wind_strength=args.wind_strength,
        spotting_chance=args.spotting_chance,
        moisture=args.moisture,
        terrain_cost=terrain_cost,
        terrain_height=terrain_height,
        terrain_cost_influence=args.terrain_cost_influence,
        slope_spread_influence=args.slope_spread_influence,
        cell_size_uu=args.cell_size_uu,
        growth_acceleration=args.growth_acceleration,
        slope_cost_ceiling=args.slope_cost_ceiling,
        slope_block_steepness=args.slope_block_steepness,
        slope_slow_steepness=args.slope_slow_steepness,
        slope_slow_max=args.slope_slow_max,
        slope_block_rank=args.slope_block_rank,
        slope_slow_rank=args.slope_slow_rank,
        slope_block_grade=args.slope_block_grade,
        density_spread_influence=args.density_spread_influence,
        density_slow_exponent=args.density_slow_exponent,
        growth_onset_fraction=args.growth_onset_fraction,
        growth_full_fraction=args.growth_full_fraction,
    )
    duration_remap = None
    if args.duration_remap_strength > 0.0:
        duration_remap = remap_arrivals_to_duration(
            arrivals,
            args.target_duration_ticks,
            args.arrival_minutes_per_tick,
            args.duration_fill_fraction,
            args.duration_remap_strength,
            args.duration_fast_start_fraction,
        )
    write_csv(arrivals, args.output, args.precision)

    burned = sum(1 for row in arrivals for v in row if v is not None)
    total_cells = args.width * args.height
    requested = target_burn_fraction
    actual = burned / total_cells

    print(f"Wrote {args.output}")
    print(f"Grid: {args.width} x {args.height}")
    print(f"Base seed: {args.seed}, spread variant: {args.spread_variant}, effective seed: {effective_seed}")
    print(f"Ignition count: {len(ignition_points)}")
    print(f"Ignition points: {', '.join(f'({x},{y})' for x, y in ignition_points)}")
    print(f"First ignition tick: {args.first_ignition_tick}")
    if len(ignition_points) > 1:
        print(
            "Additional ignition tick window: "
            f"{args.additional_ignition_start_min_tick}..{args.additional_ignition_start_max_tick}"
        )
    if args.spread_ticks_per_cell is not None:
        print(f"Nominal spread: {args.spread_ticks_per_cell:.2f} ticks/cell")
    if args.min_ignition_distance > 0:
        print(f"Requested minimum ignition distance: {args.min_ignition_distance:.1f} cells")
        print(f"Effective minimum ignition distance: {effective_min_distance:.1f} cells")
        if effective_min_distance + 1e-6 < args.min_ignition_distance:
            print("Note: requested ignition distance was too strict, so it was relaxed.")
    print(f"Requested burn fraction (of grid cells): {requested:.1%}")
    print(f"Achieved burn fraction: {burned} / {total_cells} grid cells ({actual:.1%})")
    if abs(actual - requested) > 0.005:
        print("Note: actual percentage differs because the local fire front hit max-minutes, "
              "map borders, or slope barriers.")
    if args.terrain_csv is not None:
        mode = "height+cost" if terrain_height is not None else "cost"
        print(f"Terrain-aware spread: {mode} from {args.terrain_csv}")
        if terrain_height is not None:
            print(
                f"Slope barriers: grade-based, cliff at |rise/run| >= {args.slope_block_grade:.2f} "
                "(uphill speeds / downhill damps the front)"
            )
        else:
            print(
                "Slope barriers: relative-contrast (movement cost saturates, so absolute thresholds "
                f"are unsafe). Hard barrier needs steepness >= {args.slope_block_steepness:.2f} AND top "
                f"{max(0.0, 1.0 - args.slope_block_rank):.0%} steepest; slow band from the top "
                f"{max(0.0, 1.0 - args.slope_slow_rank):.0%}."
            )
    if duration_remap is not None:
        old_last, new_last, target_last, fast_fraction, fast_until = duration_remap
        print(
            "Duration remap: last arrival "
            f"{old_last:.2f} -> {new_last:.2f} minutes "
            f"(target {target_last:.2f}, {args.target_duration_ticks} ticks)"
        )
        if fast_fraction > 0.0:
            print(
                "Fast-start remap: first "
                f"{fast_fraction:.1%} of burning cells kept on physical timing "
                f"(through {fast_until:.2f} minutes)"
            )
    print(f"Wind angle: {wind:.1f} degrees, wind strength: {args.wind_strength:.2f}")


if __name__ == "__main__":
    main()
