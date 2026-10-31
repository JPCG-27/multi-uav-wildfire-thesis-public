#!/usr/bin/env python3
"""Generate deterministic, fire- and terrain-aware human victim trajectories.

Consumes the same fire-arrival CSV the simulator replays (Cell2Fire or
generate_fire_csv.py output) plus an optional terrain-cost CSV exported from
UE, and produces per-tick person keyframes for in-engine replay.

The output is a pure function of the inputs + seed: same inputs, same bytes.

Behaviour model (PLAN.md B.2, revised):
  - React by PROJECTED ARRIVAL TIME: a person starts fleeing when the fire is
    forecast to reach their surroundings within --react-horizon-ticks (default
    150), not when it is within a fixed radius.
  - Flee = gradient ASCENT on the fire-arrival field: move toward cells that
    burn later (or never), which points away from the approaching front along
    the actual spread geometry instead of simple radial repulsion.
  - Refuge-seeking: a precomputed terrain+danger-cost Dijkstra distance field
    to the nearest LARGE never-burning region; fleeing persons descend it, so
    they route around fire tongues instead of being pushed into dead ends.
  - Groups follow a LEADER (lowest alive mobile pid) instead of the centroid;
    group speed = slowest mobile member; immobile members are abandoned
    (split event).
  - Panic: fleeing speed x --panic-speed-mult (default 1.4); a seeded fraction
    of persons (--wrong-way-fraction) initially runs the WRONG WAY for
    --wrong-way-ticks before correcting.
  - Safety: nobody steps into a cell whose fire arrival is within
    --no-step-horizon-ticks (default 10) of the current tick.
  - Capability: exposure near fire accumulates; healthy -> injured -> immobile
    -> dead, with per-state speed.

Outputs (written next to --output):
  humans.csv        tick,person,x,y,group,state   (x,y in cell space, floats)
  humans_events.csv tick,event,person,detail      (appear/state_change/merge/split)

Example:
  python Tools/generate_humans.py --fire-csv Saved/GeneratedFire.csv \
      --width 30 --height 30 --num-persons 12 --max-ticks 2000 --seed 27 \
      --output Saved/GeneratedHumans.csv
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import random
from pathlib import Path

# Person states
HEALTHY, INJURED, IMMOBILE, DEAD = 0, 1, 2, 3
STATE_SPEED = {HEALTHY: 0.14, INJURED: 0.06, IMMOBILE: 0.0, DEAD: 0.0}
STATE_NAMES = {HEALTHY: "healthy", INJURED: "injured", IMMOBILE: "immobile", DEAD: "dead"}

BIG = 10 ** 9

# 8-neighbour direction table, fixed order — every per-tick scan uses it, so
# tie-breaks are deterministic by construction.
DIRS8 = [(-1, -1), (0, -1), (1, -1),
         (-1, 0),           (1, 0),
         (-1, 1),  (0, 1),  (1, 1)]


def det_noise(seed: int, person: int, tick: int) -> float:
    """Deterministic per-(person, tick) noise in [0, 1) — order-independent."""
    h = (seed * 1_000_003 + person * 9_176_2241 + tick * 2_654_435_761) & 0xFFFFFFFF
    h ^= h >> 16
    h = (h * 0x45D9F3B) & 0xFFFFFFFF
    h ^= h >> 16
    return h / 0x100000000


def load_fire_csv(path: Path, width: int, height: int, minutes_per_tick: float):
    """Returns arrival_tick grid (row-major y*W+x), -1 = never burns."""
    arrival = [-1] * (width * height)
    with path.open(newline="", encoding="utf-8") as f:
        for y, row in enumerate(csv.reader(f)):
            if y >= height:
                break
            for x, v in enumerate(row):
                if x >= width:
                    break
                v = v.strip()
                if v:
                    arrival[y * width + x] = max(0, int(round(float(v) / minutes_per_tick)))
    return arrival


def load_terrain_csv(path: Path | None, width: int, height: int):
    """Movement-cost multiplier per cell (>= 1 slows). Flat 1.0 if absent."""
    cost = [1.0] * (width * height)
    if path is None:
        return cost
    with path.open(newline="", encoding="utf-8") as f:
        for y, row in enumerate(csv.reader(f)):
            if y >= height:
                break
            for x, v in enumerate(row):
                if x >= width:
                    break
                v = v.strip()
                if v:
                    # Accept "cost" or "height:cost"/"height|cost"; the terrain
                    # export now carries height for the fire generator's slope
                    # barriers, and the cost is the value after the colon.
                    token = v.replace("|", ":")
                    if ":" in token:
                        parts = [w for w in token.split(":") if w.strip()]
                        token = parts[-1] if parts else token
                    cost[y * width + x] = max(0.1, float(token))
    return cost


def build_refuge_field(arrival, terrain, width: int, height: int, min_size: int):
    """Cost-distance to the nearest LARGE never-burning region.

    1) 4-connected components over never-burning cells; components smaller than
       min_size are ignored (a 2-cell gap in the fire is not a refuge).
    2) Multi-source Dijkstra from all refuge cells. Step cost to ENTER a cell =
       terrain_cost * (1 + 2 if the cell ever burns) * euclidean step length —
       the terrain+danger routing cost.

    Deterministic: raster-order labelling, heap ties broken by cell index.
    Returns a flat list of floats (BIG where unreachable / no refuge exists).
    """
    n = width * height
    never = [arrival[i] < 0 for i in range(n)]

    # ---- connected components (4-conn), raster order ----
    comp = [-1] * n
    sizes = []
    for start in range(n):
        if not never[start] or comp[start] >= 0:
            continue
        label = len(sizes)
        stack = [start]
        comp[start] = label
        count = 0
        while stack:
            c = stack.pop()
            count += 1
            cx, cy = c % width, c // width
            for dx, dy in ((0, -1), (-1, 0), (1, 0), (0, 1)):
                nx_, ny_ = cx + dx, cy + dy
                if 0 <= nx_ < width and 0 <= ny_ < height:
                    nc = ny_ * width + nx_
                    if never[nc] and comp[nc] < 0:
                        comp[nc] = label
                        stack.append(nc)
        sizes.append(count)

    refuge_ok = [s >= min_size for s in sizes]

    # ---- multi-source Dijkstra ----
    dist = [float(BIG)] * n
    heap = []
    for c in range(n):                      # raster order: deterministic seeding
        if comp[c] >= 0 and refuge_ok[comp[c]]:
            dist[c] = 0.0
            heap.append((0.0, c))
    heapq.heapify(heap)

    while heap:
        d, c = heapq.heappop(heap)
        if d > dist[c] + 1e-12:
            continue
        cx, cy = c % width, c // width
        for dx, dy in DIRS8:
            nx_, ny_ = cx + dx, cy + dy
            if not (0 <= nx_ < width and 0 <= ny_ < height):
                continue
            nc = ny_ * width + nx_
            step_len = math.sqrt(2.0) if (dx and dy) else 1.0
            danger = 2.0 if arrival[nc] >= 0 else 0.0
            nd = d + step_len * terrain[nc] * (1.0 + danger)
            if nd < dist[nc] - 1e-12:
                dist[nc] = nd
                heapq.heappush(heap, (nd, nc))

    return dist


class Person:
    __slots__ = ("pid", "x", "y", "group", "state", "exposure", "appear_tick",
                 "alert_tick", "wrong_way")

    def __init__(self, pid, x, y, group, appear_tick, wrong_way):
        self.pid = pid
        self.x = x
        self.y = y
        self.group = group
        self.state = HEALTHY
        self.exposure = 0.0
        self.appear_tick = appear_tick
        self.alert_tick = -1          # first tick this person became alerted
        self.wrong_way = wrong_way    # seeded panic: initially flees the wrong way


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fire-csv", type=Path, required=True)
    p.add_argument("--terrain-csv", type=Path, default=None,
                   help="Optional per-cell movement-cost CSV from UE terrain export (flat if absent)")
    p.add_argument("--width", type=int, required=True)
    p.add_argument("--height", type=int, required=True)
    p.add_argument("--arrival-minutes-per-tick", type=float, default=1.0,
                   help="Must match ASimulationManager::ArrivalMinutesPerTick")
    p.add_argument("--burn-duration-ticks", type=int, default=60,
                   help="Must match ASimulationManager::BurnDurationTicks")
    p.add_argument("--max-ticks", type=int, default=2000)
    p.add_argument("--seed", type=int, default=27)
    p.add_argument("--num-persons", type=int, default=12)
    p.add_argument("--group-size-max", type=int, default=4)
    p.add_argument("--appear-spread-ticks", type=int, default=0,
                   help="Persons appear uniformly in [0, this] instead of all at tick 0")
    p.add_argument("--danger-radius-cells", type=float, default=6.0,
                   help="DEPRECATED (kept for CLI compat): alerting now uses --react-horizon-ticks")
    p.add_argument("--react-horizon-ticks", type=int, default=150,
                   help="Start fleeing when fire is projected to arrive nearby within this many ticks")
    p.add_argument("--no-step-horizon-ticks", type=int, default=10,
                   help="Never step into a cell whose fire arrival is within this many ticks")
    p.add_argument("--panic-speed-mult", type=float, default=1.4,
                   help="Speed multiplier while fleeing")
    p.add_argument("--wrong-way-fraction", type=float, default=0.2,
                   help="Seeded fraction of persons that initially flee the wrong way")
    p.add_argument("--wrong-way-ticks", type=int, default=40,
                   help="How long wrong-way starters run the wrong way before correcting")
    p.add_argument("--refuge-min-size", type=int, default=25,
                   help="Minimum never-burning region size (cells) that counts as a refuge")
    p.add_argument("--refuge-weight", type=float, default=1.0,
                   help="Weight of the refuge-descent direction while fleeing")
    p.add_argument("--grad-weight", type=float, default=1.0,
                   help="Weight of the arrival-gradient-ascent direction while fleeing")
    p.add_argument("--exposure-radius-cells", type=float, default=1.5)
    p.add_argument("--exposure-injure", type=float, default=8.0)
    p.add_argument("--exposure-immobile", type=float, default=20.0)
    p.add_argument("--exposure-dead", type=float, default=35.0)
    p.add_argument("--group-merge-radius-cells", type=float, default=2.0)
    p.add_argument("--cohesion-weight", type=float, default=0.35,
                   help="Leader-following steering weight for non-leader group members")
    p.add_argument("--wander-weight", type=float, default=0.25)
    p.add_argument("--output", type=Path, default=Path("GeneratedHumans.csv"))
    args = p.parse_args()

    W, H = args.width, args.height
    rng = random.Random(args.seed)
    arrival = load_fire_csv(args.fire_csv, W, H, args.arrival_minutes_per_tick)
    terrain = load_terrain_csv(args.terrain_csv, W, H)
    refuge_dist = build_refuge_field(arrival, terrain, W, H, args.refuge_min_size)

    def fire_active(cx: int, cy: int, tick: int) -> bool:
        if not (0 <= cx < W and 0 <= cy < H):
            return False
        a = arrival[cy * W + cx]
        return a >= 0 and a <= tick < a + args.burn_duration_ticks

    def cell_unsafe_to_enter(cx: int, cy: int, tick: int) -> bool:
        """Burning now, or igniting within the no-step horizon (and not burnt out)."""
        if not (0 <= cx < W and 0 <= cy < H):
            return False
        a = arrival[cy * W + cx]
        return a >= 0 and a < tick + args.no_step_horizon_ticks and tick < a + args.burn_duration_ticks

    def arrival_capped(cx: int, cy: int, tick: int, cap: float):
        """Ticks-until-burn, capped; cap (= 'safe') for never-burning or burnt-out
        cells; None for out-of-bounds."""
        if not (0 <= cx < W and 0 <= cy < H):
            return None
        a = arrival[cy * W + cx]
        if a < 0 or a + args.burn_duration_ticks <= tick:
            return cap
        return max(0.0, min(cap, float(a - tick)))

    # ---------------- Spawn (deterministic from seed) ----------------
    persons: list[Person] = []
    next_group = 0
    pid = 0
    while pid < args.num_persons:
        size = min(rng.randint(1, args.group_size_max), args.num_persons - pid)
        # Group anchor: a cell that does not burn in the first 200 ticks.
        for _ in range(10_000):
            gx, gy = rng.randrange(W), rng.randrange(H)
            a = arrival[gy * W + gx]
            if a < 0 or a > 200:
                break
        appear = rng.randint(0, args.appear_spread_ticks) if args.appear_spread_ticks > 0 else 0
        for _ in range(size):
            px = min(W - 0.001, max(0.0, gx + 0.5 + rng.uniform(-1.0, 1.0)))
            py = min(H - 0.001, max(0.0, gy + 0.5 + rng.uniform(-1.0, 1.0)))
            wrong_way = det_noise(args.seed, pid, 999_999) < args.wrong_way_fraction
            persons.append(Person(pid, px, py, next_group, appear, wrong_way))
            pid += 1
        next_group += 1

    # ---------------- Simulate ----------------
    out_rows = []
    events = []
    for pe in persons:
        events.append((pe.appear_tick, "appear", pe.pid, f"group={pe.group}"))

    GRAD_CAP = 600.0   # ticks-until-burn saturation for the gradient term

    for tick in range(args.max_ticks + 1):
        # Group centroids (alive, appeared members only) — used for merges.
        gsum: dict[int, list[float]] = {}
        for pe in persons:
            if pe.state == DEAD or tick < pe.appear_tick:
                continue
            s = gsum.setdefault(pe.group, [0.0, 0.0, 0])
            s[0] += pe.x
            s[1] += pe.y
            s[2] += 1
        centroid = {g: (s[0] / s[2], s[1] / s[2]) for g, s in gsum.items() if s[2] > 0}

        # Group leader = lowest-pid alive MOBILE member (persons is pid-ascending).
        # Members steer toward the leader; the leader navigates for the group.
        leader: dict[int, Person] = {}
        for pe in persons:
            if pe.state in (HEALTHY, INJURED) and tick >= pe.appear_tick and pe.group not in leader:
                leader[pe.group] = pe

        # Group speed = slowest mobile member (immobile members don't cap it;
        # they get abandoned — split event below).
        gspeed: dict[int, float] = {}
        for pe in persons:
            if pe.state in (DEAD, IMMOBILE) or tick < pe.appear_tick:
                continue
            sp = STATE_SPEED[pe.state]
            gspeed[pe.group] = min(gspeed.get(pe.group, 99.0), sp)

        for pe in persons:
            if pe.state == DEAD or tick < pe.appear_tick:
                continue

            cx, cy = int(pe.x), int(pe.y)

            # ---- Exposure ----
            exposed = False
            r = int(math.ceil(args.exposure_radius_cells))
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy <= args.exposure_radius_cells ** 2 and fire_active(cx + dx, cy + dy, tick):
                        exposed = True
                        break
                if exposed:
                    break
            if exposed:
                pe.exposure += 1.0
                old = pe.state
                if pe.exposure >= args.exposure_dead:
                    pe.state = DEAD
                elif pe.exposure >= args.exposure_immobile:
                    pe.state = max(pe.state, IMMOBILE)
                elif pe.exposure >= args.exposure_injure:
                    pe.state = max(pe.state, INJURED)
                if pe.state != old:
                    events.append((tick, "state_change", pe.pid,
                                   f"{STATE_NAMES[old]}->{STATE_NAMES[pe.state]}"))
                    if pe.state in (IMMOBILE, DEAD) and gsum.get(pe.group, [0, 0, 0])[2] > 1:
                        events.append((tick, "split", pe.pid, f"left_behind_group={pe.group}"))

            if pe.state in (IMMOBILE, DEAD):
                continue

            # ---- Alert: projected fire arrival near me within the react horizon ----
            alerted = False
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx_, ny_ = cx + dx, cy + dy
                    if 0 <= nx_ < W and 0 <= ny_ < H:
                        a = arrival[ny_ * W + nx_]
                        if a >= 0 and a <= tick + args.react_horizon_ticks \
                                and tick < a + args.burn_duration_ticks:
                            alerted = True
                            break
                if alerted:
                    break
            if alerted and pe.alert_tick < 0:
                pe.alert_tick = tick

            # ---- Movement direction ----
            mx = my = 0.0

            if alerted:
                # (1) Gradient ascent on the fire-arrival field: move toward
                # cells that burn later/never — away from the front along the
                # actual spread geometry.
                base = arrival_capped(cx, cy, tick, GRAD_CAP)
                if base is not None:
                    gvx = gvy = 0.0
                    for dx, dy in DIRS8:
                        v = arrival_capped(cx + dx, cy + dy, tick, GRAD_CAP)
                        if v is None:
                            continue
                        inv_len = 1.0 / math.sqrt(2.0) if (dx and dy) else 1.0
                        w = (v - base) / GRAD_CAP
                        gvx += dx * inv_len * w
                        gvy += dy * inv_len * w
                    glen = math.hypot(gvx, gvy)
                    if glen > 1e-9:
                        mx += args.grad_weight * gvx / glen
                        my += args.grad_weight * gvy / glen

                # (2) Refuge descent: step toward the neighbour closest (in
                # terrain+danger cost) to a large never-burning region.
                here = refuge_dist[cy * W + cx]
                best_d = here
                best_dir = None
                for dx, dy in DIRS8:           # fixed order = deterministic tie-break
                    nx_, ny_ = cx + dx, cy + dy
                    if 0 <= nx_ < W and 0 <= ny_ < H:
                        d = refuge_dist[ny_ * W + nx_]
                        if d < best_d - 1e-12:
                            best_d = d
                            best_dir = (dx, dy)
                if best_dir is not None:
                    dlen = math.hypot(best_dir[0], best_dir[1])
                    mx += args.refuge_weight * best_dir[0] / dlen
                    my += args.refuge_weight * best_dir[1] / dlen

                # (3) Seeded wrong-way panic: some people initially run TOWARD
                # the danger before correcting.
                if pe.wrong_way and pe.alert_tick >= 0 and tick - pe.alert_tick < args.wrong_way_ticks:
                    mx, my = -mx, -my

                # (4) Leader-following (groups flee as a unit behind the leader).
                ld = leader.get(pe.group)
                if ld is not None and ld.pid != pe.pid:
                    mx += (ld.x - pe.x) * args.cohesion_weight
                    my += (ld.y - pe.y) * args.cohesion_weight
            else:
                # Idle: follow the leader loosely + deterministic wander.
                ld = leader.get(pe.group)
                if ld is not None and ld.pid != pe.pid:
                    mx += (ld.x - pe.x) * args.cohesion_weight
                    my += (ld.y - pe.y) * args.cohesion_weight
                ang = det_noise(args.seed, pe.pid, tick // 40) * 2.0 * math.pi
                mx += math.cos(ang) * args.wander_weight
                my += math.sin(ang) * args.wander_weight

            mlen = math.hypot(mx, my)
            if mlen > 1e-6:
                speed = gspeed.get(pe.group, STATE_SPEED[pe.state])
                if alerted and gsum.get(pe.group, [0, 0, 0])[2] <= 1:
                    speed = STATE_SPEED[pe.state]  # alone: own pace
                if alerted:
                    speed *= args.panic_speed_mult  # panic running
                cost = terrain[cy * W + cx]
                step = speed / cost
                ux, uy = mx / mlen, my / mlen
                nx = pe.x + ux * step
                ny = pe.y + uy * step

                # Never step into a cell that is burning or about to ignite.
                # Try the full step, then axis-aligned slides, else stay put.
                candidates = ((nx, ny), (pe.x + ux * step, pe.y), (pe.x, pe.y + uy * step))
                for candx, candy in candidates:
                    bx = min(W - 0.001, max(0.0, candx))
                    by = min(H - 0.001, max(0.0, candy))
                    if not cell_unsafe_to_enter(int(bx), int(by), tick):
                        pe.x = bx
                        pe.y = by
                        break

        # ---- Group merges (pairwise centroid distance, ascending group id) ----
        gids = sorted(centroid.keys())
        for i in range(len(gids)):
            for j in range(i + 1, len(gids)):
                a, b = gids[i], gids[j]
                if a not in centroid or b not in centroid:
                    continue
                ax, ay = centroid[a]
                bx, by = centroid[b]
                if (ax - bx) ** 2 + (ay - by) ** 2 <= args.group_merge_radius_cells ** 2:
                    for pe in persons:
                        if pe.group == b:
                            pe.group = a
                    centroid[a] = ((ax + bx) / 2.0, (ay + by) / 2.0)
                    del centroid[b]
                    events.append((tick, "merge", -1, f"group_{b}->group_{a}"))

        # ---- Record keyframes ----
        for pe in persons:
            if tick < pe.appear_tick:
                continue
            out_rows.append((tick, pe.pid, f"{pe.x:.3f}", f"{pe.y:.3f}", pe.group, pe.state))

    # ---------------- Write ----------------
    out = args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tick", "person", "x", "y", "group", "state"])
        w.writerows(out_rows)

    ev_path = out.with_name(out.stem + "_events.csv")
    with ev_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tick", "event", "person", "detail"])
        w.writerows(sorted(events))

    n_dead = sum(1 for pe in persons if pe.state == DEAD)
    n_immobile = sum(1 for pe in persons if pe.state == IMMOBILE)
    n_alerted = sum(1 for pe in persons if pe.alert_tick >= 0)
    n_wrong = sum(1 for pe in persons if pe.wrong_way and pe.alert_tick >= 0)
    print(f"Wrote {out} ({len(out_rows)} keyframes, {len(persons)} persons, "
          f"{n_alerted} alerted ({n_wrong} wrong-way starters), "
          f"{n_immobile} immobile, {n_dead} dead) and {ev_path} ({len(events)} events)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
