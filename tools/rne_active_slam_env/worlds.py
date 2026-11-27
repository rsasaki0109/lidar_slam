"""Worlds of the closed-loop active SLAM evaluation (docs/roadmap/active-slam-2026-11.md).

A world is a dict for rne_active_slam_env (``boxes``, ``start``, ``seed``) plus what the harness
needs: ``starts`` (10 start poses [x, y, yaw]; a seed picks one) and ``free_region`` (the free
floor, for the coverage metric: ``outer`` minus ``holes``, or the union of ``free_rects``).

Development worlds (tuned on): ``ring``, ``grid``, and ``branches2``, ``long_loop2`` (the second
holdout, run once on 2026-11-27 with a sensor bug, so seen). ``branches`` and ``big_loop`` were
the first holdout, retired. ``dogleg_loop`` and ``mine_branches`` are the third holdout. All are
built the same way: walls 2.5 m high, a floor box, and pilasters (0.3 m wide, 0.2 m deep) along
the walls as door frames; flat walls alone leave a corridor's axis unobservable to the LiDAR.
"""

from __future__ import annotations

import math
from typing import Any

HEIGHT = 2.5


def _floor(width: float, height: float) -> dict[str, Any]:
    return {'min': [-1.0, -1.0, -0.1], 'max': [width + 1.0, height + 1.0, 0.0]}


def _wall(x0: float, y0: float, x1: float, y1: float, height: float = HEIGHT) -> dict[str, Any]:
    return {'min': [min(x0, x1), min(y0, y1), 0.0], 'max': [max(x0, x1), max(y0, y1), height]}


def _inside(rects: list[list[float]], x: float, y: float) -> bool:
    return any(r[0] <= x < r[2] and r[1] <= y < r[3] for r in rects)


def rect_world(width: int, height: int, free_rects: list[list[float]], starts: list[list[float]],
               pilaster_spacing: int, seed: int, crates: list[tuple[float, float]] = ()
               ) -> dict[str, Any]:
    """Build a world whose free floor is the union of free_rects (integer metres).

    Everything else inside [0, width] x [0, height] is solid: 1 m cells, merged into boxes
    (row runs, then equal runs of consecutive rows). Pilasters stand on the free side of each
    solid edge every pilaster_spacing metres.
    """
    solid = [[not _inside(free_rects, x + 0.5, y + 0.5) for x in range(width)]
             for y in range(height)]
    # Row runs, then merge equal runs of consecutive rows.
    open_runs: dict[tuple[int, int], int] = {}  # (x0, x1) -> start row
    boxes = [_floor(width, height)]

    def close(x0: int, x1: int, y0: int, y1: int) -> None:
        boxes.append({'min': [float(x0), float(y0), 0.0], 'max': [float(x1), float(y1), HEIGHT]})

    for y in range(height + 1):
        runs: set[tuple[int, int]] = set()
        if y < height:
            x = 0
            while x < width:
                if solid[y][x]:
                    x0 = x
                    while x < width and solid[y][x]:
                        x += 1
                    runs.add((x0, x))
                else:
                    x += 1
        for run in sorted(set(open_runs) - runs):
            close(run[0], run[1], open_runs.pop(run), y)
        for run in sorted(runs - set(open_runs)):
            open_runs[run] = y
    # Outer walls (thin, outside the area).
    t = 0.2
    boxes += [_wall(-t, -t, width + t, 0), _wall(-t, height, width + t, height + t),
              _wall(-t, 0, 0, height), _wall(width, 0, width + t, height)]
    # Pilasters: on free cells next to a solid cell (or the outer wall), every spacing metres.
    for y in range(height):
        for x in range(width):
            if solid[y][x]:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                wall = not (0 <= nx < width and 0 <= ny < height) or solid[ny][nx]
                along = y if dx != 0 else x
                if not wall or along % pilaster_spacing != 0:
                    continue
                if dx == 1:
                    lo, hi = [x + 0.8, y - 0.15], [x + 1.0, y + 0.15]
                elif dx == -1:
                    lo, hi = [x, y - 0.15], [x + 0.2, y + 0.15]
                elif dy == 1:
                    lo, hi = [x - 0.15, y + 0.8], [x + 0.15, y + 1.0]
                else:
                    lo, hi = [x - 0.15, y], [x + 0.15, y + 0.2]
                boxes.append({'min': lo + [0.0], 'max': hi + [HEIGHT]})
    for i, (cx, cy) in enumerate(crates):
        size = 0.6 + 0.1 * (i % 3)
        boxes.append({'min': [cx - size / 2, cy - size / 2, 0.0],
                      'max': [cx + size / 2, cy + size / 2, 0.8 + 0.2 * (i % 4)]})
    return {'boxes': boxes, 'start': starts[0], 'starts': starts, 'seed': seed,
            'free_region': {'free_rects': free_rects}}


# ------------------------------------------------------------------- development worlds


def world_ring() -> dict[str, Any]:
    """Return a 5 m wide corridor around a 30 m x 14 m block (a ~100 m loop)."""
    boxes = [_floor(40.0, 24.0)]
    t = 0.2
    boxes += [_wall(-t, -t, 40 + t, 0), _wall(-t, 24, 40 + t, 24 + t),
              _wall(-t, 0, 0, 24), _wall(40, 0, 40 + t, 24)]
    boxes.append({'min': [5.0, 5.0, 0.0], 'max': [35.0, 19.0, 2.5]})  # the block
    crates = [(1.0, 6.0), (4.0, 11.0), (1.0, 17.0), (9.0, 23.0), (16.0, 20.0), (24.0, 23.0),
              (31.0, 20.0), (39.0, 15.0), (36.0, 8.0), (39.0, 3.0), (27.0, 1.0), (20.0, 4.0),
              (12.0, 1.0)]
    for i, (cx, cy) in enumerate(crates):
        size = 0.6 + 0.1 * (i % 3)
        boxes.append({'min': [cx - size / 2, cy - size / 2, 0.0],
                      'max': [cx + size / 2, cy + size / 2, 0.8 + 0.2 * (i % 4)]})
    for k in range(1, 14):
        x = 3.0 * k
        boxes.append({'min': [x - 0.15, 0.0, 0.0], 'max': [x + 0.15, 0.2, 2.5]})
        boxes.append({'min': [x - 0.15, 23.8, 0.0], 'max': [x + 0.15, 24.0, 2.5]})
        if 5.0 < x < 35.0:
            boxes.append({'min': [x - 0.15, 4.8, 0.0], 'max': [x + 0.15, 5.0, 2.5]})
            boxes.append({'min': [x - 0.15, 19.0, 0.0], 'max': [x + 0.15, 19.2, 2.5]})
    for k in range(1, 8):
        y = 3.0 * k
        boxes.append({'min': [0.0, y - 0.15, 0.0], 'max': [0.2, y + 0.15, 2.5]})
        boxes.append({'min': [39.8, y - 0.15, 0.0], 'max': [40.0, y + 0.15, 2.5]})
        if 5.0 < y < 19.0:
            boxes.append({'min': [4.8, y - 0.15, 0.0], 'max': [5.0, y + 0.15, 2.5]})
            boxes.append({'min': [35.0, y - 0.15, 0.0], 'max': [35.2, y + 0.15, 2.5]})
    starts = [[2.5, 12.0, math.pi / 2], [20.0, 2.5, 0.0], [37.5, 12.0, math.pi / 2],
              [20.0, 21.5, math.pi], [2.5, 3.0, 0.0], [37.0, 3.0, math.pi / 2],
              [37.0, 21.0, math.pi], [3.0, 21.0, -math.pi / 2], [12.0, 2.5, math.pi],
              [28.0, 21.5, 0.0]]
    return {'boxes': boxes, 'start': starts[0], 'starts': starts, 'seed': 1,
            'free_region': {'outer': [0, 0, 40, 24], 'holes': [[5, 5, 35, 19]]}}


def world_grid() -> dict[str, Any]:
    """Return 3 m corridors around a 3 x 2 grid of 15 m x 13 m blocks (57 m x 35 m)."""
    c, bx, by = 3, 15, 13
    width, height = 3 * bx + 4 * c, 2 * by + 3 * c
    xs = [i * (bx + c) for i in range(4)]  # corridor starts
    ys = [j * (by + c) for j in range(3)]
    free = [[x, 0, x + c, height] for x in xs] + [[0, y, width, y + c] for y in ys]
    starts = [[1.5, 1.5, 0.0], [19.5, 9.0, math.pi / 2], [37.5, 25.0, -math.pi / 2],
              [55.5, 9.0, math.pi / 2], [10.0, 17.5, 0.0], [28.0, 33.5, math.pi],
              [46.0, 1.5, math.pi], [1.5, 25.0, -math.pi / 2], [46.0, 17.5, 0.0],
              [28.0, 1.5, 0.0]]
    return rect_world(width, height, free, starts, pilaster_spacing=3, seed=2)

# ------------------------------------------------------------------- holdout worlds


def world_branches() -> dict[str, Any]:
    """Return a main corridor with dead-end side corridors (some ending in rooms).

    The main corridor (x 3..63, y 20..23) has four dead ends going up and two going down; the
    only loop is the main corridor joined back to its start through a long return corridor
    (down at x 60..63, along y 3..6, up at x 3..6).
    """
    free = [
        [3, 20, 63, 23],                       # main corridor
        [60, 3, 63, 20], [3, 3, 63, 6], [3, 6, 6, 20],  # the return loop
        [10, 23, 13, 40],                      # dead end up
        [22, 23, 25, 38], [19, 38, 28, 44],    # dead end up into a room
        [34, 23, 37, 43],                      # dead end up
        [46, 23, 49, 36], [44, 36, 52, 44],    # dead end up into a room
        [16, 9, 19, 20], [40, 9, 43, 20],      # dead ends down (stop 3 m short of y = 6)
    ]
    starts = [[8.0, 21.5, 0.0], [30.0, 21.5, math.pi], [61.5, 12.0, -math.pi / 2],
              [30.0, 4.5, math.pi], [4.5, 12.0, math.pi / 2], [11.5, 30.0, -math.pi / 2],
              [23.5, 41.0, -math.pi / 2], [35.5, 30.0, -math.pi / 2], [48.0, 40.0, -math.pi / 2],
              [41.5, 14.0, math.pi / 2]]
    crates = [(15.0, 22.2), (28.0, 20.8), (52.0, 22.2), (61.0, 8.0), (45.0, 4.0), (20.0, 5.2),
              (4.0, 15.0), (11.0, 35.0), (35.0, 38.0), (17.0, 12.0)]
    return rect_world(66, 46, free, starts, pilaster_spacing=3, seed=3, crates=crates)


def world_big_loop() -> dict[str, Any]:
    """Return a 3 m corridor around a 70 m x 40 m block (~230 m loop), pilasters every 6 m."""
    free = [[0, 0, 76, 3], [0, 43, 76, 46], [0, 3, 3, 43], [73, 3, 76, 43]]
    starts = [[1.5, 20.0, math.pi / 2], [38.0, 1.5, 0.0], [74.5, 23.0, math.pi / 2],
              [38.0, 44.5, math.pi], [1.5, 1.5, 0.0], [74.5, 1.5, math.pi / 2],
              [74.5, 44.5, math.pi], [1.5, 44.5, -math.pi / 2], [24.0, 1.5, math.pi],
              [56.0, 44.5, 0.0]]
    crates = [(1.0, 10.0), (20.0, 2.2), (60.0, 0.8), (75.0, 30.0), (40.0, 45.2), (2.0, 35.0)]
    return rect_world(76, 46, free, starts, pilaster_spacing=6, seed=4, crates=crates)


def world_branches2() -> dict[str, Any]:
    """Return 4 m corridors: a main corridor with six dead ends (two into rooms) and one loop.

    Second holdout, after the first (``branches``) was cut into pieces by crates in its 3 m
    corridors. Crates stand in the rooms only.
    """
    free = [
        [4, 22, 66, 26],                                  # main corridor
        [62, 4, 66, 22], [4, 4, 66, 8], [4, 8, 8, 22],    # the return loop
        [12, 26, 16, 44],                                 # dead end up
        [26, 26, 30, 40], [22, 40, 34, 48],               # dead end up into a room
        [40, 26, 44, 46],                                 # dead end up
        [52, 26, 56, 38], [48, 38, 60, 48],               # dead end up into a room
        [20, 11, 24, 22], [46, 11, 50, 22],               # dead ends down (3 m short of y = 8)
    ]
    starts = [[10.0, 24.0, 0.0], [35.0, 24.0, math.pi], [64.0, 14.0, -math.pi / 2],
              [35.0, 6.0, math.pi], [6.0, 14.0, math.pi / 2], [14.0, 35.0, -math.pi / 2],
              [28.0, 44.0, -math.pi / 2], [42.0, 36.0, -math.pi / 2], [54.0, 43.0, -math.pi / 2],
              [48.0, 16.0, math.pi / 2]]
    crates = [(25.0, 46.0), (31.5, 42.5), (50.5, 45.5), (57.5, 40.5)]
    return rect_world(70, 50, free, starts, pilaster_spacing=3, seed=5, crates=crates)


def world_long_loop2() -> dict[str, Any]:
    """Return a 4 m corridor around a 66 m x 36 m block (~220 m loop), pilasters every 6 m.

    Second holdout, after the first (``big_loop``) was cut into pieces by crates in its 3 m
    corridor. No crates: the sparse pilasters are the only features along the walls.
    """
    free = [[0, 0, 74, 4], [0, 40, 74, 44], [0, 4, 4, 40], [70, 4, 74, 40]]
    starts = [[2.0, 22.0, math.pi / 2], [37.0, 2.0, 0.0], [72.0, 22.0, math.pi / 2],
              [37.0, 42.0, math.pi], [2.0, 2.0, 0.0], [72.0, 2.0, math.pi / 2],
              [72.0, 42.0, math.pi], [2.0, 42.0, -math.pi / 2], [20.0, 2.0, math.pi],
              [55.0, 42.0, 0.0]]
    return rect_world(74, 44, free, starts, pilaster_spacing=6, seed=6)


# ------------------------------------------------------------------- degenerate worlds


def world_tunnel_loop() -> dict[str, Any]:
    """Return a 4 m tunnel around a 156 m x 36 m block (~400 m loop), pilasters every 6 m.

    The long sides are 164 m: midway, both end walls are beyond the Mid-360's 70 m, so along
    the tunnel only the pilasters constrain the odometry (thin them to set the difficulty).
    """
    free = [[0, 0, 164, 4], [0, 40, 164, 44], [0, 4, 4, 40], [160, 4, 164, 40]]
    starts = [[2.0, 22.0, math.pi / 2], [82.0, 2.0, 0.0], [162.0, 22.0, math.pi / 2],
              [82.0, 42.0, math.pi], [2.0, 2.0, 0.0], [162.0, 2.0, math.pi / 2],
              [162.0, 42.0, math.pi], [2.0, 42.0, -math.pi / 2], [40.0, 2.0, math.pi],
              [120.0, 42.0, 0.0]]
    return rect_world(164, 44, free, starts, pilaster_spacing=6, seed=7)


def world_long_tunnels() -> dict[str, Any]:
    """Return two parallel 200 m tunnels joined at both ends and in the middle (two loops).

    4 m tunnels, pilasters every 6 m; the loops are ~310 m each, ~510 m around the outside.
    """
    free = [[0, 0, 200, 4], [0, 56, 200, 60], [0, 4, 4, 56], [196, 4, 200, 56],
            [98, 4, 102, 56]]
    starts = [[2.0, 30.0, math.pi / 2], [50.0, 2.0, 0.0], [100.0, 30.0, -math.pi / 2],
              [150.0, 58.0, math.pi], [198.0, 30.0, math.pi / 2], [2.0, 2.0, 0.0],
              [150.0, 2.0, 0.0], [50.0, 58.0, math.pi], [198.0, 58.0, math.pi],
              [100.0, 2.0, math.pi / 2]]
    return rect_world(200, 60, free, starts, pilaster_spacing=6, seed=8)


# ------------------------------------------------------------------- third holdout


def world_dogleg_loop() -> dict[str, Any]:
    """Return a 4 m tunnel loop with a dogleg (~440 m) and a 20 m dead-end spur.

    Bottom 150 m, up 40 m, left 80 m, up 30 m, left 74 m, down 70 m; the spur leaves the
    bottom tunnel at x = 100. The two longest straights (150 m, 70+ m) put the end walls out of
    range midway, as in the tunnels; the dogleg's corners are the only other constraint.
    """
    free = [[0, 0, 150, 4], [146, 4, 150, 40], [70, 36, 146, 40], [70, 40, 74, 70],
            [0, 66, 70, 70], [0, 4, 4, 66], [100, 4, 104, 24]]
    starts = [[2.0, 34.0, math.pi / 2], [60.0, 2.0, 0.0], [148.0, 20.0, math.pi / 2],
              [110.0, 38.0, math.pi], [72.0, 52.0, math.pi / 2], [36.0, 68.0, math.pi],
              [2.0, 2.0, 0.0], [130.0, 2.0, math.pi], [102.0, 18.0, -math.pi / 2],
              [20.0, 68.0, 0.0]]
    return rect_world(150, 70, free, starts, pilaster_spacing=6, seed=9)


def world_mine_branches() -> dict[str, Any]:
    """Return a 190 m main tunnel with one loop above it and three dead ends (4 m tunnels).

    The loop (~230 m) runs up at x = 30, along a top gallery and down at x = 116; dead ends go
    down at x = 60 (28 m) and x = 150 (24 m) and up at x = 160 (20 m).
    """
    free = [[0, 28, 190, 32], [30, 32, 34, 56], [30, 56, 120, 60], [116, 32, 120, 56],
            [60, 0, 64, 28], [150, 4, 154, 28], [160, 32, 164, 52]]
    starts = [[2.0, 30.0, 0.0], [90.0, 30.0, math.pi], [188.0, 30.0, math.pi],
              [32.0, 44.0, math.pi / 2], [75.0, 58.0, 0.0], [118.0, 44.0, -math.pi / 2],
              [62.0, 10.0, math.pi / 2], [152.0, 12.0, math.pi / 2], [162.0, 46.0, -math.pi / 2],
              [140.0, 30.0, 0.0]]
    return rect_world(190, 60, free, starts, pilaster_spacing=6, seed=10)


# branches2 and long_loop2 were the second holdout (run once on 2026-11-27); seen, so tuned on now.
DEVELOPMENT = {'ring': world_ring, 'grid': world_grid, 'branches2': world_branches2,
               'long_loop2': world_long_loop2, 'tunnel_loop': world_tunnel_loop,
               'long_tunnels': world_long_tunnels}
# The first holdout worlds, run once on 2026-11-27 and found unusable (see the roadmap); seen.
RETIRED = {'branches': world_branches, 'big_loop': world_big_loop}
# The third holdout: built 2026-11-27 after the configuration was frozen; checked (reach, start
# clearance, odometry drift on fixed routes, a 60 s check run from every start) and approved
# before it is run once.
HOLDOUT = {'dogleg_loop': world_dogleg_loop, 'mine_branches': world_mine_branches}
WORLDS = {**DEVELOPMENT, **RETIRED, **HOLDOUT}


def reachable_fractions(world: dict[str, Any], inflation: float = 0.8,
                        resolution: float = 0.25) -> list[float]:
    """Return, per start, the fraction of the floor a robot can stand on that it can reach.

    A cell can be stood on when it is farther than ``inflation`` from every obstacle (the
    explorer's planner inflates obstacles so); reach is 4-connected over such cells from the
    start's cell. A usable world gives 1.0 for every start: on the first holdout worlds, crates
    and pilasters in 3 m corridors cut them into pieces (8-42 % reachable).
    """
    import numpy as np  # noqa: I900 (numpy is a dependency of the harness)
    from collections import deque

    obstacles = [b for b in world['boxes'] if b['max'][2] > 0.05]
    width = int(max(b['max'][0] for b in obstacles) / resolution) + 2
    height = int(max(b['max'][1] for b in obstacles) / resolution) + 2
    cx = (np.arange(width) + 0.5) * resolution
    cy = (np.arange(height) + 0.5) * resolution
    gx, gy = np.meshgrid(cx, cy, indexing='ij')
    inside = np.zeros((width, height), dtype=bool)
    blocked = np.zeros((width, height), dtype=bool)
    for b in obstacles:
        dx = np.maximum(np.maximum(b['min'][0] - gx, 0.0), gx - b['max'][0])
        dy = np.maximum(np.maximum(b['min'][1] - gy, 0.0), gy - b['max'][1])
        distance = np.hypot(dx, dy)
        inside |= distance == 0.0
        blocked |= distance < inflation
    standable = ~inside & ~blocked
    fractions = []
    for sx, sy, _ in world['starts']:
        start = (int(sx / resolution), int(sy / resolution))
        seen = np.zeros_like(blocked)
        if not standable[start]:
            fractions.append(0.0)
            continue
        seen[start] = True
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for ox, oy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + ox, y + oy
                if 0 <= nx < width and 0 <= ny < height and standable[nx, ny] and \
                        not seen[nx, ny]:
                    seen[nx, ny] = True
                    queue.append((nx, ny))
        fractions.append(float(seen[standable].sum() / standable.sum()))
    return fractions


# Fixed calibration routes (world-frame waypoints along corridor centre lines), driven on the
# true pose to measure the odometry's drift without any exploration policy.
ROUTES = {
    'ring': [(2.5, 2.5), (37.5, 2.5), (37.5, 21.5), (2.5, 21.5), (2.5, 2.5), (37.5, 2.5),
             (37.5, 21.5), (2.5, 21.5), (2.5, 4.5)],
    'grid': [(1.5, 1.5), (55.5, 1.5), (55.5, 33.5), (1.5, 33.5), (1.5, 17.5), (55.5, 17.5),
             (55.5, 3.5)],
    'branches2': [(6.0, 24.0), (64.0, 24.0), (64.0, 6.0), (6.0, 6.0), (6.0, 24.0), (14.0, 24.0),
                  (14.0, 42.0), (14.0, 24.0), (42.0, 24.0), (42.0, 44.0), (42.0, 26.0)],
    'long_loop2': [(2.0, 2.0), (72.0, 2.0), (72.0, 42.0), (2.0, 42.0), (2.0, 4.0)],
    'tunnel_loop': [(2.0, 2.0), (162.0, 2.0), (162.0, 42.0), (2.0, 42.0), (2.0, 4.0)],
    'long_tunnels': [(2.0, 2.0), (198.0, 2.0), (198.0, 58.0), (100.0, 58.0), (100.0, 4.0)],
    'dogleg_loop': [(2.0, 2.0), (148.0, 2.0), (148.0, 38.0), (72.0, 38.0), (72.0, 68.0),
                    (2.0, 68.0), (2.0, 4.0)],
    'mine_branches': [(2.0, 30.0), (118.0, 30.0), (118.0, 58.0), (32.0, 58.0), (32.0, 30.0),
                      (188.0, 30.0)],
}


def is_pilaster(box: dict[str, Any]) -> bool:
    """Return whether a box is a pilaster (0.3 m x 0.2 m, full height)."""
    size = sorted(round(box['max'][i] - box['min'][i], 3) for i in (0, 1))
    return size == [0.2, 0.3] and box['max'][2] >= HEIGHT


def thin_pilasters(world: dict[str, Any], spacing_m: int) -> dict[str, Any]:
    """Keep only the pilasters at multiples of spacing_m along their wall (0: all, < 0: none).

    The scene's feature density, a difficulty knob set without looking at any policy.
    """
    if spacing_m == 0:
        return world
    kept = []
    for box in world['boxes']:
        if is_pilaster(box):
            wide_in_x = box['max'][0] - box['min'][0] > box['max'][1] - box['min'][1]
            along = 0.5 * (box['min'][0] + box['max'][0]) if wide_in_x else \
                0.5 * (box['min'][1] + box['max'][1])
            if spacing_m < 0 or round(along) % spacing_m != 0:
                continue
        kept.append(box)
    world['boxes'] = kept
    return world


def world_for(name: str, seed: int, pilaster_spacing_m: int = 0,
              imu_noise_scale: float = 1.0, route: bool = False) -> dict[str, Any]:
    """Return a world with the seed's start pose (seed % 10) and noise seed.

    pilaster_spacing_m and imu_noise_scale set the difficulty (see thin_pilasters); with route,
    the start is the first waypoint of ROUTES[name], facing the second.
    """
    world = thin_pilasters(WORLDS[name](), pilaster_spacing_m)
    world['start'] = world['starts'][seed % 10]
    if route:
        (x0, y0), (x1, y1) = ROUTES[name][:2]
        world['start'] = [x0, y0, math.atan2(y1 - y0, x1 - x0)]
        world['route'] = [list(p) for p in ROUTES[name]]
    world['seed'] = seed
    world['imu_noise_scale'] = imu_noise_scale
    return world
