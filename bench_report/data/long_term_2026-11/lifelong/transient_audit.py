"""How much of the lifelong NCLT map only one session ever saw (descriptive, development).

  transient_audit.py ROOT [--voxel 0.75] [--radius 15] [--min-chances 5] [--out OUT.json]
      [--points OUT.npz]

ROOT is the bench directory of lifelong_chain.sh (``stores/lifelong``, ``lifelong/chain.jsonl``,
``lifelong/run_DAY/odometry_tum.txt``, ``runs/d_base/odometry_tum.txt``). Every point the store
exports (``Store.points``, state kept or kept by an edit) falls in a ``--voxel`` cell. A cell's
**support** is the number of sessions with a point in it; its **chances** are the sessions whose
path (odometry placed by the store transform) passed within ``--radius`` m (XY) of it. A cell
seen by one session although at least ``--min-chances`` sessions passed nearby is a **single-
session cell**: a parked car, a person, a bin moved the next day, or structure the others could
not see from their paths (the count is an upper bound on transient clutter, not a measure of
it). Heights are above the local ground (lowest exported point per 2 m column). One JSON object
with totals, by height band and by session; ``--points`` saves the single-session points.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BANDS = ((-1e9, 0.3, 'ground (< 0.3 m)'), (0.3, 2.2, 'people and cars (0.3-2.2 m)'),
         (2.2, 1e9, 'high (> 2.2 m)'))


def keys(ijk: np.ndarray) -> np.ndarray:
    """One int64 per integer cell (each axis offset into 21 bits)."""
    shifted = ijk.astype(np.int64) + (1 << 20)
    return (shifted[:, 0] << 42) | (shifted[:, 1] << 21) | shifted[:, 2]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('root', type=Path)
    parser.add_argument('--voxel', type=float, default=0.75)
    parser.add_argument('--radius', type=float, default=15.0)
    parser.add_argument('--min-chances', type=int, default=5)
    parser.add_argument('--out', type=Path, default=None)
    parser.add_argument('--points', type=Path, default=None)
    args = parser.parse_args()

    from lidarslam.store import Store

    root = args.root
    store = Store.open(root / 'stores/lifelong')
    entries = store.sessions
    # The store's sessions in chain order (a run that ran out of memory in its export after
    # committing still added its session).
    chain = list(map(json.loads, (root / 'lifelong/chain.jsonl').read_text().splitlines()))
    chain = chain[:len(entries) - 1]
    days = ['2012-01-15'] + [c['day'] for c in chain]

    points = store.points()
    keep = (points['state'] == 0) | (points['state'] == 3)
    xyz = points['xyz'][keep].astype(np.float64)
    session = points['session'][keep].astype(np.int32)
    del points

    # Paths in the store frame.
    paths = []
    for k, entry in enumerate(entries):
        odometry = (root / 'runs/d_base/odometry_tum.txt' if k == 0 else
                    root / f'lifelong/run_{days[k]}/odometry_tum.txt')
        transform = entry.get('store_from_session')
        if isinstance(transform, str):
            transform = json.loads(transform)
        transform = np.eye(4) if transform is None else np.asarray(transform)
        track = np.loadtxt(odometry, ndmin=2)[:, 1:4]
        paths.append(track @ transform[:3, :3].T + transform[:3, 3])

    # Local ground: the lowest point per 2 m column.
    column = np.floor(xyz[:, :2] / 2.0).astype(np.int64)
    column_key = keys(np.c_[column, np.zeros(len(column), np.int64)])
    order = np.argsort(column_key, kind='stable')
    unique_columns, start = np.unique(column_key[order], return_index=True)
    lowest = np.minimum.reduceat(xyz[order, 2], start)
    height = xyz[:, 2] - lowest[np.searchsorted(unique_columns, column_key)]

    # Support: sessions per cell.
    cell = np.floor(xyz / args.voxel).astype(np.int64)
    cell_key = keys(cell)
    pairs = np.unique(np.c_[cell_key, session], axis=0)
    unique_cells, support = np.unique(pairs[:, 0], return_counts=True)
    point_cell = np.searchsorted(unique_cells, cell_key)

    # Chances: sessions whose path passed within the radius of the cell (1 m XY grid).
    centre = (np.c_[(unique_cells >> 42) & ((1 << 21) - 1), (unique_cells >> 21) & ((1 << 21) - 1)]
              - (1 << 20) + 0.5) * args.voxel
    lo = np.floor(centre.min(0) - args.radius - 1).astype(np.int64)
    hi = np.ceil(centre.max(0) + args.radius + 1).astype(np.int64)
    size = hi - lo + 1
    r = int(np.ceil(args.radius))
    disk = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
            if dx * dx + dy * dy <= args.radius ** 2]
    chances = np.zeros(len(unique_cells), np.int32)
    lookup = np.clip(np.floor(centre).astype(np.int64) - lo, 0, size - 1)
    for track in paths:
        visited = np.zeros((size[0], size[1]), bool)
        ij = np.unique(np.floor(track[:, :2]).astype(np.int64) - lo, axis=0)
        ij = ij[(ij >= 0).all(1) & (ij < size).all(1)]
        near = np.zeros_like(visited)
        near[ij[:, 0], ij[:, 1]] = True
        for dx, dy in disk:
            visited[max(dx, 0):size[0] + min(dx, 0), max(dy, 0):size[1] + min(dy, 0)] |= \
                near[max(-dx, 0):size[0] + min(-dx, 0), max(-dy, 0):size[1] + min(-dy, 0)]
        chances += visited[lookup[:, 0], lookup[:, 1]]

    single = (support == 1) & (chances >= args.min_chances)
    point_single = single[point_cell]
    eligible = chances[point_cell] >= args.min_chances

    def share(mask, of):
        return round(100.0 * float(mask.sum()) / max(1, int(of.sum())), 2)

    report = {
        'voxel_m': args.voxel, 'radius_m': args.radius, 'min_chances': args.min_chances,
        'sessions': len(entries), 'points': int(len(xyz)), 'cells': int(len(unique_cells)),
        'eligible_points': int(eligible.sum()),
        'single_session_points': int(point_single.sum()),
        'single_session_points_pct_of_eligible': share(point_single, eligible),
        'single_session_cells_pct_of_eligible': share(single, chances >= args.min_chances),
        'support_histogram_eligible_cells': np.bincount(
            support[chances >= args.min_chances], minlength=len(entries) + 1).tolist(),
        'by_height': [], 'by_session': []}
    for low, high, name in BANDS:
        band = (height >= low) & (height < high)
        report['by_height'].append({'band': name, 'eligible_points': int((band & eligible).sum()),
                                    'single_session_pct': share(point_single & band,
                                                                eligible & band)})
    for k, day in enumerate(days):
        own = session == k
        report['by_session'].append({
            'session': k, 'day': day, 'points': int(own.sum()),
            'single_session_points': int((point_single & own).sum()),
            'single_session_pct_of_own_eligible': share(point_single & own, eligible & own)})
    text = json.dumps(report, indent=1)
    print(text)
    if args.out:
        args.out.write_text(text + '\n', encoding='utf-8')
    if args.points:
        np.savez_compressed(args.points, xyz=xyz[point_single].astype(np.float32),
                            session=session[point_single].astype(np.int16),
                            height=height[point_single].astype(np.float32))
    return 0


if __name__ == '__main__':
    sys.exit(main())
