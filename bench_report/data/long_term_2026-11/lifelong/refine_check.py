"""T and S of an update appended again with another change mode (preregistered check, roadmap).

  refine_check.py LIFELONG_STORE RERUN_STORE UPDATE [--label NAME]

Cells (0.75 m) and their session support come from the lifelong store's exported points (state
kept or kept by an edit); heights are above the lowest of those points in the 2 m column. The
rerun store holds the same sessions before UPDATE and UPDATE appended again. T: of the rerun
update's points at 0.3-2.2 m in cells only UPDATE has points in, the share its export leaves out.
S: of its points in cells at least three sessions have points in, the share its export keeps.
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

VOXEL = 0.75


def keys(ijk: np.ndarray) -> np.ndarray:
    shifted = ijk.astype(np.int64) + (1 << 20)
    return (shifted[:, 0] << 42) | (shifted[:, 1] << 21) | shifted[:, 2]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('lifelong')
    parser.add_argument('rerun')
    parser.add_argument('update', type=int)
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    from lidarslam.store import Store

    base = Store.open(args.lifelong).points()
    keep = (base['state'] == 0) | (base['state'] == 3)
    xyz = base['xyz'][keep].astype(np.float64)
    session = base['session'][keep].astype(np.int64)
    del base
    column = keys(np.c_[np.floor(xyz[:, :2] / 2.0).astype(np.int64), np.zeros(len(xyz), np.int64)])
    order = np.argsort(column, kind='stable')
    columns, start = np.unique(column[order], return_index=True)
    lowest = np.minimum.reduceat(xyz[order, 2], start)
    cell = keys(np.floor(xyz / VOXEL).astype(np.int64))
    pairs = np.unique(np.c_[cell, session], axis=0)
    cells, support = np.unique(pairs[:, 0], return_counts=True)
    only_update = np.zeros(len(cells), bool)
    single = support == 1
    owner = np.full(len(cells), -1, np.int64)
    owner[np.searchsorted(cells, pairs[:, 0])] = pairs[:, 1]
    only_update = single & (owner == args.update)
    del xyz, session, cell, pairs, column, order

    rerun = Store.open(args.rerun).points()
    mine = rerun['session'] == args.update
    points = rerun['xyz'][mine].astype(np.float64)
    kept = ((rerun['state'] == 0) | (rerun['state'] == 3))[mine]
    del rerun
    point_cell = keys(np.floor(points / VOXEL).astype(np.int64))
    index = np.clip(np.searchsorted(cells, point_cell), 0, len(cells) - 1)
    known = cells[index] == point_cell
    point_column = keys(np.c_[np.floor(points[:, :2] / 2.0).astype(np.int64),
                              np.zeros(len(points), np.int64)])
    column_index = np.clip(np.searchsorted(columns, point_column), 0, len(columns) - 1)
    height = np.where(columns[column_index] == point_column,
                      points[:, 2] - lowest[column_index], np.nan)
    band = (height >= 0.3) & (height < 2.2)
    transient = known & only_update[index] & band
    structure = known & (support[index] >= 3)
    print(json.dumps({
        'label': args.label, 'update': args.update, 'points': int(len(points)),
        'kept_share': round(float(kept.mean()), 4),
        'transient_points': int(transient.sum()),
        'T_removed_share': round(float((~kept[transient]).mean()), 4) if transient.any() else None,
        'structure_points': int(structure.sum()),
        'S_kept_share': round(float(kept[structure].mean()), 4) if structure.any() else None}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
