"""Height drift of one session against the sessions before it (no ground truth).

  drift_check.py STORE UPDATE [--references N] [--radius 1.5] [--label NAME]

The committed submap positions of session UPDATE (its index in the store) are compared with
those of the first N sessions (default: every session before UPDATE). For each update submap
with at least three reference submap positions within ``--radius`` m (XY), dz is its height
minus their median height. One JSON line: the number of compared submaps, the median dz, the
95th percentile of |dz| and the median dz of each quarter of them in session order.
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('store')
    parser.add_argument('update', type=int)
    parser.add_argument('--references', type=int, default=None)
    parser.add_argument('--radius', type=float, default=1.5)
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    from scipy.spatial import cKDTree

    from lidarslam.relocalization import StartupRelocalization
    from lidarslam.store import Store

    aligned = Store.open(args.store)._aligned_sessions()
    relocalizer = StartupRelocalization(aligned, None, anywhere=False)
    positions = [np.array([np.asarray(p)[:3, 3] for p in poses])
                 for poses in relocalizer.matcher.session_poses()]
    count = args.update if args.references is None else args.references
    reference = np.concatenate(positions[:count])
    update = positions[args.update]
    tree = cKDTree(reference[:, :2])
    dz = []
    for position in update:
        near = tree.query_ball_point(position[:2], args.radius)
        if len(near) >= 3:
            dz.append(position[2] - float(np.median(reference[near, 2])))
    dz = np.asarray(dz)
    quarters = [round(float(np.median(part)), 3) for part in np.array_split(dz, 4)] if len(
        dz) >= 4 else []
    print(json.dumps({
        'label': args.label, 'store': args.store, 'update': args.update,
        'references': count, 'submaps': int(len(update)), 'compared': int(len(dz)),
        'median_dz_m': round(float(np.median(dz)), 3) if len(dz) else None,
        'p95_abs_dz_m': round(float(np.percentile(np.abs(dz), 95)), 3) if len(dz) else None,
        'quarters_median_dz_m': quarters}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
