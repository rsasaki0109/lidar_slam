"""Height of each lifelong update's odometry against the other sessions (no ground truth).

  odometry_height.py ROOT ARM_DIR [--label NAME]

ROOT is the bench directory of lifelong_chain.sh. ARM_DIR holds ``DAY/odometry_tum.txt`` for the
updates. Each odometry is placed by the store transform its session committed in
``stores/lifelong``; every 5th pose with at least three submap positions of the other sessions
within 1.5 m (XY) gives dz, its height minus their median height. One JSON line per session:
the number of compared poses and the 95th percentile of |dz|.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'lidarslam_core/python'))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('root', type=Path)
    parser.add_argument('arm', type=Path)
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    from lidarslam.height_check import height_differences, submap_positions
    from lidarslam.store import Store

    store = Store.open(args.root / 'stores/lifelong')
    aligned = store._aligned_sessions()
    positions = submap_positions(*zip(*aligned))
    days = [json.loads(line)['day'] for line in
            (args.root / 'lifelong/chain.jsonl').read_text().splitlines()][:len(aligned) - 1]
    for k, day in enumerate(days, start=1):
        path = args.arm / day / 'odometry_tum.txt'
        if not path.is_file():
            print(json.dumps({'label': args.label, 'day': day, 'missing': True}))
            continue
        transform = np.asarray(aligned[k][1], dtype=float)
        track = np.loadtxt(path, ndmin=2)[::5, 1:4] @ transform[:3, :3].T + transform[:3, 3]
        others = np.concatenate([p for j, p in enumerate(positions) if j != k])
        dz = height_differences(others, track)
        print(json.dumps({'label': args.label, 'day': day, 'compared': int(len(dz)),
                          'p95_abs_dz_m': round(float(np.percentile(np.abs(dz), 95)), 3)
                          if len(dz) else None}), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
