"""Height of the gate-area updates' odometry against the gate base session (no ground truth).

  gate_height.py ROOT ARM_DIR [--label NAME]

ROOT is the NCLT bench directory (``stores/gate_store`` and ``stores/any_gate_DAY_in``, whose
second session holds the store transform the update committed). ARM_DIR holds
``DAY/odometry_tum.txt``. Measure as odometry_height.py, against the base session's submaps.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

DAYS = ('2012-02-04', '2012-02-19', '2013-01-10', '2013-04-05')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('root', type=Path)
    parser.add_argument('arm', type=Path)
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    from lidarslam.height_check import height_differences, submap_positions

    reference = submap_positions([args.root / 'stores/gate_store/sessions/0000'], [np.eye(4)])[0]
    for day in DAYS:
        entry = json.loads((args.root / f'stores/any_gate_{day}_in/store.json').read_text())
        transform = entry['sessions'][1]['store_from_session']
        transform = np.asarray(json.loads(transform) if isinstance(transform, str) else transform)
        track = np.loadtxt(args.arm / day / 'odometry_tum.txt', ndmin=2)[::5, 1:4]
        track = track @ transform[:3, :3].T + transform[:3, 3]
        dz = height_differences(reference, track)
        print(json.dumps({'label': args.label, 'day': day, 'compared': int(len(dz)),
                          'p95_abs_dz_m': round(float(np.percentile(np.abs(dz), 95)), 3)
                          if len(dz) else None}), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
