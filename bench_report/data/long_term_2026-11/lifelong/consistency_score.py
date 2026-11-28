"""Height consistency of every session of a store against the others (preregistered, roadmap).

  consistency_score.py STORE [--label NAME] > OUT.jsonl

For each session: the 95th percentile of |dz| of its submap positions against those of all the
other sessions of the store (lidarslam.height_check.height_differences: 1.5 m, three
neighbours). One JSON line per session; the session's first submap stamp identifies its day.
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('store')
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    from lidarslam.height_check import height_differences, submap_positions
    from lidarslam.store import Store

    store = Store.open(args.store)
    aligned = store._aligned_sessions()
    positions = submap_positions(*zip(*aligned))
    entries = store.sessions
    for k, own in enumerate(positions):
        others = np.concatenate([p for i, p in enumerate(positions) if i != k])
        d = height_differences(others, own)
        print(json.dumps({
            'label': args.label, 'session': k, 'id': entries[k]['id'],
            'first_stamp_sec': entries[k].get('first_stamp_sec'),
            'submaps': int(len(own)), 'compared': int(len(d)),
            'p95_abs_dz_m': round(float(np.percentile(np.abs(d), 95)), 3) if len(d) else None}),
            flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
