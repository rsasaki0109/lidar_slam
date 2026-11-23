"""Which stage of ``match_anywhere`` fails for keyframes that are in the store (post hoc).

  registration_diagnosis.py --store STORE --queries Q.npz --base-map BASE.pcd
      --base-odometry BASE.tum --base-gt BASE.npy --session-map S.pcd
      --session-odometry S.tum --session-gt S.npy [--label NAME] [--every 2]

The session is placed in the store by ICP of its map against the base map from the ground-truth
placement (as ``evaluate_relocalize_anywhere.py``). For every ``--every``-th recorded keyframe
that this placement puts within 5 m of a stored keyframe, one JSON line with:

- the Scan Context distances and position errors of the eight closest stored keyframes
  (``seeds``: distance, metres from the true position, yaw error of the seed's start in degrees);
- ``from_truth``: whether ``match_with_reference`` verifies the keyframe from its true pose, and
  how far the verified pose lies from it.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[3] / 'scripts'))
from evaluate_nclt_map_update import icp, load  # noqa: E402
from evaluate_relocalize_anywhere import world_from_run  # noqa: E402
from gate_study import Searcher, turned  # noqa: E402


def yaw_deg(a: np.ndarray, b: np.ndarray) -> float:
    d = np.arctan2(a[1, 0], a[0, 0]) - np.arctan2(b[1, 0], b[0, 0])
    return float(abs(np.degrees(np.arctan2(np.sin(d), np.cos(d)))))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for name in ('store', 'queries', 'base-map', 'base-odometry', 'base-gt', 'session-map',
                 'session-odometry', 'session-gt'):
        parser.add_argument(f'--{name}', required=True)
    parser.add_argument('--label', default='')
    parser.add_argument('--every', type=int, default=2)
    args = parser.parse_args()

    base = np.loadtxt(args.base_odometry, ndmin=2)
    session = np.loadtxt(args.session_odometry, ndmin=2)
    world_from_base, _ = world_from_run(base, np.load(args.base_gt))
    world_from_session, _ = world_from_run(session, np.load(args.session_gt))
    truth = np.linalg.inv(world_from_base) @ world_from_session
    placed, residual = icp(load(args.session_map)[::4], load(args.base_map)[::2], truth)
    print(json.dumps({'label': args.label, 'icp_residual_m': residual}), flush=True)

    searcher = Searcher(args.store, args.queries)
    stored = np.array([f[:3, 3] for f in searcher.frames])
    for k in range(0, len(searcher.stamps), args.every):
        frame, odometry, local_map = (x[k] for x in searcher.inputs)
        pose = placed @ local_map @ np.linalg.inv(odometry) @ frame
        gap = float(np.min(np.linalg.norm(stored[:, :2] - pose[:2, 3], axis=1)))
        if gap > 5.0:
            continue
        seeds = []
        for distance, index, shift in searcher.rank(k):
            start = turned(searcher.frames[index], shift)
            seeds.append([round(distance, 4),
                          round(float(np.linalg.norm(start[:2, 3] - pose[:2, 3])), 2),
                          round(yaw_deg(start, pose), 1)])
        found = searcher.matcher.match_with_reference(searcher.points[k], pose)
        from_truth = None
        if found is not None:
            verified = np.asarray(found[1])
            from_truth = [round(float(np.linalg.norm(verified[:3, 3] - pose[:3, 3])), 2),
                          round(yaw_deg(verified, pose), 2)]
        print(json.dumps({'k': k, 't': round(float(searcher.stamps[k] - searcher.stamps[0]), 2),
                          'gap_m': round(gap, 2), 'seeds': seeds, 'from_truth': from_truth}),
              flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
