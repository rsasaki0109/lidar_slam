"""Run ``python -m lidarslam live ... --relocalize-anywhere`` and save the search input of every keyframe.

  python3 -I record_anywhere_queries.py OUT.npz live BAG ... --store STORE --relocalize-anywhere

The session never anchors: every keyframe's search is recorded (stamp, frame, odometry, local map
and the levelled points that ``match_anywhere`` would get) and skipped. Until a session anchors
these inputs do not depend on the search, so ``gate_study.py`` can replay the search with any
Scan Context gate offline.
"""
import runpy
import sys

import numpy as np

import lidarslam.relocalization as rel

out = sys.argv[1]
sys.argv = ['lidarslam'] + sys.argv[2:]
records = []
rel.ANYWHERE_INTERVAL_SEC = 0.0
observe = rel.StartupRelocalization.observe


def recording_observe(self, stamp, index, frame, odometry, local_map, points, body_points):
    if self.active and self.anywhere and self.prior is None:
        level_from_body = np.linalg.inv(frame) @ odometry
        level = body_points @ level_from_body[:3, :3].T + level_from_body[:3, 3]
        records.append((stamp, index, frame, odometry, local_map, level.astype(np.float32)))
        return
    return observe(self, stamp, index, frame, odometry, local_map, points, body_points)


rel.StartupRelocalization.observe = recording_observe
try:
    runpy.run_module('lidarslam', run_name='__main__')
except SystemExit as error:
    if error.code not in (None, 0):
        raise
finally:
    sizes = np.array([len(r[5]) for r in records], dtype=np.int64)
    np.savez(out, stamp=np.array([r[0] for r in records]), index=np.array([r[1] for r in records]),
             frame=np.array([r[2] for r in records]), odometry=np.array([r[3] for r in records]),
             local_map=np.array([r[4] for r in records]), sizes=sizes,
             points=np.concatenate([r[5] for r in records]) if records else np.zeros((0, 3)))
    print(f'recorded {len(records)} searches', file=sys.stderr)
