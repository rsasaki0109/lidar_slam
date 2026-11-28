"""Where the removals of a lifelong store are contradicted (development, roadmap).

  removal_breakdown.py STORE EVIDENCE.npz [EVIDENCE.npz ...] > OUT.json

STORE is the 20-session store (stores/lifelong_trace, E). Each EVIDENCE file is the change
evidence of a session outside the chain appended to a copy of STORE (removal_breakdown_run.sh).
As in trace_audit_score.py, a voxel the store has removed that the appended session decided is
contradicted when that session keeps it. The removed voxels it decided are split by:

- ``height``: the voxel centre's height above the median height of the store's submap
  positions (the sensor) within 10 m (XY); the NCLT Velodyne is about 1.1 m above the ground;
- ``range``: the XY distance from the voxel to the nearest submap position of the store;
- ``remover_misses_per_hit``: misses over max(1, hits) of the session whose decision removed it
  last (at least 8 by the removal rule), or ``new_point`` for the new-point suppression mask;
- ``remover_hits``: the hits in that decision;
- ``age_days``: days between the session that mapped the voxel and the one that removed it.
No ground truth is read.
"""

from __future__ import annotations

from collections import defaultdict
import json
import sys

import numpy as np
from scipy.spatial import cKDTree

VOXEL_M = 0.15
CELL_M = 2.0  # XY cells that share one sensor height
BINS = {
    'height': ([-0.8, 0.5, 3.0], ['< -0.8 m', '-0.8 to 0.5 m', '0.5 to 3 m', '> 3 m']),
    'range': ([5.0, 15.0, 30.0], ['< 5 m', '5 to 15 m', '15 to 30 m', '> 30 m']),
    'remover_misses_per_hit': ([16.0, 32.0, 64.0], ['8 to 16', '16 to 32', '32 to 64', '> 64']),
    'remover_hits': ([0.5, 1.5, 3.5], ['0', '1', '2 to 3', '> 3']),
    'age_days': ([30.0, 120.0, 365.0], ['< 30', '30 to 120', '120 to 365', '> 365']),
}


def replay(store, sessions):
    """The voxels removed after the decisions of ``sessions``, with the last removing decision:
    (owner, submap, x, y, z) -> (remover, hits, misses), hits -1 for the new-point mask."""
    from lidarslam.map_changes import EVIDENCE_FILE

    index = {entry['id']: i for i, entry in enumerate(sessions)}
    removed = {}
    for i, entry in enumerate(sessions):
        if not entry.get('map_change_evidence'):
            continue
        with np.load(store.session_dir(entry['id']) / EVIDENCE_FILE, allow_pickle=False) as data:
            references = np.array([index[name] for name in data['references'].tolist()])
            rows = data['rows']
            new_rows = data['new_rows'] if 'new_rows' in data else None
        if new_rows is not None:
            for submap, x, y, z in new_rows.tolist():
                removed[(i, submap, x, y, z)] = (i, -1, 0)
        for ref, submap, x, y, z, hits, misses, _, decision in rows[rows[:, 8] != 0].tolist():
            key = (int(references[ref]), submap, x, y, z)
            if decision == 2:
                removed[key] = (i, hits, misses)
            else:
                removed.pop(key, None)
    return removed


def main() -> int:
    from lidarslam.height_check import submap_poses
    from lidarslam.map_changes import EVIDENCE_FILE  # noqa: F401
    from lidarslam.store import Store

    store = Store.open(sys.argv[1])
    sessions = store.sessions[:20]
    directories = [store.session_dir(entry['id']) for entry in sessions]
    transforms = [np.asarray(entry.get('store_from_session', np.eye(4)), dtype=float)
                  for entry in sessions]
    poses = submap_poses(directories, transforms)
    positions = np.concatenate([p[:, :3, 3] for p in poses])
    tree = cKDTree(positions[:, :2])
    days = np.array([entry['first_stamp_sec'] / 86400.0 for entry in sessions])
    removed = replay(store, sessions)
    index = {entry['id']: i for i, entry in enumerate(sessions)}

    out = {'store_removed_voxels': len(removed), 'audits': {}}
    pooled = defaultdict(lambda: [0, 0])
    for path in sys.argv[2:]:
        with np.load(path, allow_pickle=False) as data:
            references = np.array([index.get(name, -1) for name in data['references'].tolist()])
            rows = data['rows']
        rows = rows[rows[:, 8] != 0]
        owners = references[rows[:, 0]]
        picked, features = [], []
        for row, owner in zip(rows.tolist(), owners.tolist()):
            hit = removed.get((owner, row[1], row[2], row[3], row[4]))
            if hit is not None:
                picked.append((owner, row[1], row[2], row[3], row[4], row[8]))
                features.append(hit)
        if not picked:
            continue
        picked = np.array(picked)
        features = np.array(features)
        local = np.c_[(picked[:, 2:5] + 0.5) * VOXEL_M, np.ones(len(picked))]
        offsets = np.cumsum([0] + [len(p) for p in poses])
        flat = np.concatenate(poses)[offsets[picked[:, 0]] + picked[:, 1]]
        world = np.einsum('nij,nj->ni', flat, local)[:, :3]
        distance, _ = tree.query(world[:, :2])
        cells = np.floor(world[:, :2] / CELL_M).astype(np.int64)
        unique, inverse = np.unique(cells, axis=0, return_inverse=True)
        ground = np.array([np.median(positions[n, 2]) if n else np.nan for n in
                           tree.query_ball_point((unique + 0.5) * CELL_M, 10.0)])
        height = world[:, 2] - ground[inverse.ravel()]
        remover, hits, misses = features[:, 0], features[:, 1], features[:, 2]
        values = {
            'height': height,
            'range': distance,
            'remover_misses_per_hit': np.where(hits < 0, np.nan, misses / np.maximum(1, hits)),
            'remover_hits': np.where(hits < 0, np.nan, hits),
            'age_days': days[remover] - days[picked[:, 0]],
        }
        contradicted = picked[:, 5] == 1
        audit = {'removed_decided': int(len(picked)),
                 'removed_contradicted': int(contradicted.sum()),
                 'new_point_mask': int((hits < 0).sum())}
        for name, (edges, labels) in BINS.items():
            value = values[name]
            known = ~np.isnan(value)
            which = np.digitize(value[known], edges)
            split = {}
            for b, label in enumerate(labels):
                sel = which == b
                n, c = int(sel.sum()), int(contradicted[known][sel].sum())
                split[label] = [n, c]
                pooled[(name, label)][0] += n
                pooled[(name, label)][1] += c
            audit[name] = split
        out['audits'][path] = audit
    out['pooled'] = {}
    for (name, label), (n, c) in pooled.items():
        out['pooled'].setdefault(name, {})[label] = {
            'decided': n, 'contradicted': c, 'share': round(c / n, 4) if n else None}
    print(json.dumps(out, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
