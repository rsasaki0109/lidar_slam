"""Why a session outside the chain contradicts the removals of a store (development, roadmap).

  probe_session.py STORE PROBE_DIR > OUT.json

STORE is the 20-session store (stores/lifelong_trace, E); PROBE_DIR is what probe_session_run.sh
kept of the session appended to a copy of it (session/, session_entry.json). Reports:

- ``height``: the session's submap heights against the store's submap positions within 1.5 m
  (XY, at least three), as the height check compares them, for every compared submap;
- ``removed``: the store's removed voxels the session decided, contradicted or confirmed, split
  by the session whose decision removed them last and by the number of misses of that decision;
- ``hits_on_contradicted``: how often the session hit the removed voxels it keeps.
No ground truth is read.
"""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np


def main() -> int:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from removal_breakdown import replay
    from lidarslam.height_check import HeightReference, submap_poses
    from lidarslam.store import Store

    store = Store.open(sys.argv[1])
    probe = Path(sys.argv[2])
    sessions = store.sessions[:20]
    entry = json.loads((probe / 'session_entry.json').read_text())
    directories = [store.session_dir(e['id']) for e in sessions]
    transforms = [np.asarray(e.get('store_from_session', np.eye(4)), dtype=float)
                  for e in sessions]
    reference = np.concatenate([p[:, :3, 3] for p in submap_poses(directories, transforms)])
    update = submap_poses([probe / 'session'], [np.asarray(entry['store_from_session'])])[0]
    dz = HeightReference(reference).differences(update[:, :3, 3])
    dz = [float(d) for d in dz if d is not None and np.isfinite(d)]
    out = {'submaps': int(len(update)),
           'height': {'compared': len(dz),
                      'median_dz_m': round(float(np.median(dz)), 3) if dz else None,
                      'dz_m': [round(d, 3) for d in dz]}}

    removed = replay(store, sessions)
    index = {e['id']: i for i, e in enumerate(sessions)}
    with np.load(probe / 'session' / 'map_change_evidence.npz', allow_pickle=False) as data:
        references = np.array([index.get(name, -1) for name in data['references'].tolist()])
        rows = data['rows']
    rows = rows[rows[:, 8] != 0]
    by_remover, by_misses, hits = Counter(), Counter(), Counter()
    for row in rows.tolist():
        hit = removed.get((int(references[row[0]]), row[1], row[2], row[3], row[4]))
        if hit is None:
            continue
        verdict = 'contradicted' if row[8] == 1 else 'confirmed'
        by_remover[(sessions[hit[0]]['id'], verdict)] += 1
        by_misses[(min(hit[2], 16) if hit[1] >= 0 else 'new_point', verdict)] += 1
        if verdict == 'contradicted':
            hits[min(row[5], 10)] += 1

    def table(counter):
        keys = sorted({k for k, _ in counter}, key=str)
        return {str(k): {'contradicted': counter[(k, 'contradicted')],
                         'confirmed': counter[(k, 'confirmed')]} for k in keys}

    out['removed'] = {'by_remover': table(by_remover), 'by_remover_misses': table(by_misses)}
    out['hits_on_contradicted'] = {str(k): v for k, v in sorted(hits.items())}
    print(json.dumps(out, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
