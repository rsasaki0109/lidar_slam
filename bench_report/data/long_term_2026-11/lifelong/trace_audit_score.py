"""Score a lifelong store's removals against a session outside the chain (preregistered, roadmap).

  trace_audit_score.py AUDITED_STORE DAY LABEL >> OUT.jsonl

AUDITED_STORE is a copy of the 20-session store with the audit session appended (session 20).
The store's removals are the per-submap removal masks of its 20 sessions
(lidarslam.map_changes.removal_masks). The audit session's evidence decides each stored voxel it
saw: kept (1: hit at least twice and at least as often as seen through), removed (2) or
undecided (0). A removed voxel the audit session keeps is contradicted.
"""

from __future__ import annotations

import json
import sys

import numpy as np


def main() -> int:
    from lidarslam.map_changes import EVIDENCE_FILE, removal_masks
    from lidarslam.store import Store

    store = Store.open(sys.argv[1])
    row = {'day': sys.argv[2], 'store': sys.argv[3]}
    sessions = store.sessions
    if len(sessions) < 21 or not sessions[20].get('map_change_evidence'):
        print(json.dumps({**row, 'skipped': 'audit session not committed with evidence'}))
        return 0
    masks = removal_masks(store, sessions[:20])
    index = {entry['id']: i for i, entry in enumerate(sessions)}
    with np.load(store.session_dir(sessions[20]['id']) / EVIDENCE_FILE,
                 allow_pickle=False) as evidence:
        references = [index[name] for name in evidence['references'].tolist()]
        rows = evidence['rows']
    removed = np.array([(int(x), int(y), int(z)) in masks[references[ref]].get(int(submap), ())
                        for ref, submap, x, y, z in rows[:, :5]], dtype=bool)
    decision = rows[:, 8]
    total_removed = sum(len(voxels) for mask in masks for voxels in mask.values())
    row.update({'store_removed_voxels': int(total_removed),
                'audited_voxels': int(len(rows)),
                'removed_seen': int(removed.sum()),
                'removed_decided': int((removed & (decision != 0)).sum()),
                'removed_contradicted': int((removed & (decision == 1)).sum()),
                'removed_confirmed': int((removed & (decision == 2)).sum()),
                'kept_decided': int((~removed & (decision != 0)).sum()),
                'kept_audit_removes': int((~removed & (decision == 2)).sum()),
                'tilt_correction': sessions[20].get('store_tilt_correction'),
                'height_profile': sessions[20].get('store_height_profile')})
    if row['removed_decided']:
        row['contradicted_share'] = round(row['removed_contradicted'] / row['removed_decided'], 4)
    print(json.dumps(row), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
