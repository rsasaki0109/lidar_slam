"""Score the height profile on the sessions outside the lifelong chain (preregistered, roadmap).

Each update was appended without correction to its own copy of the store rebuilt with the
profile. Both corrections are fitted here, with the code `live` runs, against the store's even
sessions (0, 2, ..., 18) only, and scored against its odd ones (1, 3, ..., 19):

- A: the update as committed;
- T: the tilt correction (`height_check.fit_tilt`);
- P: the tilt correction, then the profile (`height_check.fit_profile`).

  fresh_profile_score.py BENCH_ROOT > OUT.jsonl
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np


def p95(differences):
    return round(float(np.percentile(np.abs(differences), 95)), 3) if len(differences) else None


def main() -> int:
    from lidarslam import height_check
    from lidarslam.height_check import fit_profile, fit_tilt, height_differences, submap_positions
    from lidarslam.store import Store

    root = Path(sys.argv[1])
    windows = json.loads((root / 'fresh_windows.json').read_text())
    for day, window in windows.items():
        row = {'day': day}
        store_dir = root / 'fresh_profile' / day / 'store'
        if window is None or not store_dir.is_dir():
            print(json.dumps({**row, 'skipped': 'no pass through the square' if window is None
                              else 'not run'}))
            continue
        store = Store.open(store_dir)
        if len(store.sessions) < 21 or not store.sessions[20].get('store_from_session'):
            print(json.dumps({**row, 'skipped': 'not anchored or not committed'}))
            continue
        positions = submap_positions(*zip(*store._aligned_sessions()))
        even = np.concatenate(positions[0:20:2])
        odd = np.concatenate(positions[1:20:2])
        update = positions[20]
        tilt = fit_tilt(even, update)
        if tilt is None:
            print(json.dumps({**row, 'skipped': 'too little overlap with the even sessions'}))
            continue
        correction = tilt['correction']
        tilted = update @ correction[:3, :3].T + correction[:3, 3]
        profile = fit_profile(even, tilted)
        profiled = tilted if profile is None else tilted - np.outer(profile['offsets'],
                                                                     [0.0, 0.0, 1.0])
        scores = {name: height_differences(odd, positions)
                  for name, positions in (('A', update), ('T', tilted), ('P', profiled))}
        row.update({'fit_compared': tilt['compared_submaps'], 'roll_deg': tilt['roll_deg'],
                    'pitch_deg': tilt['pitch_deg'], 'offset_m': tilt['offset_m'],
                    'profile_knots': None if profile is None else profile['knots'],
                    'profile_max_abs_offset_m': None if profile is None
                    else profile['max_abs_offset_m'],
                    'score_compared': {k: int(len(v)) for k, v in scores.items()}})
        if all(len(v) >= height_check.MIN_COMPARED for v in scores.values()):
            row.update({f'{k}_p95_m': p95(v) for k, v in scores.items()})
        print(json.dumps(row), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
