"""Score the tilt and height correction on the sessions outside the lifelong chain
(preregistered, roadmap): fit against the store's even sessions, score against its odd ones.

  fresh_tilt_score.py BENCH_ROOT > OUT.jsonl
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tilt_check import GRID, rotate  # noqa: E402


def main() -> int:
    from lidarslam.height_check import height_differences, submap_positions
    from lidarslam.store import Store

    root = Path(sys.argv[1])
    windows = json.loads((root / 'fresh_windows.json').read_text())
    for day, window in windows.items():
        row = {'day': day}
        store_dir = root / 'fresh' / day / 'store'
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
        before = height_differences(even, update)
        row['fit_compared'] = int(len(before))
        if len(before) < 20:
            print(json.dumps({**row, 'skipped': 'too little overlap with the even sessions'}))
            continue
        best = None
        for roll in GRID:
            for pitch in GRID:
                d = height_differences(even, rotate(update, roll, pitch, update[0]))
                if len(d) < 0.8 * len(before):
                    continue
                score = float(np.percentile(np.abs(d - np.median(d)), 95))
                if best is None or score < best[0]:
                    best = (score, roll, pitch, float(np.median(d)))
        _, roll, pitch, offset = best
        corrected = rotate(update, roll, pitch, update[0]) - [0.0, 0.0, offset]
        a = height_differences(odd, update)
        c = height_differences(odd, corrected)
        row.update({'roll_deg': round(float(np.degrees(roll)), 2),
                    'pitch_deg': round(float(np.degrees(pitch)), 2), 'offset_m': round(offset, 3),
                    'score_compared': int(len(a))})
        if len(a) >= 20 and len(c) >= 20:
            row.update({'A_p95_m': round(float(np.percentile(np.abs(a), 95)), 3),
                        'C_p95_m': round(float(np.percentile(np.abs(c), 95)), 3)})
        print(json.dumps(row), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
