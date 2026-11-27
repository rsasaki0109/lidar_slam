"""Out-of-sample check of a tilt and height correction (preregistered, roadmap).

  tilt_check.py STORE CHAIN_JSONL > OUT.jsonl

For each update k: the correction is fitted against sessions 0..k-1 and scored against k+1..
(see docs/roadmap/long-term-mapping-2026-11.md). One JSON line per update.
"""

from __future__ import annotations

import json
import sys

import numpy as np

GRID = np.radians(np.arange(-4.0, 4.0001, 0.25))


def rotate(positions, roll, pitch, about):
    from scipy.spatial.transform import Rotation
    return (positions - about) @ Rotation.from_rotvec([roll, pitch, 0.0]).as_matrix().T + about


def main() -> int:
    from lidarslam.height_check import height_differences, submap_positions
    from lidarslam.store import Store

    aligned = Store.open(sys.argv[1])._aligned_sessions()
    days = ['base'] + [json.loads(line)['day'] for line in open(sys.argv[2])][:len(aligned) - 1]
    positions = submap_positions(*zip(*aligned))
    for k in range(1, len(positions)):
        earlier = np.concatenate(positions[:k])
        update = positions[k]
        before = height_differences(earlier, update)
        row = {'day': days[k], 'fit_compared': int(len(before))}
        if len(before) < 20:
            print(json.dumps({**row, 'skipped': 'too little overlap with earlier sessions'}))
            continue
        best = None
        for roll in GRID:
            for pitch in GRID:
                d = height_differences(earlier, rotate(update, roll, pitch, update[0]))
                if len(d) < 0.8 * len(before):
                    continue
                score = float(np.percentile(np.abs(d - np.median(d)), 95))
                if best is None or score < best[0]:
                    best = (score, roll, pitch, float(np.median(d)))
        _, roll, pitch, offset = best
        corrected = rotate(update, roll, pitch, update[0]) - [0.0, 0.0, offset]
        row.update({'roll_deg': round(float(np.degrees(roll)), 2),
                    'pitch_deg': round(float(np.degrees(pitch)), 2), 'offset_m': round(offset, 3)})
        if k + 1 < len(positions):
            later = np.concatenate(positions[k + 1:])
            a = height_differences(later, update)
            c = height_differences(later, corrected)
            if len(a) >= 20 and len(c) >= 20:
                row.update({'score_compared': int(len(a)),
                            'A_p95_m': round(float(np.percentile(np.abs(a), 95)), 3),
                            'C_p95_m': round(float(np.percentile(np.abs(c), 95)), 3)})
        print(json.dumps(row), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
