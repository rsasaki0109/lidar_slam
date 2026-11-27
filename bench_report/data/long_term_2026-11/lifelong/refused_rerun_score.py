"""Score the refused sessions appended again with the tilt correction (preregistered, roadmap).

  refused_rerun_score.py BENCH > OUT.jsonl

For each update: whether it was committed, its height check and correction as ``live`` reported
them, and the 95th percentile of |dz| of its submaps against the later lifelong sessions
(k+1..19), placed by the anchor transform (A) and by the corrected one (C).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

UPDATES = (('2012-03-31', 7), ('2012-11-16', 16))


def main() -> int:
    from lidarslam.height_check import height_differences, submap_positions
    from lidarslam.store import Store

    bench = Path(sys.argv[1])
    later = Store.open(bench / 'stores/lifelong')._aligned_sessions()
    for day, k in UPDATES:
        root = bench / 'refused' / day
        log = (root / 'live.log').read_text()
        summary = (json.JSONDecoder().raw_decode(log[log.rindex('\n{\n') + 1:])[0]
                   if '\n{\n' in log else {})
        row = {'day': day, 'committed': 'store_session' in summary,
               'store_refused': summary.get('store_refused'),
               'height_check': summary.get('store_height_check'),
               'tilt_correction': summary.get('store_tilt_correction')}
        entries = Store.open(root / 'store').sessions
        if row['committed']:
            entry = entries[-1]
            directory = str(root / 'store/sessions' / entry['id'])
            transforms = {'A': entry.get('store_from_session_uncorrected',
                                         entry['store_from_session']),
                          'C': entry['store_from_session']}
            for arm, transform in transforms.items():
                positions = submap_positions([p for p, _ in later[k + 1:]] + [directory],
                                             [t for _, t in later[k + 1:]] + [transform])
                d = height_differences(np.concatenate(positions[:-1]), positions[-1])
                row[f'later_compared_{arm}'] = int(len(d))
                row[f'later_p95_{arm}_m'] = (round(float(np.percentile(np.abs(d), 95)), 3)
                                             if len(d) else None)
        print(json.dumps(row), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
