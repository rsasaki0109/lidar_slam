"""Score gate_study.py output against NCLT ground truth (one JSON line per session and gate).

  gate_analysis.py ROOT NAME BASE_RUN BASE_DAY DAY [NAME BASE_RUN BASE_DAY DAY ...]

ROOT is the NCLT bench directory (gate/sim_NAME.jsonl, gate/run_NAME/odometry_tum.txt,
runs/BASE_RUN/odometry_tum.txt, gt/DAY.npy). Every candidate session transform (each verified
candidate of a search and the anchor) is placed against the ground-truth placement ``T_g`` (RMS
over the session's odometry path): ``near`` when within 4.0 m (the anchor check's tolerance),
otherwise wrong. A search is ``in_store`` when ``T_g`` puts its keyframe within 5 m of the base
session's path.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from evaluate_relocalize_anywhere import rms_apart, world_from_run  # noqa: E402


def main() -> int:
    root = Path(sys.argv[1])
    args = sys.argv[2:]
    for k in range(0, len(args), 4):
        name, base_run, base_day, day = args[k:k + 4]
        base = np.loadtxt(root / 'runs' / base_run / 'odometry_tum.txt', ndmin=2)
        session = np.loadtxt(root / 'gate' / f'run_{name}' / 'odometry_tum.txt', ndmin=2)
        world_from_base, _ = world_from_run(base, np.load(root / 'gt' / f'{base_day}.npy'))
        world_from_session, _ = world_from_run(session, np.load(root / 'gt' / f'{day}.npy'))
        truth = np.linalg.inv(world_from_base) @ world_from_session
        path = session[:, 1:4]
        stored = base[:, 1:3]
        placed = path @ truth[:3, :3].T + truth[:3, 3]
        gap = np.array([np.min(np.linalg.norm(stored - p[:2], axis=1)) for p in placed])
        stamps = session[:, 0]
        with open(root / 'gate' / f'sim_{name}.jsonl', encoding='utf-8') as handle:
            rows = [json.loads(line) for line in handle]
        first_in = next((float(s) for s, g in zip(stamps, gap) if g <= 5.0), None)
        log = (root / 'gate' / f'log_{name}.txt').read_text(encoding='utf-8')
        start = float(re.search(r'"store_start_stamp_sec": ([0-9.]+)', log).group(1))
        for row in rows:
            searches = []
            for search in row['log']:
                i = int(np.argmin(np.abs(stamps - search['stamp'])))
                offsets = [rms_apart(np.asarray(v['session']), truth, path)
                           for v in search['verified']]
                searches.append({'t': round(search['stamp'] - start, 2),
                                 'in_store': bool(gap[i] <= 5.0),
                                 'best_d': search['best_d'], 'match': search['match'],
                                 'verified': [[v['d'], round(o, 2)] for v, o in
                                              zip(search['verified'], offsets)]})
            anchor = row['store_from_session']
            offset = None if anchor is None else rms_apart(np.asarray(anchor), truth, path)
            print(json.dumps({
                'name': name, 'gate': row['gate'], 'anchor_after_s': row['anchor_after_s'],
                'anchor_offset_m': None if offset is None else round(offset, 2),
                'searches': row['searches'], 'found': row['found'],
                'first_in_store_after_s': (None if first_in is None
                                           else round(first_in - start, 2)),
                'in_store_fraction': round(float(np.mean(gap <= 5.0)), 3),
                'search_log': searches}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
