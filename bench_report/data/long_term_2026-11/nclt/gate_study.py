"""Replay ``--relocalize-anywhere`` searches offline with other Scan Context gates.

  gate_study.py register --store STORE --queries Q.npz --max-gate 0.30 > SEEDS.jsonl
  gate_study.py simulate --seeds SEEDS.jsonl --gates 0.18 0.20 ... [--start-stamp T]

``register`` takes the search inputs saved by ``record_anywhere_queries.py`` and, for every
keyframe, registers each of the eight closest Scan Context candidates whose distance is at most
``--max-gate``, as ``StoreRelocalizer.match_anywhere`` does (same start pose from the stored
keyframe and sector shift, same verification through ``match_with_reference``). ``simulate``
then applies, for each gate, the consensus of ``matchAnywhere`` (candidates in order up to the
gate; refused when two verified ones disagree by more than 0.5 m or 5 degrees) and the pacing
and two-search agreement of ``StartupRelocalization`` (searches in line), and prints one JSON
line per gate: when the session would anchor and with which session transform.
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

SECTORS = 60


def gravity_aligned(pose: np.ndarray) -> np.ndarray:
    frame = np.eye(4)
    yaw = np.arctan2(pose[1, 0], pose[0, 0])
    frame[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    frame[:3, 3] = pose[:3, 3]
    return frame


def turned(frame: np.ndarray, shift: int) -> np.ndarray:
    angle = shift * 2.0 * np.pi / SECTORS
    turn = np.eye(4)
    turn[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    return frame @ turn


def agrees(a: np.ndarray, b: np.ndarray, meters: float, degrees: float) -> bool:
    yaw = np.arctan2(a[1, 0], a[0, 0]) - np.arctan2(b[1, 0], b[0, 0])
    return bool(np.linalg.norm(a[:3, 3] - b[:3, 3]) <= meters and
                abs(np.arctan2(np.sin(yaw), np.cos(yaw))) <= np.deg2rad(degrees))


def register(args: argparse.Namespace) -> int:
    from lidarslam import _core
    from lidarslam.relocalization import tracked_poses
    from lidarslam.store import Store
    from pathlib import Path

    aligned = Store.open(args.store)._aligned_sessions()
    directories, transforms = zip(*aligned)
    windows = []
    for directory in directories:
        path = Path(directory) / 'keyframe_windows.npz'
        if path.is_file():
            with np.load(path, allow_pickle=False) as data:
                windows.append({int(index): data[index] for index in data.files})
        else:
            windows.append({})
    matcher = _core.StoreRelocalizer(directories, transforms, windows,
                                     [tracked_poses(path) for path in directories])
    frames = [gravity_aligned(np.asarray(p)) for poses in matcher.session_poses() for p in poses]
    queries = np.load(args.queries, allow_pickle=False)
    ends = np.cumsum(queries['sizes'])
    for k, stamp in enumerate(queries['stamp']):
        points = queries['points'][ends[k] - queries['sizes'][k]:ends[k]]
        frame, odometry, local_map = (queries[name][k] for name in ('frame', 'odometry',
                                                                    'local_map'))
        seeds = []
        for distance, index, shift in matcher.rank_anywhere(points, 8):
            seed = {'d': round(float(distance), 5), 'i': int(index), 'shift': int(shift)}
            if distance <= args.max_gate:
                found = matcher.match_with_reference(points, turned(frames[index], shift))
                if found is not None:
                    reference, pose = found
                    pose = np.asarray(pose)
                    seed['pose'] = pose.tolist()
                    seed['session'] = (pose @ np.linalg.inv(frame) @ odometry @
                                       np.linalg.inv(local_map)).tolist()
                else:
                    seed['pose'] = None
            seeds.append(seed)
        print(json.dumps({'stamp': float(stamp), 'seeds': seeds}), flush=True)
    return 0


def consensus(seeds: list[dict], gate: float):
    """matchAnywhere at this gate: the first verified candidate, None on a disagreement."""
    if not seeds or seeds[0]['d'] > gate:
        return None
    found = None
    for seed in seeds:
        if seed['d'] > gate:
            break
        if 'pose' not in seed:
            raise ValueError(f'gate {gate} above the --max-gate of the registration')
        if seed['pose'] is None:
            continue
        if found is not None and not agrees(np.asarray(found['pose']), np.asarray(seed['pose']),
                                            0.5, 5.0):
            return None
        if found is None:
            found = seed
    return found


def simulate(rows: list[dict], gate: float, interval: float = 2.0, maximum: float = 8.0):
    """StartupRelocalization in line: (anchor stamp, transform, searches, found, refused)."""
    last, wait, candidate = -np.inf, interval, None
    searches = found_count = 0
    for row in rows:
        if row['stamp'] - last < wait:
            continue
        last = row['stamp']
        searches += 1
        match = consensus(row['seeds'], gate)
        if match is not None:
            wait = interval
            found_count += 1
            transform = np.asarray(match['session'])
            if candidate is None or not agrees(candidate, transform, 0.5, 2.0):
                candidate = transform
                continue
            return row['stamp'], transform, searches, found_count
        if row['seeds'] and row['seeds'][0]['d'] <= gate:
            wait = min(2 * wait, maximum)
    return None, None, searches, found_count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    reg = sub.add_parser('register')
    reg.add_argument('--store', required=True)
    reg.add_argument('--queries', required=True)
    reg.add_argument('--max-gate', type=float, default=0.30)
    sim = sub.add_parser('simulate')
    sim.add_argument('--seeds', required=True)
    sim.add_argument('--gates', type=float, nargs='+', required=True)
    sim.add_argument('--start-stamp', type=float, default=None)
    sim.add_argument('--label', default='')
    args = parser.parse_args()
    if args.command == 'register':
        return register(args)
    with open(args.seeds, encoding='utf-8') as handle:
        rows = [json.loads(line) for line in handle]
    start = args.start_stamp if args.start_stamp is not None else (rows[0]['stamp'] if rows
                                                                   else 0.0)
    for gate in args.gates:
        stamp, transform, searches, found = simulate(rows, gate)
        print(json.dumps({
            'label': args.label, 'gate': gate, 'searches': searches, 'found': found,
            'anchor_after_s': None if stamp is None else round(stamp - start, 2),
            'store_from_session': None if transform is None else transform.tolist()}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
