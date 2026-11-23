"""Replay ``--relocalize-anywhere`` searches offline with other Scan Context gates.

  gate_study.py --store STORE --queries Q.npz --gates 0.18 0.20 ... [--start-stamp T]
      [--ranks RANKS.jsonl] [--label NAME]

The search inputs saved by ``record_anywhere_queries.py`` are searched again, for each gate, as
``StoreRelocalizer.match_anywhere`` does: the eight closest Scan Context candidates up to the gate
are registered (same start pose from the stored keyframe and sector shift, same verification
through ``match_with_reference``) and the match is refused when two verified ones disagree by
more than 0.5 m or 5 degrees. The pacing and two-search agreement of ``StartupRelocalization``
(searches in line) decide which keyframes are searched and when the session anchors. Only the
searches a gate reaches are registered, and each candidate once for all gates. One JSON line
per gate: when the session would anchor, with which session transform, and every verified
candidate of its searches. ``--ranks`` also writes the eight closest distances of every
keyframe.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

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


class Searcher:
    """Registers candidates on demand and remembers them for every gate."""

    def __init__(self, store: str, queries: str):
        from pathlib import Path

        from lidarslam import _core
        from lidarslam.relocalization import tracked_poses
        from lidarslam.store import Store

        directories, transforms = zip(*Store.open(store)._aligned_sessions())
        windows = []
        for directory in directories:
            path = Path(directory) / 'keyframe_windows.npz'
            if path.is_file():
                with np.load(path, allow_pickle=False) as data:
                    windows.append({int(index): data[index] for index in data.files})
            else:
                windows.append({})
        self.matcher = _core.StoreRelocalizer(directories, transforms, windows,
                                              [tracked_poses(path) for path in directories])
        self.frames = [gravity_aligned(np.asarray(p)) for poses in self.matcher.session_poses()
                       for p in poses]
        data = np.load(queries, allow_pickle=False)
        self.stamps = data['stamp']
        self.inputs = [data[name] for name in ('frame', 'odometry', 'local_map')]
        sizes, points = data['sizes'], data['points']  # each access of an npz member reads it
        ends = np.cumsum(sizes)
        self.points = [points[e - n:e] for e, n in zip(ends, sizes)]
        self.ranks = [None] * len(self.stamps)
        self.rank_seconds = [0.0] * len(self.stamps)
        self.cache = {}
        self.seconds = {}

    def rank(self, k: int):
        if self.ranks[k] is None:
            began = time.perf_counter()
            self.ranks[k] = [(float(d), int(i), int(s))
                             for d, i, s in self.matcher.rank_anywhere(self.points[k], 8)]
            self.rank_seconds[k] = time.perf_counter() - began
        return self.ranks[k]

    def register(self, k: int, seed: int):
        key = (k, seed)
        if key not in self.cache:
            _, index, shift = self.rank(k)[seed]
            began = time.perf_counter()
            found = self.matcher.match_with_reference(self.points[k],
                                                      turned(self.frames[index], shift))
            self.seconds[key] = time.perf_counter() - began
            if found is None:
                self.cache[key] = None
            else:
                pose = np.asarray(found[1])
                frame, odometry, local_map = (x[k] for x in self.inputs)
                session = pose @ np.linalg.inv(frame) @ odometry @ np.linalg.inv(local_map)
                self.cache[key] = (pose, session)
        return self.cache[key]

    def consensus(self, k: int, gate: float, cap: int = 8):
        """matchAnywhere at this gate, registering at most ``cap`` candidates: the first
        verified candidate (None on a disagreement), the verified ones, and the search's
        measured seconds (ranking plus registrations)."""
        ranked = self.rank(k)
        seconds = self.rank_seconds[k]
        if not ranked or ranked[0][0] > gate:
            return None, [], seconds
        found, verified = None, []
        for seed, (distance, _, _) in enumerate(ranked[:cap]):
            if distance > gate:
                break
            result = self.register(k, seed)
            seconds += self.seconds.get((k, seed), 0.0)
            if result is None:
                continue
            verified.append((seed, result))
            if found is not None and not agrees(found[0], result[0], 0.5, 5.0):
                return None, verified, seconds
            if found is None:
                found = result
        return found, verified, seconds

    def simulate(self, gate: float, cap: int = 8, realtime: bool = False, slowdown: float = 1.0,
                 interval: float = 2.0, maximum: float = 8.0):
        """StartupRelocalization at this gate. In line, or with ``realtime`` as the background
        search of live --realtime: a search started at a keyframe ends ``slowdown`` times its
        measured seconds later in data time; keyframes before that are not searched, and its
        result is taken at the first keyframe after it ends."""
        last, wait, candidate, pending = -np.inf, interval, None, None
        log = []
        for k, stamp in enumerate(self.stamps):
            if pending is not None:
                if stamp < pending[0]:
                    continue
                _, match = pending
                pending = None
            else:
                if stamp - last < wait:
                    continue
                last = stamp
                match, verified, seconds = self.consensus(k, gate, cap)
                log.append({'k': k, 'stamp': float(stamp), 'best_d': self.rank(k)[0][0],
                            'match': match is not None, 'seconds': round(seconds, 3),
                            'verified': [{'seed': s, 'd': self.rank(k)[s][0],
                                          'session': r[1].tolist()} for s, r in verified]})
                if realtime:
                    pending = (stamp + slowdown * seconds, (match, self.rank(k)[0][0]))
                    continue
                match = (match, self.rank(k)[0][0])
            match, best = match
            if match is not None:
                wait = interval
                if candidate is None or not agrees(candidate, match[1], 0.5, 2.0):
                    candidate = match[1]
                    continue
                return float(stamp), match[1], log
            if best <= gate:
                wait = min(2 * wait, maximum)
        return None, None, log


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--store', required=True)
    parser.add_argument('--queries', required=True)
    parser.add_argument('--gates', type=float, nargs='+', required=True)
    parser.add_argument('--caps', type=int, nargs='+', default=[8])
    parser.add_argument('--realtime', action='store_true')
    parser.add_argument('--slowdown', type=float, default=1.0)
    parser.add_argument('--start-stamp', type=float, default=None)
    parser.add_argument('--ranks', default=None)
    parser.add_argument('--label', default='')
    args = parser.parse_args()
    searcher = Searcher(args.store, args.queries)
    start = args.start_stamp if args.start_stamp is not None else float(searcher.stamps[0])
    for gate, cap in ((g, c) for g in args.gates for c in args.caps):
        stamp, transform, log = searcher.simulate(gate, cap, args.realtime, args.slowdown)
        print(json.dumps({
            'label': args.label, 'gate': gate, 'cap': cap, 'realtime': args.realtime,
            'slowdown': args.slowdown, 'searches': len(log),
            'found': sum(row['match'] for row in log),
            'anchor_after_s': None if stamp is None else round(stamp - start, 2),
            'store_from_session': None if transform is None else transform.tolist(),
            'log': log}), flush=True)
    if args.ranks:
        with open(args.ranks, 'w', encoding='utf-8') as handle:
            for k, stamp in enumerate(searcher.stamps):
                handle.write(json.dumps({'stamp': float(stamp), 'rank': searcher.rank(k)}) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
