"""Time and compare variants of the ``match_anywhere`` registration in Python (small_gicp).

  registration_bench.py --store STORE --queries Q.npz [--keyframes 100:140] [--seeds 2]
      [--variant NAME ...] > OUT.jsonl

``StartupRelocalizer::matchPrepared`` written again with the small_gicp Python bindings: the
query is thinned (0.2 m) and prepared (0.25 m voxels, 10-neighbour covariances); the up to three
stored keyframes within 3 m of the start are each aligned by GICP over the correspondence
distances 2.0, 1.0 and 0.5 m (30 iterations each) and kept when at least 60% of the query lies
within 0.3 m; the match needs two that agree (0.5 m, 5 degrees) or a confirmation from a
keyframe near the result. ``--variant`` changes one part of it (see VARIANTS). For every
keyframe and each of its closest Scan Context candidates (``--seeds``) the base and every
variant run from the same start, and one JSON line gives each one's seconds, result and the
distance of its pose from the C++ result (``match_with_reference``).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import small_gicp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gate_study import Searcher, agrees, turned  # noqa: E402

BASE = {'voxel': 0.25, 'distances': (2.0, 1.0, 0.5), 'iterations': 30, 'neighbours': 3,
        'coarse_voxel': None, 'threads': 1}
VARIANTS = {
    'base': {},
    'iter10': {'iterations': 10},
    'coarse1': {'coarse_voxel': 1.0},
    'coarse05': {'coarse_voxel': 0.5},
    'neigh2': {'neighbours': 2},
    'threads4': {'threads': 4},
}


class Matcher:
    def __init__(self, searcher: Searcher, store: str):
        from lidarslam.store import Store

        self.frames = searcher.frames
        poses = [np.asarray(p) for s in searcher.matcher.session_poses() for p in s]
        self.points = []
        for (directory, _), session in zip(Store.open(store)._aligned_sessions(),
                                           searcher.matcher.session_poses()):
            with np.load(Path(directory) / 'keyframe_windows.npz', allow_pickle=False) as data:
                windows = {int(k): data[k] for k in data.files}
            for j in range(len(session)):
                self.points.append(windows.get(j))
        self.local = [np.linalg.inv(f) @ p for f, p in zip(self.frames, poses)]
        self.prepared = {}

    def target(self, index: int, voxel: float):
        key = (index, voxel)
        if key not in self.prepared:
            pts = self.points[index].astype(np.float64)
            pts = pts @ self.local[index][:3, :3].T + self.local[index][:3, 3]
            pts = pts[np.einsum('ij,ij->i', pts, pts) < 1600.0]
            thin = small_gicp.voxelgrid_sampling(pts, 0.2)
            self.prepared[key] = small_gicp.preprocess_points(thin.points()[:, :3], voxel, 10, 1)
        return self.prepared[key]

    @staticmethod
    def source(points: np.ndarray, voxel: float):
        thin = small_gicp.voxelgrid_sampling(points.astype(np.float64), 0.2)
        return small_gicp.preprocess_points(thin.points()[:, :3], voxel, 10, 1)

    def neighbours(self, position: np.ndarray, count: int):
        d = [(float(np.sum((f[:3, 3] - position) ** 2)), i) for i, f in enumerate(self.frames)
             if self.points[i] is not None]
        return [i for sq, i in sorted(d) if sq <= 9.0][:count]

    def align(self, index, sources, guess, cfg):
        relative = np.linalg.inv(self.frames[index]) @ guess
        for stage, distance in enumerate(cfg['distances']):
            coarse = cfg['coarse_voxel'] is not None and stage == 0
            voxel = cfg['coarse_voxel'] if coarse else cfg['voxel']
            target, tree = self.target(index, voxel)
            result = small_gicp.align(target, sources[voxel][0], tree, relative,
                                      max_correspondence_distance=distance,
                                      num_threads=cfg['threads'],
                                      max_iterations=cfg['iterations'])
            relative = result.T_target_source
        target, tree = self.target(index, cfg['voxel'])
        src = sources[cfg['voxel']][0].points()
        moved = src @ relative.T
        _, sq = tree.batch_nearest_neighbor_search(moved[:, :3], 1)
        overlap = float(np.mean(np.asarray(sq) < 0.09)) if len(src) else 0.0
        return self.frames[index] @ relative, overlap

    def match(self, points, prior, cfg):
        voxels = {cfg['voxel']} | ({cfg['coarse_voxel']} if cfg['coarse_voxel'] else set())
        sources = {v: self.source(points, v) for v in voxels}
        cands = []
        for i in self.neighbours(prior[:3, 3], cfg['neighbours']):
            pose, overlap = self.align(i, sources, prior, cfg)
            if overlap >= 0.6:
                cands.append((i, pose, overlap))
        cands.sort(key=lambda c: (-c[2], c[0]))
        best = next((c for c in cands if agrees(prior, c[1], 2.0, 20.0)), None)
        if best is None:
            return None
        support = 0
        for c in cands:
            if agrees(best[1], c[1], 0.5, 5.0):
                support += 1
            elif c[2] >= best[2] - 0.05 and agrees(prior, c[1], 2.0, 20.0):
                return None
        if support >= 2:
            return best[1]
        for i in self.neighbours(best[1][:3, 3], cfg['neighbours']):
            if i == best[0]:
                continue
            pose, overlap = self.align(i, sources, best[1], cfg)
            if overlap >= 0.6 and agrees(best[1], pose, 0.5, 5.0):
                return best[1]
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--store', required=True)
    parser.add_argument('--queries', required=True)
    parser.add_argument('--keyframes', default='0:-1')
    parser.add_argument('--every', type=int, default=1)
    parser.add_argument('--seeds', type=int, default=2)
    parser.add_argument('--variant', nargs='+', default=list(VARIANTS))
    args = parser.parse_args()
    searcher = Searcher(args.store, args.queries)
    matcher = Matcher(searcher, args.store)
    lo, hi = (int(v) for v in args.keyframes.split(':'))
    for k in range(len(searcher.stamps))[lo:hi:args.every]:
        for seed, (distance, index, shift) in enumerate(searcher.rank(k)[:args.seeds]):
            start = turned(searcher.frames[index], shift)
            began = time.perf_counter()
            cpp = searcher.matcher.match_with_reference(searcher.points[k], start)
            row = {'k': k, 'seed': seed, 'd': round(distance, 4),
                   'cpp': {'s': round(time.perf_counter() - began, 3), 'ok': cpp is not None}}
            cpp_pose = None if cpp is None else np.asarray(cpp[1])
            for name in args.variant:
                cfg = {**BASE, **VARIANTS[name]}
                began = time.perf_counter()
                pose = matcher.match(searcher.points[k], start, cfg)
                entry = {'s': round(time.perf_counter() - began, 3), 'ok': pose is not None}
                if pose is not None and cpp_pose is not None:
                    entry['from_cpp_m'] = round(float(np.linalg.norm(pose[:3, 3] -
                                                                     cpp_pose[:3, 3])), 3)
                row[name] = entry
            print(json.dumps(row), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
