#!/usr/bin/env python3
"""Score one long-term map update on NCLT against two reference sessions mapped alone.

NCLT (scripts/convert_nclt_session.py) has no scan of the true scene, so the references are two
other sessions recorded close in time to the update, each mapped alone. Everything is compared in
the store frame:

- Each reference map is placed in the store frame by its ground-truth fit (``--ref-init``,
  a 4x4 JSON matrix: reference map frame -> store frame) and then refined by point-to-plane ICP
  against the update's export, on points above the ground (``--align-to before``: the map before
  the update; ``--align-target MAP``: one map shared by every arm, such as the append arm's
  export, which holds both the old and the new structure).
- The region is the points within ``--radius`` m (XY) of the update path and of both reference
  paths, between ``--min-height`` and ``--max-height`` m above the local ground (the lowest
  point of all maps in each 2 m cell).

Measures (``d`` = ``--match``):

- **G**: share of the export's points with no point of either reference within d.
- **S**: share of persistent structure (points of reference 1 within d of reference 2 and of the
  map before the update) that the export covers within d, divided by the same share before.
- **N**: share of new structure (points of reference 1 within d of reference 2 and farther
  than 2 d from the map before the update) that the export covers within d.

  evaluate_nclt_map_update.py --before BEFORE.pcd --after AFTER.pcd --path UPDATE.tum
      --ref R1.pcd R1_PATH.tum R1_INIT.json --ref R2.pcd R2_PATH.tum R2_INIT.json [--match 0.2]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_luh_map_update import xyz_chunks  # noqa: E402


def load(path: Path) -> np.ndarray:
    return np.concatenate([c.astype(np.float64) for c in xyz_chunks(path)])


def transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def normals(points: np.ndarray, k: int = 10) -> np.ndarray:
    _, index = cKDTree(points).query(points, k=k)
    neighbours = points[index] - points[index].mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', neighbours, neighbours)
    return np.linalg.eigh(covariance)[1][:, :, 0]


def icp(source: np.ndarray, target: np.ndarray, init: np.ndarray, iterations: int = 30,
        max_distance: float = 1.0) -> tuple[np.ndarray, float]:
    """Point-to-plane ICP; returns source -> target and the median residual of the inliers."""
    tree, target_normals = cKDTree(target), normals(target)
    matrix = init.copy()
    for step in range(iterations):
        moved = transform(source, matrix)
        limit = max_distance if step < iterations // 2 else max_distance / 2
        distance, index = tree.query(moved, distance_upper_bound=limit)
        ok = np.isfinite(distance)
        p, q, n = moved[ok], target[index[ok]], target_normals[index[ok]]
        a = np.hstack([np.cross(p, n), n])
        b = np.einsum('ij,ij->i', q - p, n)
        x = np.linalg.lstsq(a, b, rcond=None)[0]
        rx, ry, rz = x[:3]
        update = np.eye(4)
        update[:3, :3] = np.array([[1, -rz, ry], [rz, 1, -rx], [-ry, rx, 1]])
        u, _, vt = np.linalg.svd(update[:3, :3])
        update[:3, :3] = u @ vt
        update[:3, 3] = x[3:]
        matrix = update @ matrix
        if np.linalg.norm(x) < 1e-6:
            break
    distance, _ = tree.query(transform(source, matrix), distance_upper_bound=max_distance / 2)
    return matrix, float(np.median(distance[np.isfinite(distance)]))


def path_xyz(path: Path) -> np.ndarray:
    return np.loadtxt(path, ndmin=2)[:, 1:4]


def covered(points: np.ndarray, reference: np.ndarray, d: float) -> np.ndarray:
    if len(reference) == 0:
        return np.zeros(len(points), dtype=bool)
    return cKDTree(reference).query(points, distance_upper_bound=d)[0] <= d


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--path', type=Path, required=True, help='update trajectory (TUM, store frame)')
    parser.add_argument('--ref', nargs=3, action='append', required=True,
                        metavar=('MAP', 'PATH_TUM', 'INIT_JSON'))
    parser.add_argument('--match', type=float, default=.20)
    parser.add_argument('--align-to', choices=('after', 'before'), default='after',
                        help='ICP target for the references: the export (default) or the map '
                             'before the update (one alignment shared by every arm)')
    parser.add_argument('--align-target', type=Path,
                        help='ICP target for the references, overriding --align-to: a map shared '
                             'by every arm, e.g. the append arm\'s export (old and new structure)')
    parser.add_argument('--radius', type=float, default=10.)
    parser.add_argument('--min-height', type=float, default=.3)
    parser.add_argument('--max-height', type=float, default=3.)
    args = parser.parse_args()
    if len(args.ref) != 2:
        parser.error('give exactly two --ref')
    d = args.match
    before, after = load(args.before), load(args.after)
    refs, paths, report = [], [], {'match_m': d, 'references': []}
    if args.align_target is not None:
        target = load(args.align_target)
        report['align_target'] = str(args.align_target)
    else:
        target = after if args.align_to == 'after' else before
    for map_path, tum, init in args.ref:
        points = load(Path(map_path))
        init = np.asarray(json.loads(Path(init).read_text()), dtype=float)
        matrix, residual = icp(points[::4], target[::2], init)
        refs.append(transform(points, matrix))
        paths.append(transform(path_xyz(Path(tum)), matrix))
        report['references'].append({'map': map_path, 'icp_median_residual_m': residual,
                                     'correction_m': float(np.linalg.norm(
                                         (matrix @ np.linalg.inv(init))[:3, 3]))})
    update_path = path_xyz(args.path)

    everything = np.concatenate([before, after, *refs])
    cells = np.floor(everything[:, :2] / 2.).astype(np.int64)
    keys, inverse = np.unique(cells, axis=0, return_inverse=True)
    ground = np.full(len(keys), np.inf)
    np.minimum.at(ground, inverse.ravel(), everything[:, 2])
    ground_of = dict(zip(map(tuple, keys.tolist()), ground.tolist()))
    trees = [cKDTree(p[:, :2]) for p in (update_path, *paths)]

    def region(points):
        cell = np.floor(points[:, :2] / 2.).astype(np.int64)
        floor = np.array([ground_of.get(tuple(c), np.inf) for c in cell.tolist()])
        height = points[:, 2] - floor
        keep = (height >= args.min_height) & (height <= args.max_height)
        for tree in trees:
            keep &= tree.query(points[:, :2], distance_upper_bound=args.radius)[0] <= args.radius
        return points[keep]

    before, after = region(before), region(after)
    r1, r2 = region(refs[0]), region(refs[1])
    confirmed = r1[covered(r1, r2, d)]
    ghost = ~covered(after, r1, d) & ~covered(after, r2, d)
    persistent = confirmed[covered(confirmed, before, d)]
    new = confirmed[~covered(confirmed, before, 2 * d)]
    s_before = covered(persistent, before, d).mean() if len(persistent) else float('nan')
    s_after = covered(persistent, after, d).mean() if len(persistent) else float('nan')
    report.update(points_after=len(after), points_before=len(before),
                  ghost_points=int(ghost.sum()), G=float(ghost.mean()), persistent_points=len(persistent),
                  S=float(s_after / s_before) if s_before else None,
                  new_points=len(new),
                  N=float(covered(new, after, d).mean()) if len(new) else None,
                  G_before=float((~covered(before, r1, d) & ~covered(before, r2, d)).mean()))
    print(json.dumps(report, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
