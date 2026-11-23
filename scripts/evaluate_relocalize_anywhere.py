#!/usr/bin/env python3
"""Check one ``live --relocalize-anywhere`` anchor on NCLT against ground truth and the stored map.

An anchor is the session's ``store_from_session`` (session odometry frame -> store frame). It is
compared with two things:

- **The ground-truth placement** ``T_g``: the base session's odometry and the session's odometry
  are each fitted rigidly to their NCLT ground truth (Umeyama, poses matched within 0.5 s), and
  ``T_g = inv(world_from_base) @ world_from_session``. ``gt_offset_rms_m`` is the RMS distance
  between the session's odometry positions placed by the anchor and by ``T_g``. NCLT's
  ground truth places one session against another only to 1-2 m.
- **The stored map**: point-to-plane ICP of the session's map against the base map (the store
  frame), once from the anchor (``M_a``) and once from ``T_g`` (``M_g``). ``converged_apart_m``
  is the RMS distance between the session's odometry positions placed by ``M_a`` and by ``M_g``,
  ``converged_apart_deg`` the rotation angle of ``M_a @ inv(M_g)``. Small values mean ICP from
  the anchor reaches the same fit as ICP from the ground-truth placement.

With ``--gt-starts`` ICP also runs from perturbed copies of ``T_g`` (same ``DX,YAW,DIR`` form as
``--perturb``), and each row adds the best fit near the ground truth: among the solutions from
``T_g``, those starts and the anchor that lie within ``--gt-tolerance`` m RMS of ``T_g``, the one
with the lowest median residual. ``best_apart_m`` / ``best_apart_deg`` place the anchor's ICP
solution against it (0 when the anchor's own solution is the best), ``best_from`` names the start
it came from and ``best_residual_m`` its residual. This does not take ``T_g`` itself, good only to
1-2 m and a few degrees, as the reference basin.

With ``--perturb`` the anchor is replaced by perturbed copies, for calibrating the measure on
development data: each perturbation moves the anchor-placed path by ``dx`` m in the XY
direction ``dir`` degrees (store frame) and turns it by ``yaw`` degrees about the path's
centroid. One JSON line per anchor (the unperturbed one first).

  evaluate_relocalize_anywhere.py --base-map BASE.pcd --base-odometry BASE.tum --base-gt BASE.npy
      --session-map S.pcd --session-odometry S.tum --session-gt S.npy --anchor STORE.json
      [--perturb DX,YAW,DIR ...]

``--anchor`` takes a store.json (the last session's ``store_from_session``) or a JSON 4x4
matrix. Ground-truth files are NumPy arrays with rows (utime, x, y, z, ...).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_luh_trajectory import fit  # noqa: E402
from evaluate_nclt_map_update import icp, load, transform  # noqa: E402


def world_from_run(odometry: np.ndarray, truth: np.ndarray) -> tuple[np.ndarray, int]:
    """Rigid fit of an odometry trajectory (TUM rows) to NCLT ground truth (utime rows)."""
    t = truth[:, 0] * 1e-6
    i = np.searchsorted(t, odometry[:, 0]).clip(1, len(t) - 1)
    i = np.where(np.abs(t[i - 1] - odometry[:, 0]) < np.abs(t[i] - odometry[:, 0]), i - 1, i)
    ok = np.abs(t[i] - odometry[:, 0]) < 0.5
    rotation, translation = fit(odometry[ok, 1:4], truth[i[ok], 1:4])
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    return matrix, int(ok.sum())


def read_anchor(path: Path) -> np.ndarray | None:
    data = json.loads(path.read_text(encoding='utf-8'))
    if isinstance(data, dict):
        data = data['sessions'][-1].get('store_from_session')
        if data is None:
            return None
    return np.asarray(data, dtype=float)


def angle_deg(matrix: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(matrix[:3, :3]) - 1) / 2, -1, 1))))


def rms_apart(a: np.ndarray, b: np.ndarray, points: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((transform(points, a) - transform(points, b)) ** 2, 1))))


def perturbed(anchor: np.ndarray, points: np.ndarray, dx: float, yaw_deg: float,
              dir_deg: float) -> np.ndarray:
    centre = transform(points, anchor).mean(axis=0)
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    turn = np.eye(4)
    turn[:2, :2] = [[c, -s], [s, c]]
    turn[:3, 3] = centre - turn[:3, :3] @ centre
    turn[0, 3] += dx * np.cos(np.radians(dir_deg))
    turn[1, 3] += dx * np.sin(np.radians(dir_deg))
    return turn @ anchor


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for name in ('base-map', 'base-odometry', 'base-gt', 'session-map', 'session-odometry',
                 'session-gt', 'anchor'):
        parser.add_argument(f'--{name}', type=Path, required=True)
    parser.add_argument('--perturb', nargs='*', default=[], metavar='DX,YAW,DIR')
    parser.add_argument('--gt-starts', nargs='*', default=[], metavar='DX,YAW,DIR')
    parser.add_argument('--gt-tolerance', type=float, default=4.0)
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    base_odometry = np.loadtxt(args.base_odometry, ndmin=2)
    session_odometry = np.loadtxt(args.session_odometry, ndmin=2)
    world_from_base, base_matched = world_from_run(base_odometry, np.load(args.base_gt))
    world_from_session, session_matched = world_from_run(
        session_odometry, np.load(args.session_gt))
    ground_truth = np.linalg.inv(world_from_base) @ world_from_session
    path = session_odometry[:, 1:4]
    anchor = read_anchor(args.anchor)
    head = {'label': args.label, 'base_gt_matched': base_matched,
            'session_gt_matched': session_matched}
    if anchor is None:
        print(json.dumps({**head, 'anchored': False}))
        return 0

    source = load(args.session_map)[::4]
    target = load(args.base_map)[::2]
    from_truth, truth_residual = icp(source, target, ground_truth)
    near_truth = [('ground_truth', from_truth, truth_residual)]
    for spec in args.gt_starts:
        dx, yaw, direction = (float(v) for v in spec.split(','))
        solution, residual = icp(source, target, perturbed(ground_truth, path, dx, yaw, direction))
        near_truth.append((f'ground_truth+{spec}', solution, residual))
    anchors = [('anchor', anchor)]
    for spec in args.perturb:
        dx, yaw, direction = (float(v) for v in spec.split(','))
        anchors.append((spec, perturbed(anchor, path, dx, yaw, direction)))
    for name, start in anchors:
        from_anchor, residual = icp(source, target, start)
        extra = {}
        if args.gt_starts:
            candidates = [c for c in near_truth + [('anchor', from_anchor, residual)]
                          if rms_apart(c[1], ground_truth, path) <= args.gt_tolerance]
            if candidates:
                best_from, best, best_residual = min(candidates, key=lambda c: c[2])
                extra = {'best_from': best_from, 'best_residual_m': best_residual,
                         'best_apart_m': rms_apart(from_anchor, best, path),
                         'best_apart_deg': angle_deg(from_anchor @ np.linalg.inv(best)),
                         'anchor_solution_offset_m': rms_apart(from_anchor, ground_truth, path)}
        print(json.dumps({
            **head, 'anchored': True, 'start': name,
            'start_from_anchor_m': rms_apart(start, anchor, path),
            'gt_offset_rms_m': rms_apart(start, ground_truth, path),
            'icp_correction_m': rms_apart(from_anchor, start, path),
            'icp_median_residual_m': residual,
            'icp_from_gt_correction_m': rms_apart(from_truth, ground_truth, path),
            'icp_from_gt_median_residual_m': truth_residual,
            'converged_apart_m': rms_apart(from_anchor, from_truth, path),
            'converged_apart_deg': angle_deg(from_anchor @ np.linalg.inv(from_truth)),
            **extra,
        }), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
