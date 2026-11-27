#!/usr/bin/env python3
"""Break a closed-loop run's final-map error down: what loop closure could and did remove.

Per run directory (written by run_closed_loop.py):

- ``ate_raw`` / ``ate_opt``: SE(2)-aligned RMSE of the submap positions before (odometry only)
  and after the pose graph, against the truth at the submaps' stamps.
- ``ate_sim2`` and ``scale``: the same after a similarity alignment (rotation, translation and
  one scale): the part of the error a uniform stretch of the map explains.
- ``along_frac``: the share of the squared error along the robot's heading.
- per loop edge: ``loop_drift`` (truth relative pose vs the odometry's: the drift the loop can
  correct), ``loop_meas_err`` (truth relative pose vs the loop's measurement).
- ``sigma_err_spearman``: rank correlation of the final graph's per-submap sigma with its error.

The truth in frames.csv is the pose at the frame's end (``t`` + 0.1 s).

  python3 tools/rne_active_slam_env/diagnose_map_error.py RUN_DIR [RUN_DIR ...] > diag.json
"""

from __future__ import annotations

import collections
import csv
import json
from pathlib import Path
import sys

import numpy as np

FRAME_S = 0.1
STAMP0 = 1700000000.0


def align(est: np.ndarray, ref: np.ndarray, scale: bool = False) -> tuple[np.ndarray, float]:
    """Return est aligned to ref (least squares SE(2), or Sim(2) with scale) and the scale."""
    mu_e, mu_r = est.mean(0), ref.mean(0)
    e, r = est - mu_e, ref - mu_r
    u, s, vt = np.linalg.svd(r.T @ e)
    d = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    c = float((s * np.diag(d)).sum() / (e ** 2).sum()) if scale else 1.0
    return c * e @ (u @ d @ vt).T + mu_r, c


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, 1))))


def yaw_of(q: np.ndarray) -> float:
    x, y, z, w = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def ranks(v: np.ndarray) -> np.ndarray:
    return np.argsort(np.argsort(v)).astype(float)


def diagnose(run: Path) -> dict:
    """Return the error breakdown of one run."""
    frames = np.genfromtxt(run / 'frames.csv', delimiter=',', names=True, dtype=None,
                           encoding=None)
    ts = frames['t'].astype(float) + FRAME_S
    tx, ty = frames['truth_x'].astype(float), frames['truth_y'].astype(float)
    tyaw = np.unwrap(frames['truth_yaw'].astype(float))

    def truth(stamps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        s = np.asarray(stamps) - STAMP0
        return (np.stack([np.interp(s, ts, tx), np.interp(s, ts, ty)], 1),
                np.interp(s, ts, tyaw))

    raw = np.loadtxt(run / 'slam' / 'trajectory_raw.tum', ndmin=2)
    opt = np.loadtxt(run / 'slam' / 'trajectory_optimized.tum', ndmin=2)
    ref, yaw = truth(opt[:, 0])
    a_opt, _ = align(opt[:, 1:3], ref)
    a_sim, scale = align(opt[:, 1:3], ref, scale=True)
    ref_raw = truth(raw[:, 0])[0]
    out = {'ate_raw': rmse(align(raw[:, 1:3], ref_raw)[0], ref_raw), 'ate_opt': rmse(a_opt, ref),
           'ate_sim2': rmse(a_sim, ref), 'scale': scale}
    res = a_opt - ref
    along = res[:, 0] * np.cos(yaw) + res[:, 1] * np.sin(yaw)
    out['along_frac'] = float(np.sum(along ** 2) / np.sum(res ** 2))

    drift, meas_err = [], []
    with open(run / 'slam' / 'loop_edges.csv', encoding='utf-8') as f:
        for edge in csv.DictReader(f):
            a, b = int(edge['from']), int(edge['to'])
            rel = {}
            for name, pa, pb, wa in (
                    ('odom', raw[a, 1:3], raw[b, 1:3], yaw_of(raw[a, 4:8])),
                    ('truth', *truth(raw[[a, b], 0])[0], truth(raw[[a], 0])[1][0])):
                dx, dy = pb - pa
                rel[name] = np.array([np.cos(wa) * dx + np.sin(wa) * dy,
                                      -np.sin(wa) * dx + np.cos(wa) * dy])
            meas = np.array([float(edge['tx']), float(edge['ty'])])
            drift.append(float(np.hypot(*(rel['odom'] - rel['truth']))))
            meas_err.append(float(np.hypot(*(meas - rel['truth']))))
    out['loops'] = len(drift)
    out['loop_drift_median'] = float(np.median(drift)) if drift else None
    out['loop_meas_err_median'] = float(np.median(meas_err)) if meas_err else None

    last: dict[int, list] = collections.defaultdict(list)
    with open(run / 'slam' / 'pose_uncertainty_timeline.csv', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    final_step = max(int(r['step']) for r in rows)
    for r in rows:
        if int(r['step']) == final_step:
            last[int(r['id'])] = [float(r['x']), float(r['y']), float(r['sigma_translation_max'])]
    ids = np.array([i for i in sorted(last) if i < len(opt)])
    xy = np.array([last[i][:2] for i in ids])
    sigma = np.array([last[i][2] for i in ids])
    ref_ids = truth(opt[ids, 0])[0]
    err = np.linalg.norm(align(xy, ref_ids)[0] - ref_ids, axis=1)
    out['sigma_err_spearman'] = float(np.corrcoef(ranks(sigma), ranks(err))[0, 1])
    return out


def main(argv: list[str]) -> int:
    result = {}
    for arg in argv:
        run = Path(arg)
        try:
            result[run.name] = diagnose(run)
        except (OSError, ValueError, KeyError, IndexError) as error:
            print(f'skip {run}: {error}', file=sys.stderr)
    print(json.dumps(result, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
