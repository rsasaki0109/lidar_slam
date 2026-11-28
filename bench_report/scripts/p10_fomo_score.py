#!/usr/bin/env python3
"""ATE of one FoMo run (bench_report/data/fomo_dev_2026-11-28/plan.txt).

usage: p10_fomo_score.py EST.tum GT.txt [--cov gt_covariance.csv] [--lever x y z] [--json OUT]

EST: TUM rows (t [s], x y z, qx qy qz qw), the pose of base_link. GT: FoMo's gt.txt (TUM, PPK GNSS).
--lever: the reference point in base_link (applied to the estimate's pose before scoring); default 0 0 0.
ATE: estimate positions interpolated at the reference stamps inside the estimate's span, rigid SE(3) Umeyama fit
without scale, RMSE. Also the ATE without reference spikes (a single reference pose more than 1 m from both
neighbours while the neighbours are no farther apart than 0.5 m + 5 m/s x their time gap; 5 m/s is above the
Warthog's top speed), found from the reference alone, and the coverage of the reference's time span.
--at FILE scores at the stamps of FILE instead (the reference interpolated there; for sparse keyframe
trajectories, whose linear interpolation in time is wrong where the speed changes).
Run with `python3 -I`: the reference is untrusted input.
"""
import argparse
import json
import math

import numpy as np


def align(e, r):
    mu_e, mu_r = e.mean(0), r.mean(0)
    u, _, vt = np.linalg.svd((r - mu_r).T @ (e - mu_e))
    R = u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt
    return R, mu_r - R @ mu_e


def quat_rot(q, v):
    x, y, z, w = q.T
    R = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                  2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                  2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], 1).reshape(-1, 3, 3)
    return R @ v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('est')
    ap.add_argument('gt')
    ap.add_argument('--cov')
    ap.add_argument('--lever', type=float, nargs=3, default=[0.0, 0.0, 0.0])
    ap.add_argument('--json')
    ap.add_argument('--at', help='score at these stamps (first column of a TUM file, e.g. the backend keyframes): the '
                    'reference is interpolated there instead of the estimate at the reference stamps')
    a = ap.parse_args()
    est = np.loadtxt(a.est, ndmin=2)
    gt = np.loadtxt(a.gt, ndmin=2, comments='#')
    est = est[np.argsort(est[:, 0], kind='stable')]
    gt = gt[np.argsort(gt[:, 0], kind='stable')]
    if gt[0, 0] > 1e12:  # microseconds
        gt[:, 0] *= 1e-6
    out = {'est_poses': len(est), 'finite': bool(np.isfinite(est).all())}
    p = est[:, 1:4] + quat_rot(est[:, 4:8], np.array(a.lever))
    inside = (gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])
    rt, rp = gt[inside, 0], gt[inside, 1:4]
    if a.at:
        st = np.loadtxt(a.at, ndmin=2)[:, 0]
        st = st[(st >= max(est[0, 0], gt[0, 0])) & (st <= min(est[-1, 0], gt[-1, 0]))]
        rp = np.stack([np.interp(st, gt[:, 0], gt[:, k]) for k in (1, 2, 3)], 1)
        rt = st
    out['coverage'] = float((rt[-1] - rt[0]) / (gt[-1, 0] - gt[0, 0])) if inside.sum() > 1 else 0.0
    out['ref_poses'] = int(inside.sum())
    out['path_m'] = float(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum())
    out['duration_s'] = float(gt[-1, 0] - gt[0, 0])
    if inside.sum() < 3 or not out['finite']:
        out['ate'] = out['ate_nospike'] = math.nan
    else:
        ep = np.stack([np.interp(rt, est[:, 0], p[:, k]) for k in range(3)], 1)
        R, t = align(ep, rp)
        err = np.linalg.norm(ep @ R.T + t - rp, axis=1)
        spike = np.zeros(len(rp), bool)
        if len(rp) > 2:
            gp = np.linalg.norm(rp[1:-1] - rp[:-2], axis=1)
            gn = np.linalg.norm(rp[1:-1] - rp[2:], axis=1)
            across = np.linalg.norm(rp[2:] - rp[:-2], axis=1)
            spike[1:-1] = (gp > 1.0) & (gn > 1.0) & (across < 0.5 + 5.0 * (rt[2:] - rt[:-2]))
        R2, t2 = align(ep[~spike], rp[~spike])
        err2 = np.linalg.norm(ep[~spike] @ R2.T + t2 - rp[~spike], axis=1)
        # Sim(3) scale (diagnostic): the factor on the estimate that fits the reference best
        me, mr = ep.mean(0), rp.mean(0)
        E, Rr = ep - me, rp - mr
        u, sv, vt = np.linalg.svd(Rr.T @ E / len(ep))
        D = np.diag([1, 1, np.sign(np.linalg.det(u @ vt))])
        c = float((sv * np.diag(D)).sum() / ((E ** 2).sum() / len(ep)))
        Rs = u @ D @ vt
        ds = c * E @ Rs.T + mr - rp
        out.update(sim3_scale=c, ate_sim3=float(np.sqrt((ds ** 2).sum(1).mean())))
        out.update(ate=float(np.sqrt((err ** 2).mean())), median=float(np.median(err)), max=float(err.max()),
                   spikes=int(spike.sum()), ate_nospike=float(np.sqrt((err2 ** 2).mean())))
    if a.cov:
        # FoMo's gt_covariance.csv: no header; t, then the 3 x 3 position covariance row by row [m^2]
        c = np.loadtxt(a.cov, delimiter=',', ndmin=2)
        if c.shape[1] == 10:
            sh = np.sqrt(c[:, 1] + c[:, 5])
            sv = np.sqrt(c[:, 9])
            out.update(ref_sigma_h_median=float(np.median(sh)), ref_sigma_h_p95=float(np.percentile(sh, 95)),
                       ref_sigma_h_max=float(sh.max()), ref_sigma_v_median=float(np.median(sv)))
        else:
            out['cov_columns'] = int(c.shape[1])
    text = json.dumps(out, indent=1)
    print(text)
    if a.json:
        with open(a.json, 'w') as f:
            f.write(text + '\n')


if __name__ == '__main__':
    main()
