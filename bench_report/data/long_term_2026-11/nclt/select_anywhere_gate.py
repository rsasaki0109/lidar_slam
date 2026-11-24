"""Windows for the gate-0.26 check of --relocalize-anywhere, from 1 Hz GT positions only.

Only sessions never used so far. A new 150 m evaluation square that does not overlap the old
one (windows.json region_xy): on a 10 m grid, the square through which the most unused sessions
pass for at least 120 s (longest single pass), ties broken by the larger sum of those passes
capped at 300 s, then by the smaller x, y. Base: the unused session with the longest pass (its
pass +-10 s becomes the store). Every other unused session with a pass of at least 120 s gives an
in-area run (that pass +-10 s) and an out-of-area run (the first 480 s of its longest stretch at
least 100 m in XY from every GT position of the base pass).

  python3 -I select_anywhere_gate.py ROOT
  (ROOT holds windows.json and gt1hz/DAY.npy)
"""
import glob
import json
import sys

import numpy as np
from scipy.spatial import cKDTree

USED = {'2012-01-15', '2012-02-18', '2012-05-11', '2012-05-26', '2012-06-15', '2012-11-04',
        '2012-11-16', '2012-11-17', '2012-02-02', '2012-03-17', '2012-09-28', '2012-10-28',
        '2012-02-05', '2012-03-25', '2012-08-20', '2012-12-01', '2012-01-22', '2012-03-31',
        '2012-08-04', '2013-02-23'}
SIDE = 150.0
MIN_PASS = 120.0


def runs(mask):
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(edges[::2], edges[1::2]))


def longest_pass(g, x0, y0):
    inside = (g[:, 1] >= x0) & (g[:, 1] <= x0 + SIDE) & (g[:, 2] >= y0) & (g[:, 2] <= y0 + SIDE)
    best = max(runs(inside), key=lambda r: g[r[1] - 1, 0] - g[r[0], 0], default=None)
    return best, (0.0 if best is None else (g[best[1] - 1, 0] - g[best[0], 0]) / 1e6)


def window(g, r):
    return [int(g[r[0], 0] - 10e6), int(g[r[1] - 1, 0] + 10e6)]


def main():
    root = sys.argv[1]
    ox0, ox1, oy0, oy1 = json.load(open(f'{root}/windows.json'))['region_xy']
    days = sorted(p.split('/')[-1][:-4] for p in glob.glob(f'{root}/gt1hz/*.npy'))
    gts = {d: np.load(f'{root}/gt1hz/{d}.npy') for d in days if d not in USED}
    every = np.concatenate([g[:, 1:3] for g in gts.values()])
    lo, hi = np.floor(every.min(0) / 10) * 10, np.ceil(every.max(0) / 10) * 10
    best = None
    for x0 in np.arange(lo[0], hi[0] - SIDE + 1, 10.0):
        for y0 in np.arange(lo[1], hi[1] - SIDE + 1, 10.0):
            if x0 < ox1 and x0 + SIDE > ox0 and y0 < oy1 and y0 + SIDE > oy0:
                continue
            passes = [longest_pass(g, x0, y0)[1] for g in gts.values()]
            key = (sum(p >= MIN_PASS for p in passes),
                   sum(min(p, 300.0) for p in passes if p >= MIN_PASS), -x0, -y0)
            if best is None or key > best[0]:
                best = (key, float(x0), float(y0))
    _, x0, y0 = best
    lengths = {d: longest_pass(g, x0, y0) for d, g in gts.items()}
    base = max(lengths, key=lambda d: lengths[d][1])
    bs, be = lengths[base][0]
    out = {'region_xy': [x0, x0 + SIDE, y0, y0 + SIDE], 'base': base,
           'base_window_utime': window(gts[base], (bs, be)), 'sessions': {}}
    tree = cKDTree(gts[base][bs:be, 1:3])
    for day in sorted(lengths):
        r, length = lengths[day]
        if day == base or length < MIN_PASS:
            continue
        g = gts[day]
        far = tree.query(g[:, 1:3])[0] >= 100
        fs, fe = max(runs(far), key=lambda q: g[q[1] - 1, 0] - g[q[0], 0])
        f0 = int(g[fs, 0])
        f1 = int(min(g[fe - 1, 0], f0 + 480e6))
        seg = g[(g[:, 0] >= f0) & (g[:, 0] <= f1), 1:3]
        out['sessions'][day] = {
            'in_area_utime': window(g, r), 'in_area_s': round(length + 20),
            'out_of_area_utime': [f0, f1], 'out_of_area_s': round((f1 - f0) / 1e6),
            'out_of_area_min_dist_m': round(float(tree.query(seg)[0].min()), 1)}
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
