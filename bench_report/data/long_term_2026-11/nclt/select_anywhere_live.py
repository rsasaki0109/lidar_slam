"""Windows for the live (--realtime) check of --relocalize-anywhere, from 1 Hz GT positions only.

Sessions used so far are excluded. Base: the unused session with the longest pass through the
evaluation square (its pass +-10 s becomes the store). Then for each later period, the unused
session with the longest pass: in-area run = that pass +-10 s; out-of-area run = the first 480 s
of its longest stretch at least 100 m (XY) from every GT position of the base pass.

  python3 -I select_anywhere_live.py ROOT   (ROOT holds windows.json and gt1hz/DAY.npy)
"""
import glob
import json
import sys

import numpy as np
from scipy.spatial import cKDTree

USED = {'2012-01-15', '2012-02-18', '2012-05-11', '2012-05-26', '2012-06-15', '2012-11-04',
        '2012-11-16', '2012-11-17', '2012-02-02', '2012-03-17', '2012-09-28', '2012-10-28'}
PERIODS = {'2012-03..05': ('2012-03', '2012-06'), '2012-06..09': ('2012-06', '2012-10'),
           '2012-10..2013-04': ('2012-10', '2013-05')}


def runs(mask):
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(edges[::2], edges[1::2]))


def main():
    root = sys.argv[1]
    x0, x1, y0, y1 = json.load(open(f'{root}/windows.json'))['region_xy']
    days = sorted(p.split('/')[-1][:-4] for p in glob.glob(f'{root}/gt1hz/*.npy'))
    gts = {d: np.load(f'{root}/gt1hz/{d}.npy') for d in days if d not in USED}

    def longest_pass(g):
        inside = (g[:, 1] >= x0) & (g[:, 1] <= x1) & (g[:, 2] >= y0) & (g[:, 2] <= y1)
        best = max(runs(inside), key=lambda r: g[r[1] - 1, 0] - g[r[0], 0], default=None)
        return best, (0 if best is None else (g[best[1] - 1, 0] - g[best[0], 0]) / 1e6)

    def window(g, r):
        return [int(g[r[0], 0] - 10e6), int(g[r[1] - 1, 0] + 10e6)]

    base = max(gts, key=lambda d: longest_pass(gts[d])[1])
    (bs, be), _ = longest_pass(gts[base])
    out = {'base': base, 'base_window_utime': window(gts[base], (bs, be)), 'sessions': {}}
    tree = cKDTree(gts[base][bs:be, 1:3])
    for name, (lo, hi) in PERIODS.items():
        cands = [d for d in gts if lo <= d[:7] < hi and d > base]
        day = max(cands, key=lambda d: longest_pass(gts[d])[1])
        g = gts[day]
        r, length = longest_pass(g)
        far = tree.query(g[:, 1:3])[0] >= 100
        fs, fe = max(runs(far), key=lambda q: g[q[1] - 1, 0] - g[q[0], 0])
        f0 = int(g[fs, 0])
        f1 = int(min(g[fe - 1, 0], f0 + 480e6))
        seg = g[(g[:, 0] >= f0) & (g[:, 0] <= f1), 1:3]
        out['sessions'][day] = {
            'period': name, 'in_area_utime': window(g, r), 'in_area_s': round(length + 20),
            'out_of_area_utime': [f0, f1], 'out_of_area_s': round((f1 - f0) / 1e6),
            'out_of_area_min_dist_m': round(float(tree.query(seg)[0].min()), 1)}
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
