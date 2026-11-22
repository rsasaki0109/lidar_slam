"""Holdout windows from 1 Hz GT positions only (no scoring).
In-area: per period, the unused session with the longest pass through the region; window = pass +-10 s.
Out-of-area: in the same session, the first 480 s of its longest run >= 100 m (XY) from the base pass."""
import json, sys
import numpy as np
from scipy.spatial import cKDTree
root = sys.argv[1]
W = json.load(open(f'{root}/windows.json')); x0, x1, y0, y1 = W['region_xy']
used = set(W['windows_utime'])
base = '2012-02-18'
b = np.load(f'{root}/gt1hz/{base}.npy'); wb = W['windows_utime'][base]
tree = cKDTree(b[(b[:, 0] >= wb[0]) & (b[:, 0] <= wb[1]), 1:3])
periods = {'2012-01..02': ('2012-01', '2012-03'), '2012-03..05': ('2012-03', '2012-06'),
           '2012-06..09': ('2012-06', '2012-10'), '2012-10..2013-04': ('2012-10', '2013-05')}
days = sorted(p.split('/')[-1][:-4] for p in __import__('glob').glob(f'{root}/gt1hz/*.npy'))
def runs(mask):
    e = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0])); return list(zip(e[::2], e[1::2]))
out = {}
for name, (lo, hi) in periods.items():
    best = None
    for d in days:
        if d in used or not (lo <= d[:7] < hi): continue
        g = np.load(f'{root}/gt1hz/{d}.npy')
        inside = (g[:, 1] >= x0) & (g[:, 1] <= x1) & (g[:, 2] >= y0) & (g[:, 2] <= y1)
        r = max(runs(inside), key=lambda se: g[se[1] - 1, 0] - g[se[0], 0], default=None)
        if r is None: continue
        length = (g[r[1] - 1, 0] - g[r[0], 0]) / 1e6
        if best is None or length > best[1]: best = (d, length, g, r)
    d, length, g, (s, e) = best
    far = tree.query(g[:, 1:3])[0] >= 100
    fs, fe = max(runs(far), key=lambda se: g[se[1] - 1, 0] - g[se[0], 0])
    f0 = int(g[fs, 0]); f1 = int(min(g[fe - 1, 0], f0 + 480e6))
    out[d] = {'period': name, 'in_area_utime': [int(g[s, 0] - 10e6), int(g[e - 1, 0] + 10e6)],
              'in_area_s': round(length + 20), 'out_of_area_utime': [f0, f1],
              'out_of_area_s': round((f1 - f0) / 1e6),
              'out_of_area_min_dist_m': round(float(tree.query(g[(g[:, 0] >= f0) & (g[:, 0] <= f1), 1:3])[0].min()), 1)}
print(json.dumps(out, indent=1))
