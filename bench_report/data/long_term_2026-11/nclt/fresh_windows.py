"""Lifelong-square windows of the NCLT sessions outside the lifelong chain, from 1 Hz GT
positions only (window choice; nothing is scored with them). Longest pass through the square
+-10 s, at most 600 s."""
import json
import sys

import numpy as np

root = sys.argv[1]
x0, x1, y0, y1 = json.load(open(f'{root}/windows.json'))['region_xy']
out = {}
for d in ('2012-01-08', '2012-02-04', '2012-02-12', '2012-02-19', '2012-04-29', '2013-01-10',
          '2013-04-05'):
    g = np.load(f'{root}/gt1hz/{d}.npy')
    inside = (g[:, 1] >= x0) & (g[:, 1] <= x1) & (g[:, 2] >= y0) & (g[:, 2] <= y1)
    edges = np.flatnonzero(np.diff(np.r_[0, inside.astype(int), 0]))
    passes = list(zip(edges[::2], edges[1::2]))
    if not passes:
        out[d] = None
        continue
    s, e = max(passes, key=lambda se: g[se[1] - 1, 0] - g[se[0], 0])
    begin = int(g[s, 0] - 10e6)
    end = int(min(g[e - 1, 0] + 10e6, begin + 600e6))
    out[d] = {'utime': [begin, end], 'seconds': round((end - begin) / 1e6)}
print(json.dumps(out, indent=1))
