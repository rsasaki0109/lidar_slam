"""Render the lifelong NCLT chain (lifelong_chain.sh) as a GIF.

  render_lifelong_gif.py ROOT OUT.gif [--frames-per-session 10] [--cell 0.4]

ROOT is the bench directory with ``stores/lifelong`` and ``lifelong/`` (``chain.jsonl``,
``run_DAY/odometry_tum.txt``, ``export_DAY.pcd``: the store's merged map after each session).
Top down, store frame. For each session in date order its path is drawn in its season's colour
(placed by the store transform it committed), then the merged map switches to the one after it:
cells it added light up in the session's colour and cells change detection removed in red. A
timeline across the bottom marks every session; the counter gives sessions, months and map
changes so far.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from evaluate_nclt_map_update import load  # noqa: E402

BG = '#0e1116'
SEASON = {12: '#7fb3ff', 1: '#7fb3ff', 2: '#7fb3ff', 3: '#7ee0a1', 4: '#7ee0a1', 5: '#7ee0a1',
          6: '#ffc457', 7: '#ffc457', 8: '#ffc457', 9: '#ff8a5c', 10: '#ff8a5c', 11: '#ff8a5c'}


def cells(points: np.ndarray, cell: float, origin: np.ndarray) -> dict:
    """Occupied cells (points between 1 m below and 6 m above the store origin) and their
    point counts."""
    keep = (points[:, 2] > -1.0) & (points[:, 2] < 6.0)
    ij = np.floor((points[keep, :2] - origin) / cell).astype(np.int64)
    keys, counts = np.unique(ij, axis=0, return_counts=True)
    return dict(zip(map(tuple, keys), counts))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('root', type=Path)
    parser.add_argument('out', type=Path)
    parser.add_argument('--frames-per-session', type=int, default=10)
    parser.add_argument('--cell', type=float, default=0.4)
    parser.add_argument('--fps', type=int, default=12)
    parser.add_argument('--dpi', type=int, default=100)
    parser.add_argument('--wide', action='store_true',
                        help='landscape layout: map on the left, text and timeline on the right')
    args = parser.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    root = args.root
    store = json.loads((root / 'stores/lifelong/store.json').read_text())
    # Sessions the store committed (a run that did not finish is left out).
    chain = [c for c in map(json.loads, (root / 'lifelong/chain.jsonl').read_text().splitlines())
             if c['exit'] == 0][:len(store['sessions']) - 1]
    days = ['2012-01-15'] + [c['day'] for c in chain]
    exports = {d: root / f'lifelong/export_{d}.pcd' for d in days}
    days = [d for d in days if exports[d].is_file()]
    maps = {d: load(exports[d]) for d in days}
    origin = np.floor(np.min(np.concatenate([m[:, :2] for m in maps.values()]), 0))
    grids = {d: cells(maps[d], args.cell, origin) for d in days}
    transforms = [np.array(json.loads(s['store_from_session'])) if isinstance(
        s.get('store_from_session'), str) else (np.array(s['store_from_session'])
                                                 if s.get('store_from_session') else None)
        for s in store['sessions']]
    paths = {}
    for k, d in enumerate(days):
        odometry = (root / f'lifelong/run_{d}/odometry_tum.txt')
        if k == 0:
            odometry = root / 'runs/d_base/odometry_tum.txt'
        t = transforms[k] if k < len(transforms) and transforms[k] is not None else (
            np.eye(4) if k == 0 else None)
        if t is None or not odometry.is_file():
            paths[d] = None
            continue
        xyz = np.loadtxt(odometry, ndmin=2)[:, 1:4]
        paths[d] = xyz @ t[:3, :3].T + t[:3, 3]
    anchors = {c['day']: c['anchor_after_s'] for c in chain}

    # The view: every session's path with a 40 m margin.
    placed = np.concatenate([p[:, :2] for p in paths.values() if p is not None])
    lo = np.floor((placed.min(0) - 40.0 - origin) / args.cell).astype(np.int64)
    hi = np.floor((placed.max(0) + 40.0 - origin) / args.cell).astype(np.int64)
    extent = (lo, hi)
    size = (hi - lo + 1)
    peak = np.log1p(max(max(g.values()) for g in grids.values() if g))

    def inside(keys):
        ij = np.array(list(keys)).reshape(-1, 2) - lo
        ok = (ij >= 0).all(1) & (ij < size).all(1)
        return ij[ok], ok

    def image(grid, colour_cells=None):
        img = np.zeros((size[1], size[0], 3))
        img[:] = np.array([14, 17, 22]) / 255
        if grid:
            ij, ok = inside(grid.keys())
            level = 0.18 + 0.72 * np.log1p(np.array(list(grid.values()))[ok]) / peak
            img[ij[:, 1], ij[:, 0]] = level[:, None] * np.array([0.92, 0.95, 1.0])
        for colour, sel in (colour_cells or []):
            if sel:
                ij, _ = inside(sel)
                img[ij[:, 1], ij[:, 0]] = colour
        return img

    def rgb(hex_colour):
        return np.array([int(hex_colour[i:i + 2], 16) for i in (1, 3, 5)]) / 255

    if args.wide:
        fig = plt.figure(figsize=(9.6, 5.4), dpi=args.dpi, facecolor=BG)
        ax = fig.add_axes([0.01, 0.02, 0.56, 0.96])
        tl = fig.add_axes([0.60, 0.10, 0.37, 0.10])
        bars = fig.add_axes([0.60, 0.27, 0.37, 0.33])
        place = {'title': (0.60, 0.86), 'status': (0.60, 0.79), 'brand': (0.60, 0.94),
                 'area': (0.60, 0.72), 'align': 'left', 'big': 17, 'small': 12}
    else:
        fig = plt.figure(figsize=(7.2, 8.0), dpi=args.dpi, facecolor=BG)
        ax = fig.add_axes([0.03, 0.2, 0.94, 0.72])
        tl = fig.add_axes([0.06, 0.06, 0.88, 0.07])
        bars = None
        place = {'title': (0.05, 0.955), 'status': (0.05, 0.925), 'brand': (0.95, 0.955),
                 'area': (0.95, 0.925), 'align': 'right', 'big': 15, 'small': 11}
    dates = [dt.date.fromisoformat(d) for d in days]
    t0, t1 = dt.date(2012, 1, 1), dt.date(2013, 1, 1)
    span = (t1 - t0).days
    frames = []
    for k in range(len(days)):
        for f in range(args.frames_per_session):
            frames.append((k, f))
    frames += [(len(days) - 1, args.frames_per_session)] * (2 * args.fps)
    added_total = removed_total = 0
    totals = []
    for k, d in enumerate(days):
        if k:
            added_total += len(grids[d].keys() - grids[days[k - 1]].keys())
            removed_total += len(grids[days[k - 1]].keys() - grids[d].keys())
        totals.append((added_total, removed_total))

    def draw(frame):
        k, f = frame
        d = days[k]
        colour = SEASON[dates[k].month]
        ax.clear()
        tl.clear()
        ax.set_facecolor(BG)
        half = args.frames_per_session // 2
        if k == 0 or f < half:
            prev = grids[days[k - 1]] if k else grids[d]
            img = image(prev)
        else:
            gone = grids[days[k - 1]].keys() - grids[d].keys()
            new = grids[d].keys() - grids[days[k - 1]].keys()
            img = image(grids[d], [(rgb('#ff4d4d'), gone), (rgb(colour), new)])
        ax.imshow(img, origin='lower', interpolation='nearest',
                  extent=(extent[0][0] * args.cell + origin[0],
                          (extent[1][0] + 1) * args.cell + origin[0],
                          extent[0][1] * args.cell + origin[1],
                          (extent[1][1] + 1) * args.cell + origin[1]))
        for j in range(k):
            p = paths.get(days[j])
            if p is not None:
                ax.plot(p[:, 0], p[:, 1], color=SEASON[dates[j].month], lw=0.6, alpha=0.25)
        p = paths.get(d)
        if p is not None:
            n = len(p) if f >= half else max(2, int(len(p) * (f + 1) / half))
            ax.plot(p[:n, 0], p[:n, 1], color=colour, lw=2.0)
            ax.plot(p[n - 1, 0], p[n - 1, 1], 'o', color='white', ms=5)
        ax.set_aspect('equal')
        ax.axis('off')
        a, r = totals[k] if f >= half or k == 0 else totals[k - 1]
        status = ('first map' if k == 0 else
                  f'relocalized from scratch after {anchors.get(d):.0f} s'
                  if anchors.get(d) is not None else 'not relocalized')
        fig.texts.clear()
        fig.text(*place['title'], f'{d}   session {k + 1} / {len(days)}', color='white',
                 fontsize=place['big'], fontweight='bold')
        fig.text(*place['status'], status, color=colour, fontsize=place['small'])
        fig.text(*place['brand'], 'lifelong map · NCLT', color='#9aa4b2',
                 fontsize=place['small'], ha=place['align'])
        area = f'map +{a * args.cell ** 2:,.0f} m²  −{r * args.cell ** 2:,.0f} m²'
        fig.text(*place['area'], area,
                 color='#9aa4b2', fontsize=place['small'], ha=place['align'])
        tl.set_facecolor(BG)
        tl.set_xlim(0, span)
        tl.set_ylim(-1, 1)
        tl.axhline(0, color='#3a4250', lw=1)
        for j, day in enumerate(dates):
            x = (day - t0).days
            tl.plot(x, 0, 'o', ms=7 if j == k else 4,
                    color=SEASON[day.month] if j <= k else '#3a4250')
        for m in range(0, 13, 2):
            x = (dt.date(2012 + (m // 12), m % 12 + 1, 1) - t0).days
            tl.text(x, -0.95, dt.date(2012 + (m // 12), m % 12 + 1, 1).strftime('%b %y'),
                    color='#6b7482', fontsize=8, ha='center', va='bottom')
        tl.axis('off')
        if bars is not None:
            # Seconds to relocalize from scratch, one bar per session (log scale).
            bars.clear()
            bars.set_facecolor(BG)
            values = [anchors.get(days[j]) for j in range(1, k + 1)]
            for j, v in enumerate(values, start=1):
                if v is not None:
                    bars.bar(j, v, width=0.7, color=SEASON[dates[j].month],
                             alpha=1.0 if j == k else 0.55)
            bars.set_yscale('log')
            bars.set_ylim(2, 300)
            bars.set_xlim(0.3, len(days) - 0.3)
            bars.set_yticks([3, 10, 30, 100])
            bars.set_yticklabels(['3 s', '10 s', '30 s', '100 s'], color='#6b7482', fontsize=8)
            bars.minorticks_off()
            bars.tick_params(axis='x', bottom=False, labelbottom=False)
            bars.tick_params(axis='y', length=0)
            for side in ('top', 'right', 'bottom'):
                bars.spines[side].set_visible(False)
            bars.spines['left'].set_color('#3a4250')
            bars.grid(axis='y', color='#262c36', lw=0.6)
            bars.set_axisbelow(True)
            bars.set_title('time to relocalize, no initial pose', color='#9aa4b2', fontsize=9,
                           loc='left', pad=4)

    anim = FuncAnimation(fig, draw, frames=frames)
    anim.save(args.out, writer=PillowWriter(fps=args.fps))
    print(f'{args.out}: {len(frames)} frames')
    return 0


if __name__ == '__main__':
    sys.exit(main())
