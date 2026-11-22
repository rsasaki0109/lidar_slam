#!/usr/bin/env python3
"""Render a top-view GIF of one ``--relocalize-anywhere`` run against a stored map.

The stored map is drawn in grey with the path of the session that built it. While the new run
searches, only its own odometry is known (inset, its own frame). When it anchors, its pose
appears in the stored map, and from then on its path and its map grow there in orange.

  render_relocalize_anywhere_gif.py STORE_EXPORT RUN OUT.gif [--title ...] [--size 960]

STORE_EXPORT is a map store's ``export`` directory (map.pcd, pose_graph.g2o); RUN the output of
``python -m lidarslam live --map-store ... --relocalize-anywhere`` (trajectory_live.tum,
trajectory_store.tum, map.pcd).
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_luh_map_update import xyz_chunks  # noqa: E402

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
BG = (14, 17, 23)
PRIOR = (150, 158, 170)
PRIOR_PATH = (90, 140, 220)
LIVE = (255, 150, 40)
LIVE_PATH = (255, 210, 120)


def tum(path):
    d = np.loadtxt(path, ndmin=2)
    return d[:, 0], d[:, 1:4], Rotation.from_quat(d[:, 4:8])


def matrix(p, r):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r.as_matrix(), p
    return m


def g2o_positions(path):
    out = []
    for line in Path(path).read_text().splitlines():
        if line.startswith('VERTEX_SE3:QUAT'):
            out.append([float(v) for v in line.split()[2:5]])
    return np.array(out)


def cloud(path, limit, rng):
    pts = np.concatenate([c[:, :3].astype(np.float64) for c in xyz_chunks(path)])
    if len(pts) > limit:
        pts = pts[rng.choice(len(pts), limit, replace=False)]
    return pts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('store', type=Path)
    parser.add_argument('run', type=Path)
    parser.add_argument('out', type=Path)
    parser.add_argument('--title', default='')
    parser.add_argument('--subtitle', default='')
    parser.add_argument('--size', type=int, default=960)
    parser.add_argument('--search-frames', type=int, default=36)
    parser.add_argument('--flash-frames', type=int, default=14)
    parser.add_argument('--track-frames', type=int, default=90)
    parser.add_argument('--hold-frames', type=int, default=30)
    parser.add_argument('--fps', type=int, default=15)
    args = parser.parse_args()
    rng = np.random.default_rng(0)

    lt, lp, lr = tum(args.run / 'trajectory_live.tum')
    st, sp, sr = tum(args.run / 'trajectory_store.tum')
    # Time counts from the first odometry pose, as in the run log's anchor time.
    t0, anchor = tum(args.run / 'odometry_tum.txt')[0][0], st[0]
    # store_from_live at the anchor: places the run's own map in the stored map.
    k = int(np.argmin(np.abs(lt - anchor)))
    store_from_live = matrix(sp[0], sr[0]) @ np.linalg.inv(matrix(lp[k], lr[k]))

    prior = cloud(args.store / 'map.pcd', 900_000, rng)
    prior_path = g2o_positions(args.store / 'pose_graph.g2o')
    live_map = cloud(args.run / 'map.pcd', 500_000, rng)
    live_map = live_map @ store_from_live[:3, :3].T + store_from_live[:3, 3]
    # Drop ground and roofs so the top view shows walls, trees and poles.
    def band(points, path):
        z = points[:, 2] - np.median(path[:, 2])
        return points[(z > -0.8) & (z < 6.0)]
    prior = band(prior, prior_path)
    live_map = band(live_map, sp)

    xy = np.vstack([prior_path[:, :2], sp[:, :2]])
    lo, hi = xy.min(0) - 25, xy.max(0) + 25
    S = args.size
    header = 92
    scale = min((S - 20) / (hi - lo)[0], (S - header - 20) / (hi - lo)[1])
    W = S
    H = int(round((hi - lo)[1] * scale)) + header + 20

    def px(points):
        u = (points[:, 0] - lo[0]) * scale + 10
        v = H - 10 - (points[:, 1] - lo[1]) * scale
        return np.stack([u, v], 1)

    def raster(points, colour, base, alpha=0.55):
        uv = px(points).astype(int)
        ok = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= header) & (uv[:, 1] < H)
        uv = uv[ok]
        count = np.zeros((H, W))
        np.add.at(count, (uv[:, 1], uv[:, 0]), 1)
        a = np.clip(count / 3.0, 0, 1)[..., None] * alpha
        return base * (1 - a) + np.array(colour) * a

    base = np.full((H, W, 3), BG, dtype=float)
    base = raster(prior, PRIOR, base, 0.6)
    base_img = Image.fromarray(base.astype(np.uint8))

    # Reveal the run's map near the path driven so far.
    tree = cKDTree(live_map[:, :2])
    reveal_at = np.full(len(live_map), np.inf)
    for i in range(0, len(st), 5):
        for j in tree.query_ball_point(sp[i, :2], 30.0):
            if reveal_at[j] == np.inf:
                reveal_at[j] = st[i]

    f_big, f_mid, f_small = (ImageFont.truetype(BOLD, 26), ImageFont.truetype(FONT, 19),
                             ImageFont.truetype(FONT, 16))
    prior_start = prior_path[0, :2]
    anchor_xy = sp[0, :2]
    away = float(np.linalg.norm(anchor_xy - prior_start))

    def decorate(img, status, colour, t_rel):
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, W, header], fill=BG)
        d.text((14, 10), args.title, font=f_big, fill=(235, 238, 242))
        d.text((14, 44), args.subtitle, font=f_small, fill=(160, 168, 180))
        d.text((14, 66), status, font=f_mid, fill=colour)
        d.text((W - 130, 66), f't = {t_rel:6.1f} s', font=f_mid, fill=(200, 205, 212))
        # prior path and its start
        pts = [tuple(p) for p in px(prior_path[:, :2])]
        d.line(pts, fill=PRIOR_PATH, width=2)
        u, v = px(prior_start[None])[0]
        d.ellipse([u - 6, v - 6, u + 6, v + 6], outline=PRIOR_PATH, width=3)
        d.text((u - 30, v - 30), 'map start', font=f_small, fill=PRIOR_PATH)
        # legend
        y = header + 18
        d.rectangle([W - 270, y - 8, W - 8, y + 42], fill=(14, 17, 23))
        d.line([W - 258, y + 4, W - 230, y + 4], fill=PRIOR_PATH, width=3)
        d.text((W - 222, y - 6), 'stored map + its path', font=f_small, fill=(200, 205, 212))
        d.line([W - 258, y + 28, W - 230, y + 28], fill=LIVE, width=3)
        d.text((W - 222, y + 18), 'new run, after anchoring', font=f_small, fill=(200, 205, 212))
        return d

    def inset(img, upto):
        """The run's own odometry before it knows where it is."""
        d = ImageDraw.Draw(img)
        box = (14, header + 12, 214, header + 212)
        d.rectangle(box, fill=(24, 28, 36), outline=(70, 76, 88))
        d.text((box[0] + 8, box[1] + 6), 'own odometry only', font=f_small, fill=(170, 176, 188))
        m = lt <= upto
        pts = lp[m, :2] - lp[0, :2]
        span = max(8.0, np.abs(pts).max() * 2.4 if len(pts) else 8.0)
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2 + 10
        s = 180 / span
        q = [(cx + x * s, cy - y * s) for x, y in pts]
        if len(q) > 1:
            d.line(q, fill=LIVE_PATH, width=2)
        if q:
            d.ellipse([q[-1][0] - 4, q[-1][1] - 4, q[-1][0] + 4, q[-1][1] + 4], fill=LIVE)
        d.text((cx - 6, cy - 46), '?', font=f_big, fill=(255, 120, 90))

    frames = []
    for i in range(args.search_frames):
        t = t0 + (anchor - t0) * (i + 1) / args.search_frames
        img = base_img.copy()
        dots = '.' * (1 + i // 4 % 3)
        decorate(img, f'searching the whole stored map{dots}', (255, 120, 90), t - t0)
        inset(img, t)
        frames.append(img)

    def tracked(t, flash=None):
        arr = np.asarray(base_img).astype(float)
        shown = live_map[reveal_at <= t]
        if len(shown):
            arr = raster(shown, LIVE, arr, 0.75)
        img = Image.fromarray(arr.astype(np.uint8))
        d = decorate(img, f'anchored after {anchor - t0:.1f} s, {away:.0f} m from the map start '
                     '- tracking in the stored map', (120, 220, 140), t - t0)
        m = st <= t
        q = [tuple(p) for p in px(sp[m, :2])]
        if len(q) > 1:
            d.line(q, fill=LIVE_PATH, width=3)
        u, v = q[-1]
        d.ellipse([u - 6, v - 6, u + 6, v + 6], fill=LIVE, outline=(255, 255, 255))
        au, av = px(anchor_xy[None])[0]
        if flash is not None:
            r = 10 + 60 * flash
            d.ellipse([au - r, av - r, au + r, av + r], outline=(120, 220, 140), width=4)
        d.ellipse([au - 8, av - 8, au + 8, av + 8], outline=(120, 220, 140), width=3)
        d.text((au - 150, av - 34), 'anchored here', font=f_small, fill=(120, 220, 140))
        return img

    for i in range(args.flash_frames):
        frames.append(tracked(anchor, flash=i / args.flash_frames))
    for i in range(args.track_frames):
        frames.append(tracked(anchor + (st[-1] - anchor) * (i + 1) / args.track_frames))
    frames += [frames[-1]] * args.hold_frames

    args.out.parent.mkdir(parents=True, exist_ok=True)
    pal = [f.convert('P', palette=Image.ADAPTIVE, colors=128) for f in frames]
    pal[0].save(args.out, save_all=True, append_images=pal[1:], duration=int(1000 / args.fps),
                loop=0, optimize=True)
    print(f'{len(frames)} frames, {W}x{H}, anchored {anchor - t0:.1f} s, {away:.0f} m from start '
          f'-> {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
