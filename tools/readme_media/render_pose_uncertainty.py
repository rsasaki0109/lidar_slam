#!/usr/bin/env python3
"""
Render the pose-uncertainty timeline as an animated GIF (CPU only, PIL).

Input is pose_uncertainty_timeline.csv from the offline backend
(pose_uncertainty_timeline:=true, docs/roadmap/active-slam-2026-11.md P1): one block of rows
per graph state, each the submaps so far with their marginal translation sigma. Each frame is
one graph state:

  left:  the submap positions from above, coloured by sigma (one-hue ramp, light = certain,
         dark = uncertain), the newest submap ringed; an optional map (PCD) underneath.
  right: the newest submap's sigma against the number of submaps, with the loop closures
         marked.

The sigma scale is the same in every frame (0 to the largest sigma of the run). Sigmas are in
the units of the backend's edge information weights, so the colour bar says "relative".
Same input, same bytes: no randomness, no wall clock.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Light chart surface, text inks and a one-hue (blue) sequential ramp, steps 250-700 of the
# reference palette (the lightest steps are left out so a certain submap stays visible).
SURFACE = (255, 255, 255)
TEXT_PRIMARY = (25, 25, 25)
TEXT_SECONDARY = (102, 102, 102)
GRID = (230, 230, 230)
AXIS = (179, 179, 179)
MAP_POINT = (214, 214, 214)
SERIES = (37, 125, 224)
RAMP = [
    (0x84, 0xb9, 0xf3), (0x6a, 0xab, 0xf1), (0x51, 0x9c, 0xed), (0x33, 0x8c, 0xed),
    (0x25, 0x7d, 0xe0), (0x23, 0x70, 0xcd), (0x1b, 0x64, 0xbe), (0x17, 0x57, 0xac),
    (0x0f, 0x4a, 0x96), (0x0e, 0x3d, 0x7e),
]
FONT_DIR = Path('/usr/share/fonts/truetype/dejavu')


@dataclass
class GraphState:
    step: int
    stamp_sec: float
    loop_edges: int
    ids: list[int] = field(default_factory=list)
    fixed: list[bool] = field(default_factory=list)
    xy: list[tuple[float, float]] = field(default_factory=list)
    sigma: list[float] = field(default_factory=list)
    loops: list[tuple[int, int]] = field(default_factory=list)

    @property
    def newest_sigma(self) -> float:
        return self.sigma[-1]


def load_timeline(path: Path, loops_path: Path | None = None) -> list[GraphState]:
    """
    Load the graph states of a timeline CSV, in step order.

    Each state gets its loop edges from pose_uncertainty_timeline_loops.csv (by default the
    file next to the timeline, when present).
    """
    states: dict[int, GraphState] = {}
    with open(path, newline='', encoding='utf-8') as handle:
        for row in csv.DictReader(handle):
            step = int(row['step'])
            state = states.get(step)
            if state is None:
                state = GraphState(step, float(row['stamp_sec']), int(row['loop_edges']))
                states[step] = state
            state.ids.append(int(row['id']))
            state.fixed.append(row['fixed'] == '1')
            state.xy.append((float(row['x']), float(row['y'])))
            state.sigma.append(float(row['sigma_translation_max']))
    if not states:
        raise ValueError(f'{path}: no graph states')
    if loops_path is None:
        loops_path = path.with_name('pose_uncertainty_timeline_loops.csv')
    if loops_path.is_file():
        with open(loops_path, newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                state = states.get(int(row['step']))
                if state is not None:
                    state.loops.append((int(row['from']), int(row['to'])))
    return [states[k] for k in sorted(states)]


def loop_closure_steps(states: list[GraphState]) -> list[int]:
    """
    Return the indices of the states that closed a new loop.

    A state closes a new loop when it has a loop edge the state before did not have (also when
    the edge replaced an older one), or, without the loop-edge file, when its loop-edge count
    rose.
    """
    steps = []
    for i in range(1, len(states)):
        if states[i].loops or states[i - 1].loops:
            if set(states[i].loops) - set(states[i - 1].loops):
                steps.append(i)
        elif states[i].loop_edges > states[i - 1].loop_edges:
            steps.append(i)
    return steps


def ramp_color(value: float, top: float) -> tuple[int, int, int]:
    t = 0.0 if top <= 0.0 else min(max(value / top, 0.0), 1.0)
    pos = t * (len(RAMP) - 1)
    i = min(int(pos), len(RAMP) - 2)
    f = pos - i
    a, b = RAMP[i], RAMP[i + 1]
    return tuple(int(round(a[k] + (b[k] - a[k]) * f)) for k in range(3))


def lzf_decompress(data: bytes, size: int) -> bytes:
    """Decompress LZF data (PCD DATA binary_compressed) of a known uncompressed size."""
    out = bytearray(size)
    i = o = 0
    n = len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:  # literal run of ctrl + 1 bytes
            length = ctrl + 1
            out[o:o + length] = data[i:i + length]
            i += length
            o += length
            continue
        length = ctrl >> 5
        if length == 7:
            length += data[i]
            i += 1
        ref = o - ((ctrl & 0x1f) << 8) - data[i] - 1
        i += 1
        length += 2
        if ref < 0 or o + length > size:
            raise ValueError('corrupt LZF data')
        if ref + length <= o:
            out[o:o + length] = out[ref:ref + length]
        else:  # overlapping back-reference: copy byte by byte
            for k in range(length):
                out[o + k] = out[ref + k]
        o += length
    if o != size:
        raise ValueError(f'LZF data decompressed to {o} bytes, expected {size}')
    return bytes(out)


def load_pcd_xy(path: Path, z_min: float, z_max: float, max_points: int) -> np.ndarray:
    """Read x, y of the PCD points with z_min <= z <= z_max (every n-th point, at most max)."""
    raw = path.read_bytes()
    header: dict[str, list[str]] = {}
    offset = 0
    while True:
        end = raw.index(b'\n', offset)
        line = raw[offset:end].decode('ascii').strip()
        offset = end + 1
        if not line or line.startswith('#'):
            continue
        key, *values = line.split()
        header[key.upper()] = values
        if key.upper() == 'DATA':
            break
    names = header['FIELDS']
    sizes = [int(s) for s in header['SIZE']]
    types = header['TYPE']
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(names))]
    points = int(header['POINTS'][0])
    kind = header['DATA'][0]
    if kind == 'ascii':
        table = np.loadtxt(raw[offset:].decode('ascii').splitlines(), ndmin=2)
        columns = np.cumsum([0] + counts)
        xyz = np.stack([table[:, columns[names.index(k)]] for k in 'xyz'], axis=1)
    elif kind == 'binary':
        dtype = np.dtype([(n, {'F': 'f', 'I': 'i', 'U': 'u'}[t] + str(s), (c,))
                          for n, s, t, c in zip(names, sizes, types, counts)])
        table = np.frombuffer(raw, dtype=dtype, count=points, offset=offset)
        xyz = np.stack([table[k][:, 0].astype(np.float64) for k in 'xyz'], axis=1)
    elif kind == 'binary_compressed':
        compressed, size = np.frombuffer(raw, dtype='<u4', count=2, offset=offset)
        data = lzf_decompress(raw[offset + 8:offset + 8 + int(compressed)], int(size))
        # Field by field: all points' first field, then all points' second, ...
        columns = {}
        start = 0
        for n, s, t, c in zip(names, sizes, types, counts):
            width = s * c * points
            if n in 'xyz':
                kind_char = {'F': 'f', 'I': 'i', 'U': 'u'}[t]
                columns[n] = np.frombuffer(data, dtype=f'<{kind_char}{s}', count=points * c,
                                           offset=start)[::c].astype(np.float64)
            start += width
        xyz = np.stack([columns[k] for k in 'xyz'], axis=1)
    else:
        raise ValueError(f'{path}: PCD DATA {kind} is not supported')
    keep = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    xy = xyz[keep, :2]
    if len(xy) > max_points:
        xy = xy[::int(np.ceil(len(xy) / max_points))]
    return xy


class Frame:
    """One frame, drawn at `scale` times the output size and reduced (anti-aliasing)."""

    def __init__(self, width: int, height: int, scale: int) -> None:
        self.s = scale
        self.image = Image.new('RGB', (width * scale, height * scale), SURFACE)
        self.draw = ImageDraw.Draw(self.image)
        self.fonts: dict[tuple[int, bool], ImageFont.FreeTypeFont] = {}

    def font(self, size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
        key = (size, bold)
        if key not in self.fonts:
            name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
            self.fonts[key] = ImageFont.truetype(str(FONT_DIR / name), size * self.s)
        return self.fonts[key]

    def text(self, xy, text, size=13, fill=TEXT_PRIMARY, bold=False, anchor='la') -> None:
        self.draw.text((xy[0] * self.s, xy[1] * self.s), text, fill=fill,
                       font=self.font(size, bold), anchor=anchor)

    def line(self, points, fill, width=1.0) -> None:
        self.draw.line([(x * self.s, y * self.s) for x, y in points], fill=fill,
                       width=max(1, int(round(width * self.s))), joint='curve')

    def circle(self, center, radius, fill=None, outline=None, width=1.0) -> None:
        x, y, r = center[0] * self.s, center[1] * self.s, radius * self.s
        self.draw.ellipse((x - r, y - r, x + r, y + r), fill=fill, outline=outline,
                          width=max(1, int(round(width * self.s))))

    def rect(self, box, fill=None, outline=None) -> None:
        self.draw.rectangle(tuple(v * self.s for v in box), fill=fill, outline=outline)

    def result(self) -> Image.Image:
        return self.image.resize((self.image.width // self.s, self.image.height // self.s),
                                 Image.Resampling.LANCZOS)


@dataclass
class Layout:
    width: int = 960
    height: int = 540
    map_box: tuple[int, int, int, int] = (24, 72, 504, 456)      # left, top, right, bottom
    chart_box: tuple[int, int, int, int] = (588, 96, 932, 392)


def world_to_map(states: list[GraphState], box):
    """Return a uniform-scale transform that fits every submap position in box."""
    xy = np.array([p for s in states for p in s.xy])
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    span = np.maximum(hi - lo, 1e-6)
    left, top, right, bottom = box
    pad = 16
    scale = min((right - left - 2 * pad) / span[0], (bottom - top - 2 * pad) / span[1])
    center_world = (lo + hi) / 2.0
    center_px = np.array([(left + right) / 2.0, (top + bottom) / 2.0])

    def to_px(x: float, y: float) -> tuple[float, float]:
        return (center_px[0] + (x - center_world[0]) * scale,
                center_px[1] - (y - center_world[1]) * scale)  # y up
    return to_px, scale


def select_states(states: list[GraphState], max_frames: int) -> list[int]:
    """
    Pick about max_frames state indices.

    They are evenly spaced, plus every loop closure, the state before it, and the last state.
    """
    if max_frames <= 0 or len(states) <= max_frames:
        return list(range(len(states)))
    keep = set(np.linspace(0, len(states) - 1, max_frames).round().astype(int).tolist())
    for c in loop_closure_steps(states):
        keep.update((c - 1, c))
    keep.add(len(states) - 1)
    return sorted(keep)


def render(states: list[GraphState], title: str, caption: str,
           map_xy: np.ndarray | None = None, scale: int = 2,
           frames_of: list[int] | None = None) -> list[Image.Image]:
    """Draw the frames of the states with the indices frames_of (default: every state)."""
    lay = Layout()
    to_px, px_per_m = world_to_map(states, lay.map_box)
    if frames_of is None:
        frames_of = list(range(len(states)))
    # A dense graph is drawn as a thick line coloured per segment, a sparse one as dots.
    dense = max(len(s.sigma) for s in states) > 150
    radius = 4.5
    top_sigma = max(max(s.sigma) for s in states)
    closures = set(loop_closure_steps(states))
    max_submaps = max(len(s.sigma) for s in states)
    newest = [s.newest_sigma for s in states]
    counts = [len(s.sigma) for s in states]

    map_layer = None
    if map_xy is not None and len(map_xy):
        map_layer = Frame(lay.width, lay.height, scale)
        left, top, right, bottom = lay.map_box
        for x, y in map_xy:
            px, py = to_px(x, y)
            if left <= px <= right and top <= py <= bottom:
                map_layer.circle((px, py), 0.6, fill=MAP_POINT)

    frames = []
    for index in frames_of:
        state = states[index]
        f = Frame(lay.width, lay.height, scale)
        if map_layer is not None:
            f.image.paste(map_layer.image)
        f.text((24, 18), title, size=18, bold=True)
        f.text((24, 44), caption, size=12, fill=TEXT_SECONDARY)

        # Left: submaps from above, loop edges as dashed dark lines.
        f.text((lay.map_box[0], lay.map_box[1] - 6), 'Submaps, top view', size=13, bold=True)
        pts = [to_px(x, y) for x, y in state.xy]
        if dense:
            for k in range(1, len(pts)):
                f.line([pts[k - 1], pts[k]], fill=ramp_color(state.sigma[k], top_sigma),
                       width=5)
        else:
            f.line(pts, fill=AXIS, width=1.5)
        index_of = {vertex_id: k for k, vertex_id in enumerate(state.ids)}
        for a, b in state.loops:
            if a in index_of and b in index_of:
                _dashed(f, pts[index_of[a]], pts[index_of[b]], TEXT_PRIMARY, 1.5)
        for p, sigma, fixed in zip(pts, state.sigma, state.fixed):
            if dense and not fixed:
                continue
            if fixed:
                f.rect((p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4), fill=SURFACE,
                       outline=TEXT_SECONDARY)
            else:
                f.circle(p, radius, fill=ramp_color(sigma, top_sigma), outline=SURFACE, width=1)
        f.circle(pts[-1], radius + 4, outline=TEXT_PRIMARY, width=2)
        # Scale bar (metres) under the map.
        bar_m = _nice_length(60.0 / px_per_m)
        bx, by = lay.map_box[0] + 8, lay.map_box[3] + 13
        f.line([(bx, by), (bx + bar_m * px_per_m, by)], fill=TEXT_SECONDARY, width=2)
        f.text((bx + bar_m * px_per_m + 6, by), f'{bar_m:g} m', size=11, fill=TEXT_SECONDARY,
               anchor='lm')
        # Colour bar.
        cx0, cx1, cy = lay.map_box[0] + 200, lay.map_box[2] - 8, lay.map_box[3] + 8
        steps = 64
        for k in range(steps):
            x0 = cx0 + (cx1 - cx0) * k / steps
            x1 = cx0 + (cx1 - cx0) * (k + 1) / steps
            f.rect((x0, cy, x1, cy + 10), fill=ramp_color((k + 0.5) / steps, 1.0))
        f.text((cx0, cy + 14), '0', size=11, fill=TEXT_SECONDARY)
        f.text((cx1, cy + 14), f'{top_sigma:.2f}', size=11, fill=TEXT_SECONDARY, anchor='ra')
        f.text(((cx0 + cx1) / 2, cy + 14), 'σ translation (relative units)', size=11,
               fill=TEXT_SECONDARY, anchor='ma')

        # Right: the newest submap's sigma against the submap count.
        left, top, right, bottom = lay.chart_box
        f.text((left, top - 30), 'Newest submap σ', size=13, bold=True)
        y_step = _nice_step(max(newest) / 5.0)
        y_ticks = max(1, int(np.ceil(max(newest) / y_step - 1e-9)))
        y_top = y_ticks * y_step

        def chart_px(count: float, value: float) -> tuple[float, float]:
            x = left + (right - left) * (count - 1) / max(1, max_submaps - 1)
            y = bottom - (bottom - top) * value / y_top
            return x, y
        for k in range(y_ticks + 1):
            gy = bottom - (bottom - top) * k / y_ticks
            f.line([(left, gy), (right, gy)], fill=GRID, width=1)
            f.text((left - 6, gy), _tick(y_step * k, y_step), size=10, fill=TEXT_SECONDARY,
                   anchor='rm')
        f.line([(left, bottom), (right, bottom)], fill=AXIS, width=1)
        f.text(((left + right) / 2, bottom + 22), 'submaps in the graph', size=11,
               fill=TEXT_SECONDARY, anchor='ma')
        f.text((right, bottom + 6), f'{max_submaps}', size=10, fill=TEXT_SECONDARY, anchor='ra')
        f.text((left, bottom + 6), '1', size=10, fill=TEXT_SECONDARY, anchor='la')
        for c in sorted(closures):
            if c > index:
                break
            x, _ = chart_px(counts[c], 0.0)
            _dashed(f, (x, top), (x, bottom), AXIS, 1, dash=4.0)
        _dashed(f, (left + 28, bottom + 40), (left + 28, bottom + 52), AXIS, 1, dash=3.0)
        f.text((left + 36, bottom + 46), 'new loop edge', size=11, fill=TEXT_SECONDARY,
               anchor='lm')
        line_pts = [chart_px(counts[k], newest[k]) for k in range(index + 1)]
        if len(line_pts) > 1:
            f.line(line_pts, fill=SERIES, width=2)
        f.circle(line_pts[-1], 4.5, fill=SERIES, outline=SURFACE, width=2)
        f.text((right, bottom + 40), f'{len(state.sigma)} submaps · {state.loop_edges} loop '
               f'edge{"s" if state.loop_edges != 1 else ""}', size=12, fill=TEXT_PRIMARY,
               anchor='ra')
        if index in closures:
            f.text((lay.map_box[2], lay.map_box[1] - 6), 'Loop closed', size=13, bold=True,
                   anchor='ra')
        frames.append(f.result())
    return frames


def _dashed(f: Frame, a, b, fill, width: float, dash: float = 5.0) -> None:
    length = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    n = max(1, int(length // (2 * dash)))
    for k in range(n + 1):
        t0 = min(1.0, 2 * k * dash / max(length, 1e-9))
        t1 = min(1.0, (2 * k + 1) * dash / max(length, 1e-9))
        f.line([(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0),
                (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)], fill=fill, width=width)


def _nice_step(value: float) -> float:
    """Return the smallest of 1, 2, 2.5, 5 x 10^k that is at least value."""
    if value <= 0:
        return 1.0
    exp = int(np.floor(np.log10(value)))
    for e in (exp, exp + 1):
        for base in (1, 2, 2.5, 5):
            if base * 10.0 ** e >= value * (1 - 1e-12):
                return float(base * 10.0 ** e)
    return float(10.0 ** (exp + 1))


def _nice_length(metres: float) -> float:
    """Return the smallest of 1, 2, 5 x 10^k metres at least metres (a scale-bar length)."""
    exp = int(np.floor(np.log10(max(metres, 1e-9))))
    for e in (exp, exp + 1):
        for base in (1, 2, 5):
            if base * 10.0 ** e >= metres:
                return float(base * 10.0 ** e)
    return float(10.0 ** (exp + 1))


def _tick(value: float, step: float) -> str:
    decimals = max(0, -int(np.floor(np.log10(step))) + (1 if step / 10 ** np.floor(
        np.log10(step)) == 2.5 else 0))
    return f'{value:.{decimals}f}'


def frame_durations(states: list[GraphState], ms: int, hold_ms: int,
                    frames_of: list[int] | None = None) -> list[int]:
    """Return each frame's duration: ms, hold_ms for a loop closure, twice that for the last."""
    if frames_of is None:
        frames_of = list(range(len(states)))
    closures = set(loop_closure_steps(states))
    durations = [hold_ms if i in closures else ms for i in frames_of]
    durations[-1] = max(durations[-1], 2 * hold_ms)
    return durations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--timeline', type=Path, required=True,
                        help='pose_uncertainty_timeline.csv')
    parser.add_argument('--out', type=Path, required=True, help='output .gif')
    parser.add_argument('--map', type=Path, help='optional PCD map drawn underneath (ascii, '
                        'binary or binary_compressed)')
    parser.add_argument('--map-z-min', type=float, default=-1e9)
    parser.add_argument('--map-z-max', type=float, default=1e9)
    parser.add_argument('--map-max-points', type=int, default=60000)
    parser.add_argument('--title', default='Pose uncertainty while mapping')
    parser.add_argument('--caption', default='Each submap coloured by its marginal translation '
                        'σ in the pose graph so far')
    parser.add_argument('--frame-ms', type=int, default=180)
    parser.add_argument('--max-frames', type=int, default=150,
                        help='evenly spaced states, plus every loop closure (0: every state)')
    parser.add_argument('--hold-ms', type=int, default=1000)
    parser.add_argument('--frames-dir', type=Path, help='also write each frame as PNG here')
    args = parser.parse_args()

    states = load_timeline(args.timeline)
    map_xy = None
    if args.map is not None:
        map_xy = load_pcd_xy(args.map, args.map_z_min, args.map_z_max, args.map_max_points)
    frames_of = select_states(states, args.max_frames)
    frames = render(states, args.title, args.caption, map_xy, frames_of=frames_of)
    if args.frames_dir is not None:
        args.frames_dir.mkdir(parents=True, exist_ok=True)
        for i, frame in enumerate(frames):
            frame.save(args.frames_dir / f'frame_{i:04d}.png')
    paletted = [frame.convert('P', palette=Image.Palette.ADAPTIVE, colors=96,
                              dither=Image.Dither.NONE) for frame in frames]
    paletted[0].save(args.out, save_all=True, append_images=paletted[1:], loop=0,
                     duration=frame_durations(states, args.frame_ms, args.hold_ms, frames_of),
                     optimize=True, disposal=1)
    print(f'{args.out}: {len(frames)} frames, {len(loop_closure_steps(states))} loop closures')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
