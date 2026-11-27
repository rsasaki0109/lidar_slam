#!/usr/bin/env python3
r"""Closed-loop active SLAM in simulation: explore a world with lidar_slam choosing the goals.

The loop, one 0.1 s LiDAR frame at a time (lockstep, deterministic):

  rne_active_slam_env (Rust, RobotNativeEngine's Mid-360 model + IMU, kinematic robot)
      -> Mid-360 cloud and IMU messages, ground-truth pose (for the metrics only)
  lidarslam.live.Session (RKO-LIO + graph_based_slam)
      -> the robot's odometry pose, and decide_exploration(): the next goal and its path
  a pure-pursuit controller on the SLAM's own pose
      -> the speed and yaw-rate command for the next frame

Policies: ``active`` (unknown area, path length and, while the newest submap's sigma is above
the trigger, revisits) and ``nearest_frontier`` (path length only, no revisits). Writes
metrics.json (coverage, distance, ATE), frames.csv and decisions.jsonl to the output directory,
plus the backend's files (trajectories, pose_uncertainty_timeline.csv, ...).

  python3 tools/rne_active_slam_env/run_closed_loop.py --world ring --policy active \\
      --out /tmp/active_ring
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ENV_BINARY = Path(__file__).resolve().parent / 'target' / 'release' / 'rne_active_slam_env'
POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                        ('tag', 'u1'), ('line', 'u1'), ('timestamp', '<f8')])
IMU_DTYPE = np.dtype([('stamp', '<i8'), ('gyro', '<f8', 3), ('accel', '<f8', 3)])
FLOAT32, UINT8, FLOAT64 = 7, 2, 8
SENSOR_HEIGHT_M = 0.5
FRAME_S = 0.1
ACCEL_SCALE = 1.0  # the env sends g, as the Mid-360 driver does

# ------------------------------------------------------------------------------------- worlds


from worlds import world_for, WORLDS  # noqa: E402  (this directory)

# ------------------------------------------------------------------------------------- env


class Env:
    """The rne_active_slam_env process: one step() per 0.1 s frame."""

    def __init__(self, world_file: Path):
        self.proc = subprocess.Popen([str(ENV_BINARY), str(world_file)], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE)

    def step(self, v: float, w: float) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
        assert self.proc.stdin is not None and self.proc.stdout is not None
        self.proc.stdin.write(f'step {v:.6f} {w:.6f}\n'.encode())
        self.proc.stdin.flush()
        header = json.loads(self.proc.stdout.readline())
        points = np.frombuffer(self.proc.stdout.read(header['points'] * POINT_DTYPE.itemsize),
                               POINT_DTYPE)
        imu = np.frombuffer(self.proc.stdout.read(header['imu'] * IMU_DTYPE.itemsize), IMU_DTYPE)
        return header, points, imu

    def close(self) -> None:
        if self.proc.poll() is None:
            assert self.proc.stdin is not None
            self.proc.stdin.write(b'quit\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)

# ------------------------------------------------------------------------------------- messages


def _header(core: Any, stamp_ns: int, frame_id: str) -> Any:
    header = core.Header()
    header.stamp.sec = int(stamp_ns // 1_000_000_000)
    header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    header.frame_id = frame_id
    return header


def cloud_message(core: Any, stamp_ns: int, points: np.ndarray) -> Any:
    cloud = core.PackedCloud()
    cloud.header = _header(core, stamp_ns, 'livox_frame')
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [core.PointField('x', 0, FLOAT32, 1), core.PointField('y', 4, FLOAT32, 1),
                    core.PointField('z', 8, FLOAT32, 1),
                    core.PointField('intensity', 12, FLOAT32, 1),
                    core.PointField('tag', 16, UINT8, 1), core.PointField('line', 17, UINT8, 1),
                    core.PointField('timestamp', 18, FLOAT64, 1)]
    cloud.is_bigendian = False
    cloud.point_step = POINT_DTYPE.itemsize
    cloud.row_step = POINT_DTYPE.itemsize * len(points)
    cloud.data = memoryview(points.tobytes()).cast('B')
    cloud.is_dense = True
    return cloud


def imu_message(core: Any, sample: np.void) -> Any:
    imu = core.ImuSample()
    imu.header = _header(core, int(sample['stamp']), 'livox_frame')
    imu.orientation = [0.0, 0.0, 0.0, 1.0]
    imu.orientation_covariance = [-1.0] + [0.0] * 8
    imu.angular_velocity = [float(v) for v in sample['gyro']]
    imu.angular_velocity_covariance = [0.0] * 9
    imu.linear_acceleration = [float(v) * ACCEL_SCALE for v in sample['accel']]
    imu.linear_acceleration_covariance = [0.0] * 9
    return imu

# ------------------------------------------------------------------------------------- control


def yaw_of(q: list[float]) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pure_pursuit(pose: tuple[float, float, float], path: list[tuple[float, float]],
                 lookahead: float = 1.0, speed: float = 0.7) -> tuple[float, float, int]:
    """Return (v, w, index of the path point nearest the robot) to follow path."""
    x, y, yaw = pose
    pts = np.asarray(path)
    nearest = int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y)))
    target = pts[-1]
    for k in range(nearest, len(pts)):
        if math.hypot(pts[k, 0] - x, pts[k, 1] - y) >= lookahead:
            target = pts[k]
            break
    error = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
    w = max(-0.5, min(0.5, 1.5 * error))
    v = 0.0 if abs(error) > 1.0 else speed * math.cos(error) ** 2
    return v, w, nearest

# ------------------------------------------------------------------------------------- metrics


class Coverage:
    """Floor cells seen (a floor return landed in them) over the world's free floor."""

    def __init__(self, region: dict[str, Any], resolution: float = 0.5):
        if 'free_rects' in region:
            rects = region['free_rects']
            x0, y0 = min(r[0] for r in rects), min(r[1] for r in rects)
            x1, y1 = max(r[2] for r in rects), max(r[3] for r in rects)
        else:
            x0, y0, x1, y1 = region['outer']
        self.origin = np.array([x0, y0], dtype=float)
        self.res = resolution
        nx, ny = int(math.ceil((x1 - x0) / resolution)), int(math.ceil((y1 - y0) / resolution))
        cx = self.origin[0] + (np.arange(nx) + 0.5) * resolution
        cy = self.origin[1] + (np.arange(ny) + 0.5) * resolution
        gx, gy = np.meshgrid(cx, cy, indexing='ij')
        if 'free_rects' in region:
            free = np.zeros((nx, ny), dtype=bool)
            for fx0, fy0, fx1, fy1 in region['free_rects']:
                free |= (gx > fx0) & (gx < fx1) & (gy > fy0) & (gy < fy1)
        else:
            free = np.ones((nx, ny), dtype=bool)
            for hx0, hy0, hx1, hy1 in region['holes']:
                free &= ~((gx > hx0) & (gx < hx1) & (gy > hy0) & (gy < hy1))
        self.free = free
        self.seen = np.zeros_like(free)

    def add(self, truth: tuple[float, float, float], points: np.ndarray) -> None:
        x, y, yaw = truth
        floor = points[points['z'] < -SENSOR_HEIGHT_M + 0.1]
        if len(floor) == 0:
            return
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * floor['x'] - s * floor['y']
        wy = y + s * floor['x'] + c * floor['y']
        ix = np.floor((wx - self.origin[0]) / self.res).astype(int)
        iy = np.floor((wy - self.origin[1]) / self.res).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < self.free.shape[0]) & (iy < self.free.shape[1])
        self.seen[ix[ok], iy[ok]] = True

    def fraction(self) -> float:
        return float((self.seen & self.free).sum() / self.free.sum())

# ------------------------------------------------------------------------------------- main


POLICIES = ('nearest_frontier', 'frontier_info', 'active')


def densify(waypoints: list[list[float]], step: float = 0.25) -> list[tuple[float, float]]:
    """Return points every step metres along a polyline."""
    out: list[tuple[float, float]] = [tuple(waypoints[0])]
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n + 1)]
    return out


def odometry_drift(truth: list[tuple[float, float, float]],
                   odometry: list[tuple[float, float, float]], distances: list[float],
                   segment_m: float = 50.0) -> float | None:
    """Mean translational drift of the odometry over segment_m of travel, in % (KITTI-style).

    For each start frame, the odometry's displacement over the next segment_m of travel,
    expressed in the start pose, against the truth's; None when the run is shorter.
    """
    errors = []
    j = 0
    for i in range(0, len(truth), 5):
        while j < len(truth) and distances[j] - distances[i] < segment_m:
            j += 1
        if j >= len(truth):
            break
        rel = []
        for track in (truth, odometry):
            (xi, yi, ai), (xj, yj, _) = track[i], track[j]
            c, s = math.cos(ai), math.sin(ai)
            dx, dy = xj - xi, yj - yi
            rel.append((c * dx + s * dy, -s * dx + c * dy))
        errors.append(math.hypot(rel[0][0] - rel[1][0], rel[0][1] - rel[1][1]) /
                      (distances[j] - distances[i]))
    return round(100.0 * float(np.mean(errors)), 4) if errors else None


def save_grid_png(grid: dict[str, Any], track: list[tuple[float, float]], path: Path,
                  scale: int = 4) -> None:
    """Write the explorer's coverage grid (unknown grey, free white, occupied black) and the
    estimated track (red) to a PNG, for looking at why a run ended."""
    from PIL import Image, ImageDraw
    width, height, res = grid['width'], grid['height'], grid['resolution']
    if width == 0 or height == 0:
        return
    cells = np.frombuffer(bytes(grid['cells']), dtype=np.uint8).reshape(height, width)
    palette = np.array([[150, 150, 150], [255, 255, 255], [0, 0, 0]], dtype=np.uint8)
    image = Image.fromarray(palette[np.clip(cells, 0, 2)][::-1]).resize(
        (width * scale, height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)

    def pixel(x: float, y: float) -> tuple[float, float]:
        return ((x / res - grid['x0']) * scale, (height - (y / res - grid['y0'])) * scale)
    if len(track) > 1:
        draw.line([pixel(x, y) for x, y in track], fill=(220, 30, 30), width=2)
    image.save(path)


def policy_parameters(policy: str, trigger: float, release: float, uncertainty_weight: float,
                      min_predicted_drop: float, absolute_gain: bool = False) -> dict[str, Any]:
    """Return the backend's exploration parameters of a policy (see the roadmap's P0)."""
    if policy == 'nearest_frontier':
        return {'exploration_info_weight': 0.0, 'exploration_distance_weight': 1.0,
                'exploration_revisit_trigger_sigma': 0.0}
    frontier_info = {'exploration_info_weight': 1.0, 'exploration_distance_weight': 0.5,
                     'exploration_revisit_trigger_sigma': 0.0}
    if policy == 'frontier_info':
        return frontier_info
    if policy == 'active':
        # A revisit's gain is the predicted fraction of the newest sigma it removes, or with
        # absolute_gain the predicted sigma removed [m].
        return {**frontier_info, 'exploration_uncertainty_weight': uncertainty_weight,
                'exploration_revisit_absolute_gain': absolute_gain,
                'exploration_revisit_trigger_sigma': trigger,
                'exploration_revisit_release_sigma': release,
                'exploration_min_predicted_drop': min_predicted_drop}
    raise ValueError(policy)


def final_map_ate(tum_path: Path, truth_stamps: list[float],
                  truth_xy: list[tuple[float, float]]) -> dict[str, Any]:
    """Return the optimized submap positions' RMSE against the truth at their stamps.

    ``rmse_m`` after an SE(2) alignment (rotation and translation, least squares), ``raw_rmse_m``
    without. The truth is interpolated linearly between frame ends.
    """
    if not tum_path.is_file():
        return {'rmse_m': None, 'raw_rmse_m': None, 'poses': 0}
    data = np.loadtxt(tum_path, ndmin=2)
    if len(data) == 0:
        return {'rmse_m': None, 'raw_rmse_m': None, 'poses': 0}
    stamps = np.asarray(truth_stamps)
    txy = np.asarray(truth_xy)
    keep = (data[:, 0] >= stamps[0]) & (data[:, 0] <= stamps[-1])
    est = data[keep, 1:3]
    ref = np.stack([np.interp(data[keep, 0], stamps, txy[:, 0]),
                    np.interp(data[keep, 0], stamps, txy[:, 1])], axis=1)
    raw = float(np.sqrt(np.mean(np.sum((est - ref) ** 2, axis=1))))
    mu_e, mu_r = est.mean(axis=0), ref.mean(axis=0)
    cov = (ref - mu_r).T @ (est - mu_e)
    u, _, vt = np.linalg.svd(cov)
    d = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    rot = u @ d @ vt
    aligned = (est - mu_e) @ rot.T + mu_r
    rmse = float(np.sqrt(np.mean(np.sum((aligned - ref) ** 2, axis=1))))
    return {'rmse_m': round(rmse, 4), 'raw_rmse_m': round(raw, 4), 'poses': int(keep.sum())}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', default='ring', choices=sorted(WORLDS))
    parser.add_argument('--policy', default='active', choices=POLICIES)
    parser.add_argument('--seed', type=int, default=1,
                        help="start pose (seed %% 10 of the world's starts) and noise seeds")
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=600.0, help='time budget')
    parser.add_argument('--divergence-m', type=float, default=2.0,
                        help='stop as failed when the live position error exceeds this')
    parser.add_argument('--uncertainty-weight', type=float, default=20.0,
                        help="active: weight of a revisit's predicted sigma fraction removed")
    parser.add_argument('--min-predicted-drop', type=float, default=0.2)
    parser.add_argument('--absolute-gain', action='store_true',
                        help="active: score a revisit by the sigma it removes [m], "
                        "not the fraction")
    parser.add_argument('--revisit-trigger-sigma', type=float, default=0.0,
                        help='active: revisit above this newest-submap sigma (0: never)')
    parser.add_argument('--revisit-release-sigma', type=float, default=0.0)
    parser.add_argument('--inflation-radius', type=float, default=0.8,
                        help='obstacle inflation [m]: robot radius 0.25 m plus the SLAM error')
    parser.add_argument('--pilaster-spacing', type=int, default=0,
                        help='difficulty: keep pilasters only every this many metres '
                        '(0: all, -1: none)')
    parser.add_argument('--imu-noise-scale', type=float, default=1.0,
                        help='difficulty: scale of the simulated IMU noise and bias')
    parser.add_argument('--route', action='store_true',
                        help="calibration: drive the world's fixed route on the true pose "
                        '(no exploration decisions; --policy is ignored)')
    parser.add_argument('--rko-param', type=Path,
                        default=ROOT / 'lidarslam' / 'param' / 'rko_lio_mid360_walking.yaml',
                        help='RKO-LIO parameters (the indoor preset; on the simulated scans the '
                        'car and handheld-outdoor presets slide 1-2 m along a corridor when '
                        'the robot stops and turns in place)')
    parser.add_argument('--lidarslam-param', type=Path,
                        default=ROOT / 'lidarslam' / 'param' /
                        'lidarslam_mid360_rko_graph_walking.yaml',
                        help='graph_based_slam parameters (the indoor preset, as the odometry; '
                        'the car preset only searches loops 100 m of travel apart)')
    args = parser.parse_args(argv)

    from lidarslam import _core, backend
    from lidarslam.live import Session
    sys.path.insert(0, str(ROOT / 'scripts'))
    from run_rko_lio_graph_core import odometry_parameters

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    world = world_for(args.world, args.seed, args.pilaster_spacing, args.imu_noise_scale,
                      route=args.route)
    route_path = densify(world['route']) if args.route else []
    world_file = out / 'world.json'
    world_file.write_text(json.dumps(world, indent=1) + '\n', encoding='utf-8')

    odometry = odometry_parameters(args.rko_param.resolve(), '/livox/lidar', '/livox/imu',
                                   'livox_frame', None, None)
    # The env sends the acceleration in g, as the Mid-360 driver does; let RKO-LIO detect the
    # unit (the walking preset does already; the others were tuned on m/s^2 data).
    odometry['imu_acceleration_unit'] = 'auto'
    graph = backend.load_parameters(args.lidarslam_param.resolve())
    graph.update({'pose_uncertainty_timeline': True, 'active_exploration_timeline': True,
                  'exploration_inflation_radius': args.inflation_radius})
    graph.update(policy_parameters(args.policy, args.revisit_trigger_sigma,
                                   args.revisit_release_sigma, args.uncertainty_weight,
                                   args.min_predicted_drop, args.absolute_gain))
    (out / 'params.json').write_text(json.dumps(
        {'odometry': odometry, 'backend': graph, 'policy': args.policy, 'world': args.world,
         'seed': args.seed},
        indent=2, sort_keys=True, default=str) + '\n', encoding='utf-8')

    latest: dict[str, Any] = {}
    backend_log = open(out / 'backend.log', 'w', encoding='utf-8')
    session = Session(odometry, graph, out / 'slam', on_pose=lambda p: latest.update(p),
                      log=lambda level, msg: backend_log.write(f'[{level}] {msg}\n'))
    env = Env(world_file)
    start = world['start']
    c0, s0 = math.cos(start[2]), math.sin(start[2])
    coverage = Coverage(world['free_region'])
    frames = open(out / 'frames.csv', 'w', encoding='utf-8')
    frames.write('frame,t,truth_x,truth_y,truth_yaw,est_x,est_y,est_yaw,v,w,status,kind,'
                 'coverage,collided\n')
    decisions = open(out / 'decisions.jsonl', 'w', encoding='utf-8')

    v = w = 0.0
    backing_off = 0  # frames left of backing away after a collision
    goal_collisions = 0  # collisions while heading for the current goal
    rejected = 0
    path: list[tuple[float, float]] = []
    decision: dict[str, Any] = {}
    frames_since_decision = 10**9
    done_count = 0
    distance = 0.0
    errors = []
    prev_truth = None
    collisions = 0
    status = 'init'
    truth_stamps: list[float] = []
    truth_xy: list[tuple[float, float]] = []
    diverged = False
    coverage_90 = None  # (seconds, metres) when 90 % of the floor was first seen
    revisit_frames = 0
    est_track: list[tuple[float, float]] = []
    distances: list[float] = []  # travelled distance at each frame
    odometry_distances: list[float] = []
    odometry_track: list[tuple[float, float, float]] = []
    truth_track: list[tuple[float, float, float]] = []
    max_frames = int(args.max_seconds / FRAME_S)
    settle_frames = 30  # stand still while RKO-LIO initializes
    for frame in range(max_frames):
        header, points, imu = env.step(v, w)
        for sample in imu:
            session.add_imu(imu_message(_core, sample))
        # As the real driver stamps a frame: with its first point's time.
        cloud_stamp = int(points['timestamp'].min()) if len(points) else header['stamp_ns']
        session.add_lidar(cloud_message(_core, cloud_stamp, points))
        tx, ty, tyaw = header['truth']
        # Truth in the start frame (the odometry frame starts at the start pose).
        dx, dy = tx - start[0], ty - start[1]
        truth = (c0 * dx + s0 * dy, -s0 * dx + c0 * dy, wrap(tyaw - start[2]))
        coverage.add((tx, ty, tyaw), points)
        # The truth returned is the pose at the frame's end.
        truth_stamps.append((header['stamp_ns'] + int(FRAME_S * 1e9)) * 1e-9)
        truth_xy.append((truth[0], truth[1]))
        if prev_truth is not None:
            distance += math.hypot(tx - prev_truth[0], ty - prev_truth[1])
        prev_truth = (tx, ty)
        distances.append(distance)
        collisions += int(header['collided'])

        if 'odometry' not in latest:
            est = (0.0, 0.0, 0.0)
        else:
            o = latest['odometry']
            est = (o[0], o[1], yaw_of(o[3:7]))
            odometry_track.append(est)
            truth_track.append(truth)
            odometry_distances.append(distance)
        if 'map' in latest:
            m = latest['map']
            est_track.append((m[0], m[1]))
            errors.append(math.hypot(m[0] - truth[0], m[1] - truth[1]))
            if errors[-1] > args.divergence_m:
                diverged = True

        v = w = 0.0
        if header['collided']:
            backing_off = 8  # back away 0.8 s, then plan again
            path = []
            goal_collisions += 1
            if goal_collisions >= 3 and decision.get('status') == 'goal':
                # As a navigation stack aborts a goal it cannot reach: the explorer drops it.
                session.reject_exploration_goal(*decision['goal'])
                rejected += 1
                goal_collisions = 0
        if args.route:
            if frame >= settle_frames and route_path:
                # Only the next stretch: a route passes the same place more than once.
                v, w, nearest = pure_pursuit((tx, ty, tyaw), route_path[:40])
                route_path = route_path[nearest:]
                if len(route_path) <= 4 or len(route_path) < 40 and math.hypot(
                        route_path[-1][0] - tx, route_path[-1][1] - ty) < 0.5:
                    route_path = []
            elif frame >= settle_frames:
                break  # the route is done
        elif backing_off > 0:
            backing_off -= 1
            v = -0.3
        elif frame >= settle_frames and 'odometry' in latest:
            reached = bool(path) and math.hypot(path[-1][0] - est[0], path[-1][1] - est[1]) < 0.6
            if not path or reached or frames_since_decision >= 10 or header['collided']:
                previous_goal = decision.get('goal')
                decision = session.decide_exploration(est[0], est[1])
                if previous_goal is None or math.hypot(
                        decision['goal'][0] - previous_goal[0],
                        decision['goal'][1] - previous_goal[1]) > 0.5:
                    goal_collisions = 0
                frames_since_decision = 0
                path = ([tuple(p) for p in decision['path']]
                        if decision['status'] == 'goal' and len(decision['path']) > 1 else [])
                record = {k: decision[k] for k in decision if k != 'path'}
                record.update({'frame': frame, 'path_points': len(decision['path'])})
                decisions.write(json.dumps(record) + '\n')
                # Nothing left to explore and no revisit target reachable: the mission is over.
                done_count = (done_count + 1 if decision['status'] in
                              ('exploration_done', 'no_reachable_goal') else 0)
            frames_since_decision += 1
            status = decision.get('status', 'init')
            if path and len(path) > 1:
                v, w, _ = pure_pursuit(est, path)
            elif path:
                v, w = 0.0, 0.0
                path = []
        frames.write(f'{frame},{frame * FRAME_S:.1f},{truth[0]:.4f},{truth[1]:.4f},'
                     f'{truth[2]:.5f},{est[0]:.4f},{est[1]:.4f},{est[2]:.5f},{v:.3f},{w:.3f},'
                     f'{status},{decision.get("kind", "")},{coverage.fraction():.4f},'
                     f'{int(header["collided"])}\n')
        revisit_frames += int(path != [] and decision.get('kind') == 'revisit')
        if coverage_90 is None and coverage.fraction() >= 0.9:
            coverage_90 = ((frame + 1) * FRAME_S, distance)
        if done_count >= 3 or diverged:
            break
    try:
        save_grid_png(session.exploration_grid(), est_track, out / 'exploration_grid.png')
    except (ImportError, KeyError, ValueError) as error:
        print(f'warning: no exploration_grid.png: {error}', file=sys.stderr)
    env.close()
    frames.close()
    decisions.close()
    session.finish()
    backend_log.close()
    ate = final_map_ate(out / 'slam' / 'trajectory_optimized.tum', truth_stamps, truth_xy)
    loop_file = out / 'slam' / 'loop_edges.csv'
    loop_edges = (len(loop_file.read_text().splitlines()) - 1) if loop_file.is_file() else None
    cov = coverage.fraction()
    metrics = {
        'world': args.world, 'policy': args.policy, 'seed': args.seed, 'frames': frame + 1,
        'seconds': round((frame + 1) * FRAME_S, 1), 'distance_m': round(distance, 2),
        'coverage': round(cov, 4),
        'final_map_ate_rmse_m': ate['rmse_m'], 'final_map_ate_raw_rmse_m': ate['raw_rmse_m'],
        'final_map_poses': ate['poses'],
        'live_position_error_rmse_m': round(float(np.sqrt(np.mean(np.square(errors)))), 4)
        if errors else None,
        'live_position_error_max_m': round(float(np.max(errors)), 4) if errors else None,
        'coverage_90_seconds': coverage_90[0] if coverage_90 else None,
        'coverage_90_distance_m': round(coverage_90[1], 2) if coverage_90 else None,
        'loop_edges': loop_edges, 'revisit_goal_frames': revisit_frames,
        'odometry_drift_50m_pct': odometry_drift(truth_track, odometry_track,
                                                 odometry_distances),
        'pilaster_spacing_m': args.pilaster_spacing, 'imu_noise_scale': args.imu_noise_scale,
        'route': args.route,
        'collisions': collisions, 'rejected_goals': rejected,
        'exploration_done': done_count >= 3, 'diverged': diverged,
        'failed': diverged or cov < 0.5,
    }
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(metrics))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
