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


def _wall(x0: float, y0: float, x1: float, y1: float, height: float = 2.5) -> dict[str, Any]:
    return {'min': [min(x0, x1), min(y0, y1), 0.0], 'max': [max(x0, x1), max(y0, y1), height]}


def world_ring() -> dict[str, Any]:
    """Return a 5 m wide corridor around a 30 m x 14 m block (a ~100 m loop)."""
    boxes = [{'min': [-1.0, -1.0, -0.1], 'max': [41.0, 25.0, 0.0]}]  # floor
    t = 0.2
    boxes += [_wall(-t, -t, 40 + t, 0), _wall(-t, 24, 40 + t, 24 + t),
              _wall(-t, 0, 0, 24), _wall(40, 0, 40 + t, 24)]
    boxes.append({'min': [5.0, 5.0, 0.0], 'max': [35.0, 19.0, 2.5]})  # the block
    # Crates along the corridor (deterministic), alternating sides.
    crates = [(1.0, 6.0), (4.0, 11.0), (1.0, 17.0), (9.0, 23.0), (16.0, 20.0), (24.0, 23.0),
              (31.0, 20.0), (39.0, 15.0), (36.0, 8.0), (39.0, 3.0), (27.0, 1.0), (20.0, 4.0),
              (12.0, 1.0)]
    for i, (cx, cy) in enumerate(crates):
        size = 0.6 + 0.1 * (i % 3)
        boxes.append({'min': [cx - size / 2, cy - size / 2, 0.0],
                      'max': [cx + size / 2, cy + size / 2, 0.8 + 0.2 * (i % 4)]})
    # Pilasters every 3 m along every wall (0.3 m wide, 0.2 m deep), as door frames and columns
    # of a real corridor: flat walls alone leave the corridor axis unobservable to the LiDAR.
    for k in range(1, 14):
        x = 3.0 * k
        boxes.append({'min': [x - 0.15, 0.0, 0.0], 'max': [x + 0.15, 0.2, 2.5]})
        boxes.append({'min': [x - 0.15, 23.8, 0.0], 'max': [x + 0.15, 24.0, 2.5]})
        if 5.0 < x < 35.0:
            boxes.append({'min': [x - 0.15, 4.8, 0.0], 'max': [x + 0.15, 5.0, 2.5]})
            boxes.append({'min': [x - 0.15, 19.0, 0.0], 'max': [x + 0.15, 19.2, 2.5]})
    for k in range(1, 8):
        y = 3.0 * k
        boxes.append({'min': [0.0, y - 0.15, 0.0], 'max': [0.2, y + 0.15, 2.5]})
        boxes.append({'min': [39.8, y - 0.15, 0.0], 'max': [40.0, y + 0.15, 2.5]})
        if 5.0 < y < 19.0:
            boxes.append({'min': [4.8, y - 0.15, 0.0], 'max': [5.0, y + 0.15, 2.5]})
            boxes.append({'min': [35.0, y - 0.15, 0.0], 'max': [35.2, y + 0.15, 2.5]})
    return {'boxes': boxes, 'start': [2.5, 12.0, math.pi / 2], 'seed': 1,
            'free_region': {'outer': [0, 0, 40, 24], 'holes': [[5, 5, 35, 19]]}}


def world_grid() -> dict[str, Any]:
    """Return 3 m corridors around a 3 x 2 grid of blocks (several loops), 57 m x 35 m."""
    corridor, block_x, block_y = 3.0, 15.0, 13.0
    width = 3 * block_x + 4 * corridor
    height = 2 * block_y + 3 * corridor
    boxes = [{'min': [-1.0, -1.0, -0.1], 'max': [width + 1.0, height + 1.0, 0.0]}]
    t = 0.2
    boxes += [_wall(-t, -t, width + t, 0), _wall(-t, height, width + t, height + t),
              _wall(-t, 0, 0, height), _wall(width, 0, width + t, height)]
    holes = []
    for i in range(3):
        for j in range(2):
            x0 = corridor + i * (block_x + corridor)
            y0 = corridor + j * (block_y + corridor)
            boxes.append({'min': [x0, y0, 0.0], 'max': [x0 + block_x, y0 + block_y, 2.5]})
            holes.append([x0, y0, x0 + block_x, y0 + block_y])
            # Pilasters every 3 m on the block's faces, as door frames.
            for k in range(1, int(block_x // 3)):
                x = x0 + 3.0 * k
                boxes.append({'min': [x - 0.15, y0 - 0.2, 0.0], 'max': [x + 0.15, y0, 2.5]})
                boxes.append({'min': [x - 0.15, y0 + block_y, 0.0],
                              'max': [x + 0.15, y0 + block_y + 0.2, 2.5]})
            for k in range(1, int(block_y // 3)):
                y = y0 + 3.0 * k
                boxes.append({'min': [x0 - 0.2, y - 0.15, 0.0], 'max': [x0, y + 0.15, 2.5]})
                boxes.append({'min': [x0 + block_x, y - 0.15, 0.0],
                              'max': [x0 + block_x + 0.2, y + 0.15, 2.5]})
    for k in range(1, int(width // 3)):  # outer walls
        x = 3.0 * k
        boxes.append({'min': [x - 0.15, 0.0, 0.0], 'max': [x + 0.15, 0.2, 2.5]})
        boxes.append({'min': [x - 0.15, height - 0.2, 0.0], 'max': [x + 0.15, height, 2.5]})
    for k in range(1, int(height // 3)):
        y = 3.0 * k
        boxes.append({'min': [0.0, y - 0.15, 0.0], 'max': [0.2, y + 0.15, 2.5]})
        boxes.append({'min': [width - 0.2, y - 0.15, 0.0], 'max': [width, y + 0.15, 2.5]})
    return {'boxes': boxes, 'start': [1.5, 1.5, 0.0], 'seed': 2,
            'free_region': {'outer': [0, 0, width, height], 'holes': holes}}


WORLDS = {'ring': world_ring, 'grid': world_grid}

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
        x0, y0, x1, y1 = region['outer']
        self.origin = np.array([x0, y0], dtype=float)
        self.res = resolution
        nx, ny = int(math.ceil((x1 - x0) / resolution)), int(math.ceil((y1 - y0) / resolution))
        cx = self.origin[0] + (np.arange(nx) + 0.5) * resolution
        cy = self.origin[1] + (np.arange(ny) + 0.5) * resolution
        gx, gy = np.meshgrid(cx, cy, indexing='ij')
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


def policy_parameters(policy: str, trigger: float, release: float) -> dict[str, Any]:
    if policy == 'nearest_frontier':
        return {'exploration_info_weight': 0.0, 'exploration_distance_weight': 1.0,
                'exploration_revisit_trigger_sigma': 0.0}
    if policy == 'active':
        # A revisit's gain is the predicted fraction of the newest sigma it removes: removing
        # 90 % is worth 36 m of driving at these weights.
        return {'exploration_info_weight': 1.0, 'exploration_distance_weight': 0.5,
                'exploration_uncertainty_weight': 20.0,
                'exploration_revisit_trigger_sigma': trigger,
                'exploration_revisit_release_sigma': release}
    raise ValueError(policy)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', default='ring', choices=sorted(WORLDS))
    parser.add_argument('--policy', default='active', choices=['active', 'nearest_frontier'])
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=400.0, help='time budget')
    parser.add_argument('--revisit-trigger-sigma', type=float, default=0.0,
                        help='active: revisit above this newest-submap sigma (0: never)')
    parser.add_argument('--revisit-release-sigma', type=float, default=0.0)
    parser.add_argument('--inflation-radius', type=float, default=0.8,
                        help='obstacle inflation [m]: robot radius 0.25 m plus the SLAM error')
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
    world = WORLDS[args.world]()
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
                                   args.revisit_release_sigma))
    (out / 'params.json').write_text(json.dumps(
        {'odometry': odometry, 'backend': graph, 'policy': args.policy, 'world': args.world},
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
        if prev_truth is not None:
            distance += math.hypot(tx - prev_truth[0], ty - prev_truth[1])
        prev_truth = (tx, ty)
        collisions += int(header['collided'])

        if 'odometry' not in latest:
            est = (0.0, 0.0, 0.0)
        else:
            o = latest['odometry']
            est = (o[0], o[1], yaw_of(o[3:7]))
        if 'map' in latest:
            m = latest['map']
            errors.append(math.hypot(m[0] - truth[0], m[1] - truth[1]))

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
        if backing_off > 0:
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
                done_count = done_count + 1 if decision['status'] == 'exploration_done' else 0
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
        if done_count >= 3:
            break
    env.close()
    frames.close()
    decisions.close()
    summary = session.finish()
    backend_log.close()
    metrics = {
        'world': args.world, 'policy': args.policy, 'frames': frame + 1,
        'seconds': round((frame + 1) * FRAME_S, 1), 'distance_m': round(distance, 2),
        'coverage': round(coverage.fraction(), 4), 'collisions': collisions,
        'rejected_goals': rejected,
        'exploration_done': done_count >= 3,
        'live_position_error_rmse_m': round(float(np.sqrt(np.mean(np.square(errors)))), 4)
        if errors else None,
        'live_position_error_max_m': round(float(np.max(errors)), 4) if errors else None,
        'submaps': summary.get('submaps') if isinstance(summary, dict) else None,
    }
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(metrics))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
