#!/usr/bin/env python3
"""Write a COLMAP scene for 3D Gaussian Splatting from a lidar_slam run on a KITTI raw drive.

Camera poses come from LiDAR SLAM, not from SfM: the run's optimised keyframe poses
(``trajectory_optimized.tum``) carry its odometry (``odometry_tum.txt``) between keyframes to each
image time, and the KITTI calibration places the rectified left colour camera (``image_02``) on
the OXTS body frame the run tracks. The initial points are the run's map (``map.pcd``), coloured
from the images. The layout is the one ``visloc-rs`` ``gsplat_train --data`` reads:

  OUT/images/<frame>.jpg
  OUT/sparse/0/cameras.txt  images.txt  points3D.txt

  export_kitti_colmap.py DRIVE_DIR RUN_DIR OUT [--points 400000] [--max-depth 40] [--scale 0.5]
         [--first-frame N --last-frame M]

DRIVE_DIR is the output of fetch_kitti_raw_drive.py (image_02/, calib/); RUN_DIR the output of
``python -m lidarslam live`` on its bag.
"""

import argparse
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_luh_map_update import xyz_chunks  # noqa: E402


def calib(path):
    values = {}
    for line in path.read_text().splitlines():
        if ':' in line:
            key, _, rest = line.partition(':')
            try:
                values[key] = np.array(rest.split(), dtype=float)
            except ValueError:
                pass
    return values


def rigid(rotation, translation):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = np.reshape(rotation, (3, 3)), translation
    return m


def camera_on_base(calib_dir):
    """Rectified image_02 camera pose in the OXTS body frame, and its PINHOLE intrinsics."""
    imu_velo = calib(calib_dir / 'calib_imu_to_velo.txt')
    velo_cam = calib(calib_dir / 'calib_velo_to_cam.txt')
    cams = calib(calib_dir / 'calib_cam_to_cam.txt')
    velo_from_imu = rigid(imu_velo['R'], imu_velo['T'])
    cam0_from_velo = rigid(velo_cam['R'], velo_cam['T'])
    rect = np.eye(4)
    rect[:3, :3] = cams['R_rect_00'].reshape(3, 3)
    p2 = cams['P_rect_02'].reshape(3, 4)
    shift = np.eye(4)
    shift[0, 3] = p2[0, 3] / p2[0, 0]
    cam_from_imu = shift @ rect @ cam0_from_velo @ velo_from_imu
    width, height = cams['S_rect_02'].astype(int)
    return np.linalg.inv(cam_from_imu), (width, height, p2[0, 0], p2[1, 1], p2[0, 2], p2[1, 2])


def stamps(path):
    out = []
    for line in path.read_text().strip().splitlines():
        day, clock = line.strip().split(' ')
        whole, fraction = clock.split('.')
        moment = datetime.strptime(f'{day} {whole}', '%Y-%m-%d %H:%M:%S').replace(tzinfo=timezone.utc)
        out.append(moment.timestamp() + int(fraction.ljust(9, '0')[:9]) * 1e-9)
    return np.array(out)


def tum(path):
    data = np.loadtxt(path, ndmin=2)
    return data[:, 0], data[:, 1:4], Rotation.from_quat(data[:, 4:8])


def poses_at(times, run):
    """Base poses in the map frame: optimised keyframes, odometry in between."""
    kt, kp, kr = tum(run / 'trajectory_optimized.tum')
    ot, op, orot = tum(run / 'odometry_tum.txt')
    slerp = Slerp(ot, orot)

    def odom(t):
        t = np.clip(t, ot[0], ot[-1])
        m = np.eye(4)
        m[:3, :3] = slerp([t]).as_matrix()[0]
        m[:3, 3] = [np.interp(t, ot, op[:, i]) for i in range(3)]
        return m

    out = []
    for t in times:
        k = max(0, np.searchsorted(kt, t, side='right') - 1)
        key = np.eye(4)
        key[:3, :3], key[:3, 3] = kr[k].as_matrix(), kp[k]
        out.append(key @ np.linalg.inv(odom(kt[k])) @ odom(t))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('drive', type=Path)
    parser.add_argument('run', type=Path)
    parser.add_argument('out', type=Path)
    parser.add_argument('--points', type=int, default=400000)
    parser.add_argument('--max-depth', type=float, default=40.)
    parser.add_argument('--scale', type=float, default=1.0,
                        help='resize the images (and intrinsics) by this factor')
    parser.add_argument('--first-frame', type=int, default=0)
    parser.add_argument('--last-frame', type=int, default=10**9)
    args = parser.parse_args()

    base_from_cam, (width, height, fx, fy, cx, cy) = camera_on_base(args.drive / 'calib')
    all_stamps = stamps(args.drive / 'image_02' / 'timestamps.txt')
    files = sorted((args.drive / 'image_02').glob('*.jpg'))
    frames = [int(f.stem) for f in files]
    ot, _, _ = tum(args.run / 'odometry_tum.txt')
    keep = [i for i, f in enumerate(frames) if ot[0] <= all_stamps[f] <= ot[-1]
            and args.first_frame <= f <= args.last_frame]
    files, frames = [files[i] for i in keep], [frames[i] for i in keep]
    world_from_cam = [p @ base_from_cam for p in poses_at(all_stamps[frames], args.run)]
    if args.scale != 1.0:
        width, height = int(round(width * args.scale)), int(round(height * args.scale))
        fx, fy, cx, cy = (v * args.scale for v in (fx, fy, cx, cy))

    images = args.out / 'images'
    sparse = args.out / 'sparse' / '0'
    images.mkdir(parents=True, exist_ok=True)
    sparse.mkdir(parents=True, exist_ok=True)
    (sparse / 'cameras.txt').write_text(f'1 PINHOLE {width} {height} {fx} {fy} {cx} {cy}\n')
    with (sparse / 'images.txt').open('w') as out:
        for i, (path, pose) in enumerate(zip(files, world_from_cam), start=1):
            cam_from_world = np.linalg.inv(pose)
            qx, qy, qz, qw = Rotation.from_matrix(cam_from_world[:3, :3]).as_quat()
            t = cam_from_world[:3, 3]
            out.write(f'{i} {qw} {qx} {qy} {qz} {t[0]} {t[1]} {t[2]} 1 {path.name}\n\n')
            if args.scale == 1.0:
                shutil.copy(path, images / path.name)
            else:
                Image.open(path).convert('RGB').resize((width, height), Image.LANCZOS).save(
                    images / path.name, quality=95)

    points = np.concatenate([c.astype(np.float64) for c in xyz_chunks(args.run / 'map.pcd')])
    # Keep only map points near this segment's cameras before sampling.
    centres = np.array([pose[:3, 3] for pose in world_from_cam])
    near, _ = cKDTree(centres).query(points, distance_upper_bound=args.max_depth)
    points = points[np.isfinite(near)]
    rng = np.random.default_rng(0)
    if len(points) > args.points:
        points = points[rng.choice(len(points), args.points, replace=False)]
    colours = np.zeros((len(points), 3), dtype=np.int64)
    seen = np.zeros(len(points), dtype=bool)
    for path, pose in zip(files, world_from_cam):
        cam = (points - pose[:3, 3]) @ pose[:3, :3]
        z = cam[:, 2]
        u = fx * cam[:, 0] / np.maximum(z, 1e-6) + cx
        v = fy * cam[:, 1] / np.maximum(z, 1e-6) + cy
        visible = ~seen & (z > 1.) & (z < args.max_depth) & (u >= 0) & (u < width) & (v >= 0) & \
            (v < height)
        if not visible.any():
            continue
        pixels = np.asarray(Image.open(images / path.name).convert('RGB'))
        colours[visible] = pixels[v[visible].astype(int), u[visible].astype(int)]
        seen |= visible
    points, colours = points[seen], colours[seen]
    with (sparse / 'points3D.txt').open('w') as out:
        for i, (p, c) in enumerate(zip(points, colours), start=1):
            out.write(f'{i} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {c[0]} {c[1]} {c[2]} 0\n')
    print(f'{len(files)} images, {len(points)} coloured points -> {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
