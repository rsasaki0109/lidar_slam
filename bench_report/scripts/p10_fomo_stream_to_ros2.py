#!/usr/bin/env python3
"""FoMo (Foret Montmorency, s3://fomo-dataset, CC BY 4.0) drive, streamed from S3, into a ROS 2 bag (mcap) with the
RoboSense LiDAR and one IMU, plus the rig's RKO-LIO extrinsics.

usage: p10_fomo_stream_to_ros2.py DEPLOYMENT/TRAJECTORY OUT_BAG [--imu vectornav|xsens] [--lidar robosense|leishen]
                                  [--max-lidar N]
                                  [--rig-yaml OUT.yaml] [--summary OUT.json] [--jobs 16]
       e.g. 2025-08-20/red_2025-08-20-10-42

Read (and nothing else; gt.txt, gt_covariance.csv, odom.csv and metadata/ are never fetched here):
  robosense/<us>.bin   one revolution, 26 bytes a point: float32 x, y, z, intensity; uint16 ring (0-127);
                       int64 absolute time [us]. The file name is the first point's time. (The tutorial's loader
                       reads 6 float32 and truncates; the files are not that layout.)
  <imu>.csv            t [us], wx wy wz [rad/s], ax ay az [m/s^2]
  calib/transforms.json  entries {from, to, position, orientation}: the pose of "to" in "from" (T_from_to), e.g.
                       base_link -> footprint is z = -1.114.

--lidar leishen reads leishen/<us>.bin (same 26-byte layout, 4 rings, forward field of view) instead.
Written: /points (sensor_msgs/PointCloud2, frame = the LiDAR's name, stamp = file name; fields x, y, z, intensity float32,
ring uint16, t float32 seconds since the stamp: 22 bytes a point, every point kept) and /imu (sensor_msgs/Imu, frame
= the IMU's name, no orientation), in stamp order. --rig-yaml writes extrinsic_lidar2base and extrinsic_imu2base
(quat xyzw, xyz; T_base_lidar and T_base_imu) for base_frame base_link.
"""
import argparse
import concurrent.futures as cf
import hashlib
import io
import json
import re
import sys
import time
import urllib.parse
import urllib.request

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

BUCKET = 'https://fomo-dataset.s3.us-west-2.amazonaws.com'
LIDAR, IMU = '/points', '/imu'
RAW = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4'), ('ring', '<u2'), ('t', '<i8')])
POINT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('t', '<f4')])


def get(key, tries=6):
    for k in range(tries):
        try:
            with urllib.request.urlopen(f'{BUCKET}/{key}', timeout=120) as r:
                return r.read()
        except Exception:  # noqa: BLE001 - network: retry with backoff, then fail loudly
            if k == tries - 1:
                raise
            time.sleep(2 ** k)


def list_keys(prefix):
    keys, token = [], None
    while True:
        q = f'?list-type=2&prefix={prefix}' + (f'&continuation-token={urllib.parse.quote(token)}' if token else '')
        x = get(q).decode()
        keys += re.findall(r'<Key>([^<]+)</Key>', x)
        m = re.search(r'<NextContinuationToken>([^<]+)</NextContinuationToken>', x)
        if not m:
            return keys
        token = m.group(1)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def chain(transforms, src, dst):
    """T_src_dst from the tree of T_from_to entries (walked in both directions)."""
    edges = {}
    for e in transforms:
        T = np.eye(4)
        o, p = e['orientation'], e['position']
        T[:3, :3] = quat_to_R([o['x'], o['y'], o['z'], o['w']])
        T[:3, 3] = [p['x'], p['y'], p['z']]
        edges.setdefault(e['from'], []).append((e['to'], T))
        edges.setdefault(e['to'], []).append((e['from'], np.linalg.inv(T)))
    stack, seen = [(src, np.eye(4))], {src}
    while stack:
        f, T = stack.pop()
        if f == dst:
            return T
        for t, E in edges.get(f, []):
            if t not in seen:
                seen.add(t)
                stack.append((t, T @ E))
    raise KeyError(f'no chain {src} -> {dst}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('drive')
    ap.add_argument('out')
    ap.add_argument('--imu', default='vectornav', choices=['vectornav', 'xsens'])
    ap.add_argument('--lidar', default='robosense', choices=['robosense', 'leishen'])
    ap.add_argument('--max-lidar', type=int)
    ap.add_argument('--point-scale', type=float, default=1.0,
                    help='diagnostic only: multiply every point by this (a range-scale sensitivity check)')
    ap.add_argument('--rig-yaml')
    ap.add_argument('--summary')
    ap.add_argument('--jobs', type=int, default=16)
    a = ap.parse_args()
    prefix = f'data/{a.drive.strip("/")}/'

    transforms = json.loads(get(prefix + 'calib/transforms.json'))
    rig = {}
    for name, frame in (('lidar', a.lidar), ('imu', a.imu)):
        T = chain(transforms, 'base_link', frame)
        rig[name] = [*R_to_quat(T[:3, :3]).tolist(), *T[:3, 3].tolist()]
    if a.rig_yaml:
        with open(a.rig_yaml, 'w') as f:
            f.write('# FoMo rig keys (p10_fomo_stream_to_ros2.py, from calib/transforms.json of ' + a.drive + ')\n')
            f.write('/**:\n  ros__parameters:\n    base_frame: base_link\n    imu_acceleration_unit: mps2\n')
            for name in ('imu', 'lidar'):
                vals = ', '.join(repr(float(v)) for v in rig[name])
                f.write(f'    extrinsic_{name}2base_quat_xyzw_xyz: [{vals}]\n')

    imu_raw = get(prefix + f'{a.imu}.csv')
    imu = np.loadtxt(io.BytesIO(imu_raw), delimiter=',', skiprows=1, dtype=np.float64, ndmin=2)
    imu_us = np.array([int(line.split(b',', 1)[0]) for line in imu_raw.splitlines()[1:] if line.strip()], np.int64)
    order = np.argsort(imu_us, kind='stable')
    imu_us, imu = imu_us[order], imu[order]

    keys = sorted(k for k in list_keys(prefix + a.lidar + '/') if re.search(r'/\d+\.bin$', k))
    keys.sort(key=lambda k: int(k.rsplit('/', 1)[1][:-4]))
    if a.max_lidar:
        keys = keys[:a.max_lidar]

    store = get_typestore(Stores.ROS2_JAZZY)
    T = store.types
    writer = Writer(a.out, version=9, storage_plugin=StoragePlugin.MCAP)
    writer.open()
    conn = {LIDAR: writer.add_connection(LIDAR, 'sensor_msgs/msg/PointCloud2', typestore=store),
            IMU: writer.add_connection(IMU, 'sensor_msgs/msg/Imu', typestore=store)}
    fields = [T['sensor_msgs/msg/PointField'](name=n, offset=POINT.fields[n][1], datatype=d, count=1)
              for n, d in (('x', 7), ('y', 7), ('z', 7), ('intensity', 7), ('ring', 4), ('t', 7))]
    stats = {LIDAR: {'messages': 0, 'points': 0, 'bad_size': 0, 'max_t_offset_s': 0.0, 'min_t_offset_s': 0.0},
             IMU: {'messages': 0}}
    sha = hashlib.sha256()
    v3, cov = T['geometry_msgs/msg/Vector3'], np.zeros(9)

    def header(us, frame):
        return T['std_msgs/msg/Header'](stamp=T['builtin_interfaces/msg/Time'](sec=int(us // 10**6),
                                                                              nanosec=int(us % 10**6) * 1000),
                                        frame_id=frame)

    def write(topic, us, msg, name):
        writer.write(conn[topic], int(us) * 1000, store.serialize_cdr(msg, name))
        st = stats[topic]
        st['messages'] += 1
        st.setdefault('first_us', int(us))
        st['last_us'] = int(us)

    imu_i = 0

    def flush_imu(until_us):
        nonlocal imu_i
        while imu_i < len(imu_us) and imu_us[imu_i] <= until_us:
            r = imu[imu_i]
            write(IMU, imu_us[imu_i], T['sensor_msgs/msg/Imu'](
                header=header(imu_us[imu_i], a.imu),
                orientation=T['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=cov,
                angular_velocity=v3(x=r[1], y=r[2], z=r[3]), angular_velocity_covariance=cov,
                linear_acceleration=v3(x=r[4], y=r[5], z=r[6]), linear_acceleration_covariance=cov),
                'sensor_msgs/msg/Imu')
            imu_i += 1

    with cf.ThreadPoolExecutor(a.jobs) as pool:
        window = a.jobs * 4
        futs = [pool.submit(get, k) for k in keys[:window]]
        for n, key in enumerate(keys):
            data = futs[n].result()
            futs[n] = None
            if n + window < len(keys):
                futs.append(pool.submit(get, keys[n + window]))
            sha.update(data)
            if len(data) % RAW.itemsize:
                stats[LIDAR]['bad_size'] += 1
                data = data[:len(data) - len(data) % RAW.itemsize]
            p = np.frombuffer(data, RAW)
            us = int(key.rsplit('/', 1)[1][:-4])
            out = np.empty(len(p), POINT)
            for f in ('x', 'y', 'z'):
                out[f] = p[f] * np.float32(a.point_scale) if a.point_scale != 1.0 else p[f]
            out['ring'] = p['ring']
            out['intensity'] = p['i']
            off = (p['t'] - us).astype(np.float64) * 1e-6
            out['t'] = off.astype(np.float32)
            if len(p):
                stats[LIDAR]['max_t_offset_s'] = max(stats[LIDAR]['max_t_offset_s'], float(off.max()))
                stats[LIDAR]['min_t_offset_s'] = min(stats[LIDAR]['min_t_offset_s'], float(off.min()))
            flush_imu(us)
            write(LIDAR, us, T['sensor_msgs/msg/PointCloud2'](
                header=header(us, a.lidar), height=1, width=len(out), fields=fields, is_bigendian=False,
                point_step=POINT.itemsize, row_step=POINT.itemsize * len(out), data=out.view(np.uint8).ravel(),
                is_dense=True), 'sensor_msgs/msg/PointCloud2')
            stats[LIDAR]['points'] += len(out)
            if n % 500 == 0:
                print(f'{n}/{len(keys)}', file=sys.stderr, flush=True)
    flush_imu(stats[LIDAR].get('last_us', 0) + 10**6 if a.max_lidar else 1 << 62)
    writer.close()
    summary = {'drive': a.drive, 'imu': a.imu, 'lidar': a.lidar, 'point_scale': a.point_scale, 'lidar_files': len(keys), 'lidar_sha256_concat': sha.hexdigest(),
               'imu_csv_sha256': hashlib.sha256(imu_raw).hexdigest(), 'rig': rig, 'written': stats}
    text = json.dumps(summary, indent=1)
    print(text)
    if a.summary:
        with open(a.summary, 'w') as f:
            f.write(text + '\n')


if __name__ == '__main__':
    main()
