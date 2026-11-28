#!/usr/bin/env python3
"""HDMapping (Mandeye) raw data of the MapsHD HUMANOID-LIO-DATASET -> ROS 2 bag (MCAP), without Docker or ROS.

Port of mapshd_to_ros2.py to rosbags (pure Python; the same messages and stamps). Unlike the original, the IMU and the
clouds are written interleaved in log-time order (the original wrote all IMU messages first; readers sort by log time).

Mirrors MapsHD/mandeye_to_bag's mandeye_to_rosbag1_node (the converter the benchmark uses before its own ROS 1/2
steps), except that the cloud is written as PointCloud2 with a per-point time:
  /livox/imu        sensor_msgs/Imu, frame livox: every imu*.csv row with timestamp != 0 (stamp = timestamp / 1e9),
                    angular velocity and acceleration as stored (the converter does not change units either);
  /livox/pointcloud sensor_msgs/PointCloud2, frame livox: x y z intensity t (float32). The points of all lidar*.laz
                    files in order, timestamp (GPS time, s) != 0, cut like the converter's CustomMsg: after each
                    5th point (its line counter) once a message holds more than 19,968 points; the last partial
                    message is dropped as there. The header stamp is the first point's time and t is seconds since
                    it (0 for a point earlier than that, like the converter's offset_time). The message is
                    stored at its last point's time (when a driver would deliver it).
Usage: mapshd_to_ros2.py <hdmapping-raw dir> <out_bag_dir>
"""
import glob
import os
import sys

import laspy
import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

STORE = get_typestore(Stores.ROS2_JAZZY)
T = STORE.types

RAW, OUT = sys.argv[1], sys.argv[2]
NUM_POINT = 19968
LINE_COUNT = 4


def stamp(ns):
    return T['builtin_interfaces/msg/Time'](sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def imu_rows(path):
    """Rows of one imu*.csv as (stamp_ns, gyro xyz, acc xyz), the converter's non-legacy format."""
    with open(path) as f:
        header = f.readline().replace(',', ' ').split()
        cols = {name: i for i, name in enumerate(header)}
        need = ['timestamp', 'gyroX', 'gyroY', 'gyroZ', 'accX', 'accY', 'accZ']
        assert all(n in cols for n in need), (path, header)
        for line in f:
            v = line.replace(',', ' ').split()
            if not v:
                continue
            ts = float(v[cols['timestamp']])
            if ts == 0:
                continue
            yield (int(round(ts)), [float(v[cols[n]]) for n in need[1:4]], [float(v[cols[n]]) for n in need[4:7]])


def cloud(points_xyz, intensity, t_ns):
    t0 = int(t_ns[0])
    out = np.empty(len(t_ns), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4'), ('t', 'f4')])
    out['x'], out['y'], out['z'] = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]
    out['intensity'] = intensity
    # Like the converter's offset_time: a point earlier than the first one of its message gets 0.
    out['t'] = (np.maximum(t_ns - t0, 0) * 1e-9).astype(np.float32)
    fields = [T['sensor_msgs/msg/PointField'](name=n, offset=4 * i, datatype=7, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity', 't'))]
    msg = T['sensor_msgs/msg/PointCloud2'](
        header=T['std_msgs/msg/Header'](stamp=stamp(t0), frame_id='livox'), height=1, width=len(out), fields=fields,
        is_bigendian=False, point_step=20, row_step=20 * len(out), data=out.view(np.uint8).ravel(), is_dense=True)
    return msg


def main():
    writer = Writer(OUT, version=9, storage_plugin=StoragePlugin.MCAP)
    writer.open()
    conn = {'/livox/pointcloud': writer.add_connection('/livox/pointcloud', 'sensor_msgs/msg/PointCloud2',
                                                       typestore=STORE),
            '/livox/imu': writer.add_connection('/livox/imu', 'sensor_msgs/msg/Imu', typestore=STORE)}
    rows = [r for path in sorted(glob.glob(os.path.join(RAW, 'imu*.csv'))) for r in imu_rows(path)]
    rows.sort(key=lambda r: r[0])
    imu = 0
    v3 = T['geometry_msgs/msg/Vector3']
    cov = np.zeros(9)
    ocov = cov.copy()
    ocov[0] = -1.0

    def flush_imu(until_ns):
        nonlocal imu
        while imu < len(rows) and rows[imu][0] <= until_ns:
            ns, gyro, acc = rows[imu]
            msg = T['sensor_msgs/msg/Imu'](
                header=T['std_msgs/msg/Header'](stamp=stamp(ns), frame_id='livox'),
                orientation=T['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=ocov,
                angular_velocity=v3(x=gyro[0], y=gyro[1], z=gyro[2]), angular_velocity_covariance=cov,
                linear_acceleration=v3(x=acc[0], y=acc[1], z=acc[2]), linear_acceleration_covariance=cov)
            writer.write(conn['/livox/imu'], ns, STORE.serialize_cdr(msg, 'sensor_msgs/msg/Imu'))
            imu += 1
    # Points: keep the converter's message boundaries across files.
    pending_xyz, pending_i, pending_t = [], [], []
    held = 0
    line_id = 0
    scans = points = dropped_zero = 0
    for path in sorted(glob.glob(os.path.join(RAW, 'lidar*.laz'))):
        las = laspy.read(path)
        xyz = np.column_stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)])
        gps = np.asarray(las.gps_time, dtype=np.float64)
        inten = np.asarray(las.intensity, dtype=np.float32)
        keep = gps != 0
        dropped_zero += int((~keep).sum())
        xyz, gps, inten = xyz[keep], gps[keep], inten[keep]
        t_ns = np.round(gps * 1e9).astype(np.int64)
        start = 0
        n = len(t_ns)
        # Message boundary rule of the converter, vectorized: the counter wraps after each 5th point, and a
        # message is emitted at the first wrap where it holds more than NUM_POINT points.
        while start < n:
            # points still needed before the message can be emitted (strictly more than NUM_POINT)
            need = max(0, NUM_POINT + 1 - held)
            # index (within this file) of the first wrap at or after taking `need` more points
            first = start + max(need, 1) - 1
            # wraps happen when the point taken has line_id == LINE_COUNT before increment
            offset = (LINE_COUNT - (line_id + (first - start)) % (LINE_COUNT + 1)) % (LINE_COUNT + 1)
            end = first + offset  # inclusive index of the point after which the message is emitted
            if end >= n:
                pending_xyz.append(xyz[start:]); pending_i.append(inten[start:]); pending_t.append(t_ns[start:])
                line_id = (line_id + (n - start)) % (LINE_COUNT + 1)
                held += n - start
                break
            pending_xyz.append(xyz[start:end + 1]); pending_i.append(inten[start:end + 1])
            pending_t.append(t_ns[start:end + 1])
            line_id = 0
            m_xyz, m_i, m_t = np.concatenate(pending_xyz), np.concatenate(pending_i), np.concatenate(pending_t)
            msg = cloud(m_xyz, m_i, m_t)
            flush_imu(int(m_t.max()))
            writer.write(conn['/livox/pointcloud'], int(m_t.max()), STORE.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2'))
            scans += 1
            points += len(m_t)
            pending_xyz, pending_i, pending_t = [], [], []
            held = 0
            start = end + 1
        print(f'{os.path.basename(path)}: {scans} scans so far', flush=True)
    flush_imu(1 << 62)
    writer.close()
    print(f'done: {scans} scans, {points} points, {imu} imu samples, {dropped_zero} zero-time points, '
          f'{held} trailing points dropped', flush=True)


if __name__ == '__main__':
    main()
