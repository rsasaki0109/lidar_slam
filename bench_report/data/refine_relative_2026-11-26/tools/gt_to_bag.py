"""GrandTour zarr (one Hesai LiDAR + one IMU) -> rosbag2 with rosbags (no ROS 2),
the same messages as p1_grandtour_to_ros2.py's convert (fields x y z intensity t, stamps, order)."""
import heapq, sys
import numpy as np
import zarr
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import iter_scans, imu_topic, lidar_topic  # noqa: E402
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

mission, out, group, lidar_group = sys.argv[1:5]
ts = get_typestore(Stores.ROS2_JAZZY); T = ts.types
def stamp(s):
    sec = int(np.floor(s)); return T['builtin_interfaces/msg/Time'](sec=sec, nanosec=int(round((s - sec) * 1e9)) % 1000000000)
lidar = zarr.open(f'{mission}/{lidar_group}', mode='r')
lidar_frame = dict(lidar.attrs).get('frame_id', lidar_group)
imu = zarr.open(f'{mission}/{group}', mode='r')
imu_frame = dict(imu.attrs).get('frame_id', group)
it, acc, gyro = imu['timestamp'][:], imu['lin_acc'][:], imu['ang_vel'][:]
V3, Q = T['geometry_msgs/msg/Vector3'], T['geometry_msgs/msg/Quaternion']
cov = np.zeros(9); ocov = np.zeros(9); ocov[0] = -1.0
def imu_msgs():
    for i in range(len(it)):
        m = T['sensor_msgs/msg/Imu'](header=T['std_msgs/msg/Header'](stamp=stamp(it[i]), frame_id=imu_frame),
            orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=ocov,
            angular_velocity=V3(x=float(gyro[i][0]), y=float(gyro[i][1]), z=float(gyro[i][2])), angular_velocity_covariance=cov,
            linear_acceleration=V3(x=float(acc[i][0]), y=float(acc[i][1]), z=float(acc[i][2])), linear_acceleration_covariance=cov)
        yield float(it[i]), 0, imu_topic(group), m
PF = T['sensor_msgs/msg/PointField']
fields = [PF(name=n, offset=4 * k, datatype=7, count=1) for k, n in enumerate(('x', 'y', 'z', 'intensity', 't'))]
kept = [0]
def clouds():
    window = []
    for xyz, t, inten in iter_scans(lidar):
        t0 = float(t.min())
        c = np.empty((len(t), 5), np.float32); c[:, :3] = xyz; c[:, 3] = inten; c[:, 4] = (t - t0).astype(np.float32)
        m = T['sensor_msgs/msg/PointCloud2'](header=T['std_msgs/msg/Header'](stamp=stamp(t0), frame_id=lidar_frame),
            height=1, width=len(t), fields=fields, is_bigendian=False, point_step=20, row_step=20 * len(t),
            data=c.view(np.uint8).ravel(), is_dense=True)
        window.append((float(t.max()), kept[0], lidar_topic(lidar_group), m)); kept[0] += 1
        if len(window) > 8:
            window.sort(key=lambda w: (w[0], w[1])); f = window.pop(0); yield f[0], 1, f[2], f[3]
    window.sort(key=lambda w: (w[0], w[1]))
    for w in window: yield w[0], 1, w[2], w[3]
with Writer(out, version=8) as wr:
    ci = wr.add_connection(imu_topic(group), 'sensor_msgs/msg/Imu', typestore=ts)
    cl = wr.add_connection(lidar_topic(lidar_group), 'sensor_msgs/msg/PointCloud2', typestore=ts)
    for t, kind, topic, m in heapq.merge(imu_msgs(), clouds(), key=lambda x: x[0]):
        wr.write(ci if kind == 0 else cl, int(round(t * 1e9)), ts.serialize_cdr(m, m.__msgtype__))
print(f'done: {kept[0]} scans, {len(it)} imu samples')
