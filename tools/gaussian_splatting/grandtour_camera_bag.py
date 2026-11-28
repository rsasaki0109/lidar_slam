"""A rosbag2 window of a GrandTour mission with one camera, for `lidarslam colmap`.

    python -I grandtour_camera_bag.py MISSION_Z IMAGES_DIR LIDAR_BAG OUT T0 T1 \
        [--camera hdr_front] [--base stim320_imu] [--lidar hesai_points]

MISSION_Z  the mission's zarr groups (`data/<group>.tar` extracted), with the camera, the base
           (the frame the LiDAR SLAM run tracks as base_link) and the LiDAR
IMAGES_DIR the mission's `images/<camera>.tar` extracted (`<camera>/000000.jpeg`, ...)
LIDAR_BAG  the rosbag2 the run was mapped from; its PointCloud2 topic is copied within T0-1 .. T1+1
T0, T1     the window in seconds (UNIX time, the stamps of the mission)

Writes `/<camera>/image_raw/compressed`, `/<camera>/camera_info` and one `/tf_static` with
base_link -> camera optical frame and base_link -> the scans' frame_id. GrandTour zarr
`transform` attributes are the inverse of box_base -> child, so base <- child is
inv(box <- base) @ box <- child with box <- child = inv(transform).
Needs `pip install rosbags zarr scipy`.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation
import zarr


def box_from(mission: Path, group: str) -> np.ndarray:
    tr = json.loads((mission / group / '.zattrs').read_text())['transform']
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat([tr['rotation'][k] for k in 'xyzw']).as_matrix()
    m[:3, 3] = [tr['translation'][k] for k in 'xyz']
    return np.linalg.inv(m)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('mission', type=Path)
    ap.add_argument('images', type=Path)
    ap.add_argument('lidar_bag', type=Path)
    ap.add_argument('out', type=Path)
    ap.add_argument('t0', type=float)
    ap.add_argument('t1', type=float)
    ap.add_argument('--camera', default='hdr_front')
    ap.add_argument('--base', default='stim320_imu')
    ap.add_argument('--lidar', default='hesai_points')
    a = ap.parse_args()

    ts = get_typestore(Stores.LATEST)
    T = ts.types
    cam = a.camera
    ci = json.loads((a.mission / cam / '.zattrs').read_text())['camera_info']
    base_inv = np.linalg.inv(box_from(a.mission, a.base))
    base_from_cam = base_inv @ box_from(a.mission, cam)
    base_from_lidar = base_inv @ box_from(a.mission, a.lidar)

    def stamp(t):
        sec = int(np.floor(t))
        return T['builtin_interfaces/msg/Time'](sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)

    def header(t, frame):
        return T['std_msgs/msg/Header'](stamp=stamp(t), frame_id=frame)

    def transform(t, child, m):
        q, p = Rotation.from_matrix(m[:3, :3]).as_quat(), m[:3, 3]
        return T['geometry_msgs/msg/TransformStamped'](
            header=header(t, 'base_link'), child_frame_id=child,
            transform=T['geometry_msgs/msg/Transform'](
                translation=T['geometry_msgs/msg/Vector3'](x=p[0], y=p[1], z=p[2]),
                rotation=T['geometry_msgs/msg/Quaternion'](x=q[0], y=q[1], z=q[2], w=q[3])))

    stamps = zarr.open(str(a.mission / cam), mode='r')['timestamp'][:]
    with AnyReader([a.lidar_bag], default_typestore=ts) as r, Writer(a.out, version=8) as w:
        clouds = [c for c in r.connections if c.msgtype == 'sensor_msgs/msg/PointCloud2']
        if len(clouds) != 1:
            raise SystemExit(f'{a.lidar_bag}: expected one PointCloud2 topic, found {len(clouds)}')
        out_cloud = w.add_connection(clouds[0].topic, clouds[0].msgtype, typestore=ts)
        lidar_frame = None
        for c, t, raw in r.messages(connections=clouds):
            if a.t0 - 1 <= t * 1e-9 <= a.t1 + 1:
                if lidar_frame is None:
                    lidar_frame = r.deserialize(raw, c.msgtype).header.frame_id
                w.write(out_cloud, t, raw)
        if lidar_frame is None:
            raise SystemExit(f'{a.lidar_bag}: no scan within {a.t0 - 1} .. {a.t1 + 1}')
        tf = w.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=ts)
        w.write(tf, int(a.t0 * 1e9), ts.serialize_cdr(T['tf2_msgs/msg/TFMessage'](transforms=[
            transform(a.t0, cam, base_from_cam), transform(a.t0, lidar_frame, base_from_lidar)]),
            'tf2_msgs/msg/TFMessage'))
        out_img = w.add_connection(f'/{cam}/image_raw/compressed', 'sensor_msgs/msg/CompressedImage', typestore=ts)
        out_info = w.add_connection(f'/{cam}/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=ts)
        n = 0
        for i, t in enumerate(stamps):
            if not a.t0 <= t <= a.t1:
                continue
            h = header(float(t), cam)
            info = T['sensor_msgs/msg/CameraInfo'](
                header=h, height=ci['height'], width=ci['width'], distortion_model=ci['distortion_model'],
                d=np.array(ci['D'], float), k=np.array(ci['K'], float), r=np.array(ci['R'], float),
                p=np.array(ci['P'], float), binning_x=0, binning_y=0,
                roi=T['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
            img = T['sensor_msgs/msg/CompressedImage'](
                header=h, format='jpeg',
                data=np.frombuffer((a.images / cam / f'{i:06d}.jpeg').read_bytes(), np.uint8))
            w.write(out_info, int(t * 1e9), ts.serialize_cdr(info, 'sensor_msgs/msg/CameraInfo'))
            w.write(out_img, int(t * 1e9), ts.serialize_cdr(img, 'sensor_msgs/msg/CompressedImage'))
            n += 1
    print(f'{n} images, LiDAR frame {lidar_frame}')


if __name__ == '__main__':
    main()
