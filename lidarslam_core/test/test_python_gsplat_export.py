"""lidarslam.gsplat_export on a small camera log and run written here (no ROS 2 needed).

A rosbag2 with a raw ``bgr8`` image topic, its CameraInfo and a two-link ``/tf_static``; a run
directory with a keyframe trajectory, odometry and a ``binary_compressed`` map. The exported
COLMAP poses must place the camera where the run and the static transforms put it, and the
initial points must take their colour from the pixel they project to.
"""

from __future__ import annotations

from pathlib import Path
import struct

import pytest

pytest.importorskip('lidarslam')
pytest.importorskip('PIL')
import numpy as np  # noqa: E402
from lidarslam import gsplat_export as ge  # noqa: E402
from lidarslam.__main__ import main  # noqa: E402
from rosbags.rosbag2 import Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

TS = get_typestore(Stores.ROS2_JAZZY)
W, H, F = 64, 48, 40.0
# Optical frame (z forward, x right, y down) looking along the base x axis.
OPTICAL = Rotation.from_matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def _pose(xyz, rot: Rotation) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot.as_matrix(), xyz
    return m


BASE_FROM_MOUNT = _pose([0.5, 0.0, 1.0], Rotation.from_euler('z', 10, degrees=True))
MOUNT_FROM_CAM = _pose([0.1, -0.2, 0.0], OPTICAL)


def _tf(parent: str, child: str, m: np.ndarray):
    t = TS.types
    q = Rotation.from_matrix(m[:3, :3]).as_quat()
    return t['geometry_msgs/msg/TransformStamped'](
        header=t['std_msgs/msg/Header'](stamp=t['builtin_interfaces/msg/Time'](sec=10, nanosec=0),
                                        frame_id=parent),
        child_frame_id=child,
        transform=t['geometry_msgs/msg/Transform'](
            translation=t['geometry_msgs/msg/Vector3'](x=m[0, 3], y=m[1, 3], z=m[2, 3]),
            rotation=t['geometry_msgs/msg/Quaternion'](x=q[0], y=q[1], z=q[2], w=q[3])))


def _write_log(path: Path, stamps: list[float]) -> None:
    t = TS.types
    with Writer(path, version=8) as writer:
        tf = writer.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS)
        info_c = writer.add_connection('/cam/camera_info', 'sensor_msgs/msg/CameraInfo',
                                       typestore=TS)
        image_c = writer.add_connection('/cam/image_raw', 'sensor_msgs/msg/Image', typestore=TS)
        # Child-to-parent order on purpose: the tree is walked in both directions.
        msg = t['tf2_msgs/msg/TFMessage'](transforms=[
            _tf('base_link', 'mount', BASE_FROM_MOUNT), _tf('cam_optical', 'mount',
                                                            np.linalg.inv(MOUNT_FROM_CAM))])
        writer.write(tf, 10_000_000_000, TS.serialize_cdr(msg, 'tf2_msgs/msg/TFMessage'))
        for k, stamp in enumerate(stamps):
            header = t['std_msgs/msg/Header'](
                stamp=t['builtin_interfaces/msg/Time'](sec=int(stamp),
                                                       nanosec=int(round(stamp % 1 * 1e9))),
                frame_id='cam_optical')
            info = t['sensor_msgs/msg/CameraInfo'](
                header=header, height=H, width=W, distortion_model='plumb_bob', d=np.zeros(5),
                k=np.array([F, 0, W / 2, 0, F, H / 2, 0, 0, 1.0]), r=np.eye(3).ravel(),
                p=np.array([F, 0, W / 2, 0, 0, F, H / 2, 0, 0, 0, 1, 0.0]),
                binning_x=0, binning_y=0,
                roi=t['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0,
                                                          width=0, do_rectify=False))
            # Left half blue, right half red (bgr8), green row band in the top quarter.
            pixels = np.zeros((H, W, 3), dtype=np.uint8)
            pixels[:, :W // 2, 0] = 255
            pixels[:, W // 2:, 2] = 255
            pixels[:H // 4, :, 1] = 255
            image = t['sensor_msgs/msg/Image'](
                header=header, height=H, width=W, encoding='bgr8', is_bigendian=0, step=W * 3,
                data=pixels.ravel())
            ns = int(stamp * 1e9)
            writer.write(info_c, ns, TS.serialize_cdr(info, 'sensor_msgs/msg/CameraInfo'))
            writer.write(image_c, ns, TS.serialize_cdr(image, 'sensor_msgs/msg/Image'))


def _lzf_literals(data: bytes) -> bytes:
    """LZF of ``data`` with literal runs only (a valid stream any decoder reads)."""
    out = bytearray()
    for i in range(0, len(data), 32):
        chunk = data[i:i + 32]
        out.append(len(chunk) - 1)
        out += chunk
    return bytes(out)


def _write_pcd(path: Path, xyz: np.ndarray) -> None:
    fields = np.zeros((len(xyz), 4), dtype=np.float32)
    fields[:, :3] = xyz
    raw = b''.join(fields[:, i].tobytes() for i in range(4))  # field-major
    compressed = _lzf_literals(raw)
    header = ('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(xyz)}\nDATA binary_compressed\n').encode()
    path.write_bytes(header + struct.pack('<II', len(compressed), len(raw)) + compressed)


def _write_tum(path: Path, rows) -> None:
    lines = []
    for t, m in rows:
        q = Rotation.from_matrix(m[:3, :3]).as_quat()
        lines.append(' '.join(f'{v:.9f}' for v in (t, *m[:3, 3], *q)))
    path.write_text('\n'.join(lines) + '\n')


def _write_run(run: Path) -> dict[float, np.ndarray]:
    """Odometry drifting 0.2 m in y, keyframes that correct it; the true base poses."""
    run.mkdir()
    times = np.arange(100.0, 102.01, 0.5)
    truth = {t: _pose([t - 100.0, 0.0, 0.0], Rotation.from_euler('z', 5 * (t - 100), degrees=True))
             for t in times}
    drift = _pose([0.0, 0.2, 0.0], Rotation.identity())
    _write_tum(run / 'odometry_tum.txt', [(t, drift @ truth[t]) for t in times])
    _write_tum(run / 'trajectory_optimized.tum', [(t, truth[t]) for t in times[::2]])
    # A wall 6 m ahead of the base start: points left and right of the first camera's axis.
    first_cam = truth[100.0] @ BASE_FROM_MOUNT @ MOUNT_FROM_CAM
    in_cam = np.array([[-1.0, 0.5, 6.0], [1.0, 0.5, 6.0], [0.0, -2.0, 6.0], [0.0, 0.0, -6.0]])
    world = (first_cam[:3, :3] @ in_cam.T).T + first_cam[:3, 3]
    _write_pcd(run / 'map.pcd', np.vstack([world, [[500.0, 500.0, 500.0]]]))
    return truth


def _read_images(path: Path) -> list[np.ndarray]:
    poses = []
    for line in path.read_text().splitlines():
        words = line.split()
        if len(words) < 10:
            continue
        qw, qx, qy, qz, tx, ty, tz = map(float, words[1:8])
        cam_from_world = _pose([tx, ty, tz], Rotation.from_quat([qx, qy, qz, qw]))
        poses.append(np.linalg.inv(cam_from_world))
    return poses


def test_export_places_cameras_and_colours_points(tmp_path: Path) -> None:
    stamps = [99.0, 100.0, 100.75, 101.5, 102.0, 103.0]  # the first and last are outside the run
    _write_log(tmp_path / 'log', stamps)
    truth = _write_run(tmp_path / 'run')
    summary = ge.export(tmp_path / 'log', tmp_path / 'run', tmp_path / 'out', '/cam/image_raw')
    assert summary.images == 4
    assert summary.camera == ge.Camera('PINHOLE', W, H, (F, F, W / 2, H / 2))
    assert (tmp_path / 'out' / 'sparse' / '0' / 'cameras.txt').read_text().split()[1] == 'PINHOLE'
    names = sorted(p.name for p in (tmp_path / 'out' / 'images').iterdir())
    assert names == ['000000.jpg', '000001.jpg', '000002.jpg', '000003.jpg']

    poses = _read_images(tmp_path / 'out' / 'sparse' / '0' / 'images.txt')
    base_from_cam = BASE_FROM_MOUNT @ MOUNT_FROM_CAM
    # At keyframe times the pose is the keyframe; in between, odometry moves it from there.
    np.testing.assert_allclose(poses[0], truth[100.0] @ base_from_cam, atol=1e-6)
    np.testing.assert_allclose(poses[3], truth[102.0] @ base_from_cam, atol=1e-6)
    expected = truth[101.0] @ np.linalg.inv(truth[101.0]) @ truth[101.5] @ base_from_cam
    np.testing.assert_allclose(poses[2], expected, atol=1e-6)

    # The wall's points: left of the axis blue, right red, above it green; the point behind
    # the camera and the far one are dropped.
    rows = [line.split() for line in
            (tmp_path / 'out' / 'sparse' / '0' / 'points3D.txt').read_text().splitlines()]
    colours = {(round(float(r[1]), 2), round(float(r[2]), 2), round(float(r[3]), 2)):
               tuple(int(c) for c in r[4:7]) for r in rows}
    assert len(colours) == 3
    got = sorted(colours.values())
    assert got[0][0] < 60 and got[0][2] > 200  # blue
    assert got[-1][0] > 200 and got[-1][2] < 60  # red
    assert any(c[1] > 200 for c in got)  # green band


def test_extrinsic_every_and_scale(tmp_path: Path, capsys: pytest.CaptureFixture) -> None:
    _write_log(tmp_path / 'log', [100.0, 100.5, 101.0, 101.5, 102.0])
    truth = _write_run(tmp_path / 'run')
    base_from_cam = BASE_FROM_MOUNT @ MOUNT_FROM_CAM
    q = Rotation.from_matrix(base_from_cam[:3, :3]).as_quat()
    extrinsic = [str(v) for v in (*base_from_cam[:3, 3], *q)]
    assert main(['colmap', str(tmp_path / 'log'), '--run', str(tmp_path / 'run'),
                 '--image-topic', '/cam/image_raw', '--out', str(tmp_path / 'out'),
                 '--every', '2', '--scale', '0.5', '--extrinsic', *extrinsic]) == 0
    assert '"images": 3' in capsys.readouterr().out
    assert (tmp_path / 'out' / 'sparse' / '0' / 'cameras.txt').read_text().split()[1:4] == \
        ['PINHOLE', str(W // 2), str(H // 2)]
    poses = _read_images(tmp_path / 'out' / 'sparse' / '0' / 'images.txt')
    np.testing.assert_allclose(poses[1], truth[101.0] @ base_from_cam, atol=1e-6)
    stamps = (tmp_path / 'out' / 'image_stamps.txt').read_text().split()
    assert stamps[1::2] == ['100.000000000', '101.000000000', '102.000000000']


def test_camera_models() -> None:
    class Info:
        width, height = 640, 480
        k = [500.0, 0, 320, 0, 505.0, 240, 0, 0, 1]

        def __init__(self, model, d):
            self.distortion_model, self.d = model, d

    assert ge.camera_from_info(Info('plumb_bob', [0.1, -0.2, 0.001, 0.002, 0.0])).model == 'OPENCV'
    full = ge.camera_from_info(Info('plumb_bob', [0.1, -0.2, 0.001, 0.002, 0.05]))
    assert full.model == 'FULL_OPENCV' and full.params[8] == 0.05
    fisheye = ge.camera_from_info(Info('equidistant', [0.1, 0.01, 0.0, 0.0]))
    assert fisheye.model == 'OPENCV_FISHEYE'
    with pytest.raises(ValueError, match='rational'):
        ge.camera_from_info(Info('rational_polynomial', [0, 0, 0, 0, 0, 0.1, 0, 0]))
    # Distortion moves an off-axis point outwards (k1 > 0), never the principal point.
    u, v = full.project(np.array([[0.0, 0.0, 1.0], [0.5, 0.0, 1.0]]))
    assert (u[0], v[0]) == (320.0, 240.0)
    assert u[1] > 320 + 500 * 0.5


def test_lzf_back_reference_and_pcd_layouts(tmp_path: Path) -> None:
    # 'abc' literal, then a back-reference of 6 bytes to offset 3 (overlapping: 'abcabc').
    assert ge._decompress_lzf(bytes([2]) + b'abc' + bytes([(4 << 5) | 0, 2]), 9) == b'abcabcabc'
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    ascii_path = tmp_path / 'a.pcd'
    ascii_path.write_text('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\n'
                          'HEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 3\n4 5 6\n')
    np.testing.assert_allclose(ge.read_pcd_xyz(ascii_path), xyz)
    binary = np.zeros(2, dtype=[('i', '<f4'), ('x', '<f8'), ('y', '<f8'), ('z', '<f8')])
    binary['x'], binary['y'], binary['z'] = xyz.T
    binary_path = tmp_path / 'b.pcd'
    binary_path.write_bytes(b'FIELDS intensity x y z\nSIZE 4 8 8 8\nTYPE F F F F\n'
                            b'COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n' +
                            binary.tobytes())
    np.testing.assert_allclose(ge.read_pcd_xyz(binary_path), xyz)
    _write_pcd(tmp_path / 'c.pcd', xyz)
    np.testing.assert_allclose(ge.read_pcd_xyz(tmp_path / 'c.pcd'), xyz)


def test_missing_transform_and_clock_mismatch(tmp_path: Path) -> None:
    _write_log(tmp_path / 'log', [100.0])
    _write_run(tmp_path / 'run')
    with pytest.raises(KeyError, match='no static transform chain'):
        ge.export(tmp_path / 'log', tmp_path / 'run', tmp_path / 'o1', '/cam/image_raw',
                  base_frame='imu_link')
    _write_log(tmp_path / 'log2', [5.0])
    with pytest.raises(ValueError, match='share a clock'):
        ge.export(tmp_path / 'log2', tmp_path / 'run', tmp_path / 'o2', '/cam/image_raw')
