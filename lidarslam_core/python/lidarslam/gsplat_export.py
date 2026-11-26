"""A COLMAP scene for 3D Gaussian Splatting from a camera log and a lidar_slam run.

Camera poses come from LiDAR SLAM, not from structure from motion: the run's optimised keyframe
poses (``trajectory_optimized.tum``) carry its odometry (``odometry_tum.txt``) between keyframes
to each image's stamp, and the camera's mounting on the frame the run tracks (the odometry's
``base_frame``) comes from the log's ``/tf_static`` or from ``extrinsic``. The initial points are
the run's map (``map.pcd``) near the cameras, coloured from the images. Intrinsics come from the
log's ``sensor_msgs/msg/CameraInfo``.

  OUT/images/<n>.jpg
  OUT/sparse/0/cameras.txt  images.txt  points3D.txt
  OUT/image_stamps.txt      <image name> <stamp in seconds> per image
  OUT/depth_priors/<image name>.bin   with ``depth_lidar_topic``: a LiDAR depth per image

With ``depth_lidar_topic`` each image also gets the LiDAR scan nearest its stamp projected into
it, for depth-supervised 3DGS (visloc-rs ``gsplat_train --depth-prior OUT/depth_priors``): the
scan is placed in the world at its own stamp and seen from the camera at the image's, both by
the run's poses; the nearest point per pixel is kept. The file is ``u32 width, u32 height`` and
then ``width x height`` little-endian ``f32`` camera-z depths in metres, row-major, 0 where no
point falls.

The layout is COLMAP's text model, as 3DGS trainers read it (``gsplat_train --data`` of
visloc-rs, the original 3DGS, nerfstudio's ``colmap`` loader). Images are written as they come
(no undistortion): a distorted camera is written as ``OPENCV``, ``FULL_OPENCV`` (no rational
terms) or ``OPENCV_FISHEYE``.

Needs Pillow (``pip install "lidarslam[gsplat]"``) to decode and write images.
"""

from __future__ import annotations

from dataclasses import dataclass
import io
from pathlib import Path
import struct
from typing import Any, Iterator, Sequence

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

IMAGE = 'sensor_msgs/msg/Image'
COMPRESSED_IMAGE = 'sensor_msgs/msg/CompressedImage'
CAMERA_INFO = 'sensor_msgs/msg/CameraInfo'
TF_MESSAGE = 'tf2_msgs/msg/TFMessage'
POINT_CLOUD2 = 'sensor_msgs/msg/PointCloud2'
_FLOAT_FIELDS = {7: '<f4', 8: '<f8'}  # sensor_msgs/PointField FLOAT32, FLOAT64


@dataclass(frozen=True)
class Camera:
    """A COLMAP camera: model name, image size and parameters (in COLMAP's order)."""

    model: str
    width: int
    height: int
    params: tuple[float, ...]

    def scaled(self, scale: float) -> 'Camera':
        if scale == 1.0:
            return self
        params = list(self.params)
        for i in range(4):  # fx, fy, cx, cy lead every model written here
            params[i] *= scale
        return Camera(self.model, int(round(self.width * scale)), int(round(self.height * scale)),
                      tuple(params))

    def project(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Pixel coordinates (u, v) of camera-frame points (z > 0)."""
        fx, fy, cx, cy = self.params[:4]
        z = np.maximum(points[:, 2], 1e-9)
        x, y = points[:, 0] / z, points[:, 1] / z
        d = self.params[4:]
        if self.model == 'OPENCV_FISHEYE':
            r = np.hypot(x, y)
            theta = np.arctan(r)
            theta2 = theta * theta
            radial = theta * (1 + theta2 * (d[0] + theta2 * (d[1] + theta2 * (
                d[2] + theta2 * d[3]))))
            factor = np.where(r > 1e-12, radial / np.maximum(r, 1e-12), 1.0)
            x, y = x * factor, y * factor
        elif self.model in ('OPENCV', 'FULL_OPENCV'):
            k1, k2, p1, p2 = d[:4]
            k3 = d[4] if self.model == 'FULL_OPENCV' else 0.0
            r2 = x * x + y * y
            radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
            x, y = (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                    y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)
        return fx * x + cx, fy * y + cy

    def in_range(self, points: np.ndarray) -> np.ndarray:
        """Camera-frame points the distortion maps one-to-one (no fold-back from outside)."""
        if self.model not in ('OPENCV', 'FULL_OPENCV'):
            return np.ones(len(points), dtype=bool)
        z = np.maximum(points[:, 2], 1e-9)
        r2 = (points[:, 0] / z) ** 2 + (points[:, 1] / z) ** 2
        k1, k2 = self.params[4:6]
        k3 = self.params[8] if self.model == 'FULL_OPENCV' else 0.0
        return 1 + r2 * (3 * k1 + r2 * (5 * k2 + r2 * 7 * k3)) > 0  # d(r * radial)/dr > 0

    def line(self, camera_id: int = 1) -> str:
        return f'{camera_id} {self.model} {self.width} {self.height} ' + \
            ' '.join(repr(float(p)) for p in self.params)


def camera_from_info(info: Any) -> Camera:
    """The COLMAP camera of a ``sensor_msgs/msg/CameraInfo`` (its ``K`` and ``D``)."""
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    if fx <= 0 or fy <= 0:
        raise ValueError('CameraInfo has no intrinsics (K is zero)')
    d = [float(v) for v in info.d]
    model = (info.distortion_model or '').strip().lower()
    size = (int(info.width), int(info.height))
    if not any(d):
        return Camera('PINHOLE', *size, (fx, fy, cx, cy))
    if model in ('plumb_bob', 'rational_polynomial'):
        d = (d + [0.0] * 8)[:8]
        k1, k2, p1, p2, k3, k4, k5, k6 = d
        if any((k4, k5, k6)):
            raise ValueError('rational_polynomial distortion with k4..k6 is not supported; '
                             'use the rectified image topic')
        if k3 == 0.0:
            return Camera('OPENCV', *size, (fx, fy, cx, cy, k1, k2, p1, p2))
        return Camera('FULL_OPENCV', *size, (fx, fy, cx, cy, k1, k2, p1, p2, k3, 0.0, 0.0, 0.0))
    if model in ('equidistant', 'fisheye'):
        return Camera('OPENCV_FISHEYE', *size, (fx, fy, cx, cy, *(d + [0.0] * 4)[:4]))
    raise ValueError(f'distortion model {info.distortion_model!r} is not supported; '
                     'use the rectified image topic')


def _matrix(translation: Sequence[float], quat_xyzw: Sequence[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    m[:3, 3] = translation
    return m


def static_transform(edges: dict[tuple[str, str], np.ndarray], target: str,
                     source: str) -> np.ndarray:
    """``target_from_source`` through a tree of static transforms.

    ``edges[(parent, child)]`` is ``parent_from_child`` (a ``TransformStamped``'s
    ``header.frame_id`` and ``child_frame_id``); edges are walked in either direction.
    """
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (parent, child), m in edges.items():
        graph.setdefault(parent, []).append((child, m))
        graph.setdefault(child, []).append((parent, np.linalg.inv(m)))
    found = {target: np.eye(4)}
    queue = [target]
    while queue:
        frame = queue.pop(0)
        for other, frame_from_other in graph.get(frame, []):
            if other not in found:
                found[other] = found[frame] @ frame_from_other
                queue.append(other)
    if source not in found:
        raise KeyError(f'no static transform chain from {source!r} to {target!r} '
                       f'(frames: {sorted(graph)})')
    return found[source]


def _strip(frame: str) -> str:
    return frame.lstrip('/')


def _stamp(header: Any) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def decode_image(msg: Any, msgtype: str) -> Any:
    """A Pillow RGB image of a ``sensor_msgs/msg/Image`` or ``CompressedImage``."""
    from PIL import Image
    if msgtype == COMPRESSED_IMAGE:
        return Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')
    encoding = msg.encoding.lower()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    rows = data[:h * step].reshape(h, step)
    if encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        channels = 4 if encoding.endswith('a8') else 3
        pixels = rows[:, :w * channels].reshape(h, w, channels)[:, :, :3]
        if encoding.startswith('bgr'):
            pixels = pixels[:, :, ::-1]
        return Image.fromarray(np.ascontiguousarray(pixels), 'RGB')
    if encoding in ('mono8', '8uc1'):
        return Image.fromarray(np.ascontiguousarray(rows[:, :w]), 'L').convert('RGB')
    if encoding in ('mono16', '16uc1'):
        dtype = '>u2' if msg.is_bigendian else '<u2'
        values = rows[:, :w * 2].copy().view(dtype).reshape(h, w).astype(np.float64)
        top = max(float(values.max()), 1.0)
        return Image.fromarray((values * (255.0 / top)).astype(np.uint8), 'L').convert('RGB')
    raise ValueError(f'image encoding {msg.encoding!r} is not supported '
                     '(rgb8, bgr8, rgba8, bgra8, mono8, mono16 or a CompressedImage)')


def pointcloud_xyz(msg: Any) -> np.ndarray:
    """XYZ of a ``sensor_msgs/msg/PointCloud2`` as float64 (N, 3), non-finite points dropped."""
    fields = {f.name: f for f in msg.fields}
    if any(a not in fields or fields[a].datatype not in _FLOAT_FIELDS for a in 'xyz'):
        raise ValueError('PointCloud2 needs floating-point x, y, z fields')
    if msg.is_bigendian:
        raise ValueError('big-endian PointCloud2 is not supported')
    count = int(msg.width) * int(msg.height)
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if int(msg.height) > 1:
        rows = rows.reshape(int(msg.height), int(msg.row_step))[:, :int(msg.width) * msg.point_step]
    rows = rows.reshape(-1)[:count * msg.point_step].reshape(count, msg.point_step)
    xyz = np.column_stack([
        rows[:, fields[a].offset:fields[a].offset + np.dtype(_FLOAT_FIELDS[fields[a].datatype])
             .itemsize].copy().view(_FLOAT_FIELDS[fields[a].datatype])[:, 0] for a in 'xyz'])
    xyz = xyz.astype(np.float64)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def depth_map(camera: Camera, cam_points: np.ndarray, min_depth: float,
              max_depth: float) -> np.ndarray:
    """Camera-z of the nearest point per pixel (float32, row-major, 0 = no point)."""
    z = cam_points[:, 2]
    keep = (z > min_depth) & (z < max_depth)
    keep[keep] = camera.in_range(cam_points[keep])
    u, v = camera.project(cam_points[keep])
    u, v, z = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64), z[keep]
    inside = (u >= 0) & (u < camera.width) & (v >= 0) & (v < camera.height)
    depth = np.full(camera.width * camera.height, np.inf, dtype=np.float32)
    np.minimum.at(depth, v[inside] * camera.width + u[inside], z[inside].astype(np.float32))
    depth[~np.isfinite(depth)] = 0.0
    return depth


def write_depth_map(path: Path, camera: Camera, depth: np.ndarray) -> None:
    path.write_bytes(np.array([camera.width, camera.height], dtype='<u4').tobytes()
                     + depth.astype('<f4').tobytes())


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray, Rotation]:
    data = np.loadtxt(path, ndmin=2)
    if not len(data):
        raise ValueError(f'{path}: empty trajectory')
    return data[:, 0], data[:, 1:4], Rotation.from_quat(data[:, 4:8])


class RunPoses:
    """Base poses of a run at any time: optimised keyframes, odometry carried in between."""

    def __init__(self, run: Path):
        self.kt, self.kp, self.kr = read_tum(run / 'trajectory_optimized.tum')
        self.ot, self.op, orot = read_tum(run / 'odometry_tum.txt')
        if len(self.ot) < 2:
            raise ValueError(f'{run}: odometry needs two poses or more')
        self._slerp = Slerp(self.ot, orot)

    @property
    def span(self) -> tuple[float, float]:
        return float(self.ot[0]), float(self.ot[-1])

    def _odometry(self, t: float) -> np.ndarray:
        t = float(np.clip(t, self.ot[0], self.ot[-1]))
        m = np.eye(4)
        m[:3, :3] = self._slerp([t]).as_matrix()[0]
        m[:3, 3] = [np.interp(t, self.ot, self.op[:, i]) for i in range(3)]
        return m

    def at(self, t: float) -> np.ndarray:
        k = max(0, int(np.searchsorted(self.kt, t, side='right')) - 1)
        key = np.eye(4)
        key[:3, :3], key[:3, 3] = self.kr[k].as_matrix(), self.kp[k]
        return key @ np.linalg.inv(self._odometry(self.kt[k])) @ self._odometry(t)


def _decompress_lzf(data: bytes, size: int) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        control = data[i]
        i += 1
        if control < 32:
            out += data[i:i + control + 1]
            i += control + 1
            continue
        length = control >> 5
        if length == 7:
            length += data[i]
            i += 1
        start = len(out) - ((control & 0x1F) << 8) - data[i] - 1
        i += 1
        length += 2
        if start < 0:
            raise ValueError('invalid LZF back-reference')
        while length > 0:  # copy in non-overlapping slices (overlap repeats the run)
            chunk = out[start:start + min(length, len(out) - start)]
            out += chunk
            start += len(chunk)
            length -= len(chunk)
    if len(out) != size:
        raise ValueError(f'LZF payload expanded to {len(out)} bytes, expected {size}')
    return bytes(out)


def read_pcd_xyz(path: Path) -> np.ndarray:
    """XYZ of a PCD file (ascii, binary or binary_compressed) as float64 (N, 3)."""
    with Path(path).open('rb') as stream:
        header: dict[str, list[str]] = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f'{path}: PCD header without DATA')
            words = line.decode('ascii', errors='replace').split()
            if not words or words[0].startswith('#'):
                continue
            header[words[0].upper()] = words[1:]
            if words[0].upper() == 'DATA':
                break
        body = stream.read()
    fields = [f.lower() for f in header['FIELDS']]
    sizes = [int(s) for s in header['SIZE']]
    types = [t.upper() for t in header['TYPE']]
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
    points = int(header.get('POINTS', [0])[0]) or \
        int(header['WIDTH'][0]) * int(header.get('HEIGHT', ['1'])[0])
    data = header['DATA'][0].lower()
    index = [fields.index(axis) for axis in 'xyz']
    if any(types[i] != 'F' or sizes[i] not in (4, 8) or counts[i] != 1 for i in index):
        raise ValueError(f'{path}: expected scalar floating-point x, y, z')
    if data == 'ascii':
        rows = np.loadtxt(io.BytesIO(body), ndmin=2)
        columns = np.cumsum([0] + counts[:-1])
        return rows[:points, [columns[i] for i in index]].astype(np.float64)
    widths = [s * c for s, c in zip(sizes, counts)]
    if data == 'binary':
        offsets = np.cumsum([0] + widths[:-1])
        dtype = np.dtype({'names': list('xyz'), 'formats': [f'<f{sizes[i]}' for i in index],
                          'offsets': [int(offsets[i]) for i in index],
                          'itemsize': int(sum(widths))})
        raw = np.frombuffer(body, dtype=dtype, count=points)
        return np.column_stack([raw[a] for a in 'xyz']).astype(np.float64)
    if data == 'binary_compressed':
        compressed, expanded = struct.unpack('<II', body[:8])
        raw = _decompress_lzf(body[8:8 + compressed], expanded)
        offsets = np.cumsum([0] + [w * points for w in widths[:-1]])  # field-major
        return np.column_stack([
            np.frombuffer(raw, dtype=f'<f{sizes[i]}', count=points, offset=int(offsets[i]))
            for i in index]).astype(np.float64)
    raise ValueError(f'{path}: unsupported PCD DATA {data!r}')


@dataclass(frozen=True)
class Summary:
    images: int
    points: int
    camera: Camera
    out: Path
    depth_priors: int = 0
    depth_coverage: float = 0.0


def _messages(reader: AnyReader, topics: set[str]) -> Iterator[tuple[str, str, Any]]:
    connections = [c for c in reader.connections if c.topic in topics]
    for connection, _, raw in reader.messages(connections=connections):
        yield connection.topic, connection.msgtype, reader.deserialize(raw, connection.msgtype)


def export(log: str | Path, run: str | Path, out: str | Path, image_topic: str, *,
           camera_info_topic: str | None = None, base_frame: str = 'base_link',
           camera_frame: str | None = None,
           extrinsic: Sequence[float] | None = None, every: int = 1, scale: float = 1.0,
           points: int = 400_000, max_depth: float = 40.0, min_depth: float = 1.0,
           jpeg_quality: int = 95, depth_lidar_topic: str | None = None,
           lidar_frame: str | None = None, lidar_extrinsic: Sequence[float] | None = None,
           depth_max: float = 80.0, depth_max_gap: float = 0.2) -> Summary:
    """Write the COLMAP scene of ``image_topic`` in ``log`` posed by the lidar_slam run ``run``.

    ``extrinsic`` is ``base_from_camera`` as ``[x, y, z, qx, qy, qz, qw]`` of the camera's
    optical frame (z forward, x right, y down) in ``base_frame``; without it the transform comes
    from ``/tf_static`` (``base_frame`` to ``camera_frame``, by default the CameraInfo's
    ``header.frame_id``). ``camera_info_topic`` defaults to the image topic's sibling
    ``camera_info`` (``.../image_raw[/compressed]`` -> ``.../camera_info``).

    ``depth_lidar_topic`` (a ``sensor_msgs/msg/PointCloud2`` topic) also writes a LiDAR depth per
    image to ``OUT/depth_priors`` from the scan nearest the image stamp (at most
    ``depth_max_gap`` s away; an image without one gets none), z in (``min_depth``,
    ``depth_max``). ``lidar_extrinsic`` is ``base_from_lidar`` as for ``extrinsic``; without it
    the transform comes from ``/tf_static`` (``lidar_frame``, by default the scans' frame_id).
    """
    from PIL import Image  # noqa: F401  (fail early without Pillow)
    log, run, out = Path(log), Path(run), Path(out)
    if every < 1:
        raise ValueError('every must be 1 or more')
    poses = RunPoses(run)
    begin, end = poses.span
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([log], default_typestore=typestore) as reader:
        types = {c.topic: c.msgtype for c in reader.connections}
        if types.get(image_topic) not in (IMAGE, COMPRESSED_IMAGE):
            raise KeyError(f'{image_topic!r} is not an Image or CompressedImage topic in {log}')
        if camera_info_topic is None:
            parts = image_topic.rstrip('/').split('/')
            while parts and parts[-1] in ('compressed', 'image_raw', 'image_rect',
                                          'image_rect_color', 'image_color', 'image'):
                parts.pop()
            camera_info_topic = '/'.join(parts + ['camera_info'])
        if types.get(camera_info_topic) != CAMERA_INFO:
            raise KeyError(f'no CameraInfo topic {camera_info_topic!r} in {log} '
                           '(pass camera_info_topic)')
        info = next(m for _, _, m in _messages(reader, {camera_info_topic}))
        camera = camera_from_info(info)
        edges: dict[tuple[str, str], np.ndarray] | None = None

        def static_edges() -> dict[tuple[str, str], np.ndarray]:
            nonlocal edges
            if edges is None:
                edges = {}
                tf_topics = {t for t, ty in types.items()
                             if ty == TF_MESSAGE and t.endswith('tf_static')}
                for _, _, message in _messages(reader, tf_topics):
                    for tf in message.transforms:
                        q, p = tf.transform.rotation, tf.transform.translation
                        edges[(_strip(tf.header.frame_id), _strip(tf.child_frame_id))] = \
                            _matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            return edges

        def base_from(frame: str, given: Sequence[float] | None, what: str,
                      child: str) -> np.ndarray:
            if given is not None:
                if len(given) != 7:
                    raise ValueError(f'{what} is x y z qx qy qz qw')
                return _matrix(given[:3], given[3:])
            if not static_edges():
                raise KeyError(f'{log} has no /tf_static; pass {what} (base_from_{child})')
            return static_transform(static_edges(), _strip(base_frame), _strip(frame))

        base_from_camera = base_from(camera_frame or info.header.frame_id, extrinsic,
                                     'extrinsic', 'camera')
        if depth_lidar_topic is not None and types.get(depth_lidar_topic) != POINT_CLOUD2:
            raise KeyError(f'{depth_lidar_topic!r} is not a PointCloud2 topic in {log}')

        out_camera = camera.scaled(scale)
        images_dir = out / 'images'
        sparse = out / 'sparse' / '0'
        images_dir.mkdir(parents=True, exist_ok=True)
        sparse.mkdir(parents=True, exist_ok=True)
        names, world_from_cams, stamps = [], [], []
        depth_dir = out / 'depth_priors'
        coverage: list[float] = []
        pending: list[int] = []  # images waiting for the next scan
        previous: tuple[float, np.ndarray] | None = None  # (stamp, world points) of a scan
        base_from_lidar: np.ndarray | None = None

        def write_prior(i: int, scans: list[tuple[float, np.ndarray]]) -> None:
            if not scans:
                return
            t_scan, world = min(scans, key=lambda s: abs(s[0] - stamps[i]))
            if abs(t_scan - stamps[i]) > depth_max_gap:
                return
            pose = world_from_cams[i]
            depth = depth_map(out_camera, (world - pose[:3, 3]) @ pose[:3, :3], min_depth,
                              depth_max)
            write_depth_map(depth_dir / f'{names[i]}.bin', out_camera, depth)
            coverage.append(float((depth > 0).mean()))

        topics = {image_topic} | ({depth_lidar_topic} if depth_lidar_topic else set())
        if depth_lidar_topic:
            depth_dir.mkdir(parents=True, exist_ok=True)
        seen = 0
        for topic, msgtype, msg in _messages(reader, topics):
            t = _stamp(msg.header)
            if topic == depth_lidar_topic and topic != image_topic:
                if not begin - depth_max_gap <= t <= end + depth_max_gap:
                    continue
                if base_from_lidar is None:
                    base_from_lidar = base_from(lidar_frame or msg.header.frame_id,
                                                lidar_extrinsic, 'lidar_extrinsic', 'lidar')
                world_from_lidar = poses.at(t) @ base_from_lidar
                scan = (t, pointcloud_xyz(msg) @ world_from_lidar[:3, :3].T
                        + world_from_lidar[:3, 3])
                for i in [i for i in pending if stamps[i] <= t]:
                    write_prior(i, [s for s in (previous, scan) if s is not None])
                    pending.remove(i)
                previous = scan
                continue
            if not begin <= t <= end:
                continue
            seen += 1
            if (seen - 1) % every:
                continue
            image = decode_image(msg, msgtype)
            if image.size != (camera.width, camera.height):
                raise ValueError(f'image {image.size} does not match CameraInfo '
                                 f'{(camera.width, camera.height)}')
            if scale != 1.0:
                image = image.resize((out_camera.width, out_camera.height), Image.LANCZOS)
            name = f'{len(names):06d}.jpg'
            image.save(images_dir / name, quality=jpeg_quality)
            names.append(name)
            stamps.append(t)
            world_from_cams.append(poses.at(t) @ base_from_camera)
            if depth_lidar_topic:
                pending.append(len(names) - 1)
        for i in pending:
            write_prior(i, [previous] if previous is not None else [])
    if not names:
        raise ValueError(f'no {image_topic} image inside the run span [{begin:.3f}, {end:.3f}] s '
                         '(header stamps and trajectory times must share a clock)')

    (sparse / 'cameras.txt').write_text(out_camera.line() + '\n')
    with (sparse / 'images.txt').open('w') as stream:
        for i, (name, pose) in enumerate(zip(names, world_from_cams), start=1):
            cam_from_world = np.linalg.inv(pose)
            qx, qy, qz, qw = Rotation.from_matrix(cam_from_world[:3, :3]).as_quat()
            values = ' '.join(repr(float(v)) for v in (qw, qx, qy, qz, *cam_from_world[:3, 3]))
            stream.write(f'{i} {values} 1 {name}\n\n')
    (out / 'image_stamps.txt').write_text(
        ''.join(f'{n} {t:.9f}\n' for n, t in zip(names, stamps)))

    cloud = read_pcd_xyz(run / 'map.pcd')
    cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
    centres = np.array([p[:3, 3] for p in world_from_cams])
    near, _ = cKDTree(centres).query(cloud, distance_upper_bound=max_depth)
    cloud = cloud[np.isfinite(near)]
    if len(cloud) > points:
        cloud = cloud[np.random.default_rng(0).choice(len(cloud), points, replace=False)]
    colours = np.zeros((len(cloud), 3), dtype=np.int64)
    coloured = np.zeros(len(cloud), dtype=bool)
    for name, pose in zip(names, world_from_cams):
        cam = (cloud - pose[:3, 3]) @ pose[:3, :3]
        z = cam[:, 2]
        candidate = ~coloured & (z > min_depth) & (z < max_depth)
        if not candidate.any():
            continue
        u, v = out_camera.project(cam[candidate])
        inside = (u >= 0) & (u < out_camera.width) & (v >= 0) & (v < out_camera.height)
        if not inside.any():
            continue
        which = np.flatnonzero(candidate)[inside]
        pixels = np.asarray(Image.open(images_dir / name).convert('RGB'))
        colours[which] = pixels[v[inside].astype(int), u[inside].astype(int)]
        coloured[which] = True
    cloud, colours = cloud[coloured], colours[coloured]
    with (sparse / 'points3D.txt').open('w') as stream:
        for i, (p, c) in enumerate(zip(cloud, colours), start=1):
            stream.write(f'{i} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {c[0]} {c[1]} {c[2]} 0\n')
    return Summary(len(names), len(cloud), out_camera, out, len(coverage),
                   float(np.mean(coverage)) if coverage else 0.0)
