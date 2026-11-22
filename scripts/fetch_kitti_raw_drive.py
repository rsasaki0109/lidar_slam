#!/usr/bin/env python3
"""Fetch one KITTI raw drive for LiDAR SLAM and 3D Gaussian Splatting, without the full archives.

KITTI raw (www.cvlibs.net/datasets/kitti/raw_data.php, CC BY-NC-SA 3.0) serves each drive as a
zip of 10-20 GB. The host answers HTTP range requests, so this reads the zips' directories and
pulls only the members it needs:

  - from ``<drive>_sync.zip``: the left colour camera (``image_02``), re-encoded as JPEG with
    ``--image-step`` (every N-th frame) and ``--jpeg-quality``, and its timestamps; the Velodyne
    scans (``velodyne_points``), written to a ROS 2 bag as ``/kitti/velo`` (x, y, z, intensity,
    float32; with ``--voxel`` only one point per voxel is kept);
  - from ``<drive>_extract.zip``: the 100 Hz OXTS packets, written to the same bag as
    ``/kitti/imu`` (sensor_msgs/Imu, OXTS body frame: x forward, y left, z up);
  - from ``<date>_calib.zip``: the calibration files, copied as they are.

``--first`` / ``--count`` select a frame window of the synchronised data. Output layout:

  OUT/bag/              ROS 2 bag (zstd), LiDAR + IMU
  OUT/image_02/*.jpg    camera frames, named by frame index
  OUT/image_02/timestamps.txt   (all frames, as in KITTI)
  OUT/calib/*.txt

  fetch_kitti_raw_drive.py 2011_09_30 0018 OUT [--first 0 --count 1200 --image-step 2 --voxel 0.1]
"""

import argparse
import io
import sys
import time
import urllib.request
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image
from rosbags.rosbag2 import CompressionFormat, CompressionMode, Writer
from rosbags.typesys import Stores, get_typestore

BASE = 'https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data'
FLOAT32 = 7


class HttpFile(io.RawIOBase):
    """Seekable read-only file over HTTP range requests, with a small read-ahead cache."""

    def __init__(self, url, block=8 << 20):
        self.url, self.block, self.pos = url, block, 0
        request = urllib.request.Request(url, method='HEAD')
        with urllib.request.urlopen(request, timeout=60) as response:
            self.size = int(response.headers['Content-Length'])
        self.cache_start, self.cache = 0, b''

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=0):
        self.pos = {0: offset, 1: self.pos + offset, 2: self.size + offset}[whence]
        return self.pos

    def _fetch(self, start, end):
        for attempt in range(8):
            try:
                request = urllib.request.Request(self.url, headers={'Range': f'bytes={start}-{end - 1}'})
                deadline = time.monotonic() + 180
                chunks = []
                with urllib.request.urlopen(request, timeout=60) as response:
                    while chunk := response.read(1 << 20):
                        chunks.append(chunk)
                        if time.monotonic() > deadline:
                            raise TimeoutError('range read too slow')
                data = b''.join(chunks)
                if len(data) != end - start:
                    raise OSError(f'short read {len(data)}')
                return data
            except OSError as error:
                print(f'range {start}-{end}: {error}; retrying', file=sys.stderr)
                time.sleep(2 ** attempt)
        raise RuntimeError('too many HTTP errors')

    def read(self, size=-1):
        if size < 0:
            size = self.size - self.pos
        size = max(0, min(size, self.size - self.pos))
        end = self.pos + size
        if not (self.cache_start <= self.pos and end <= self.cache_start + len(self.cache)):
            fetch_end = min(self.size, max(end, self.pos + self.block))
            self.cache, self.cache_start = self._fetch(self.pos, fetch_end), self.pos
        offset = self.pos - self.cache_start
        data = self.cache[offset:offset + size]
        self.pos += len(data)
        return data

    def readinto(self, buffer):
        data = self.read(len(buffer))
        buffer[:len(data)] = data
        return len(data)


def stamps(text):
    """KITTI timestamps.txt lines (local time with nanoseconds) to ns since the epoch."""
    out = []
    for line in text.decode().strip().splitlines():
        day, clock = line.strip().split(' ')
        whole, fraction = clock.split('.')
        moment = datetime.strptime(f'{day} {whole}', '%Y-%m-%d %H:%M:%S').replace(tzinfo=timezone.utc)
        out.append(int(moment.timestamp()) * 1_000_000_000 + int(fraction.ljust(9, '0')[:9]))
    return out


def voxel_downsample(points, size):
    if size <= 0:
        return points
    keys = np.floor(points[:, :3] / size).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def cloud_message(typestore, stamp, points):
    types = typestore.types
    fields = [types['sensor_msgs/msg/PointField'](name=n, offset=o, datatype=FLOAT32, count=1)
              for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
    header = types['std_msgs/msg/Header'](
        stamp=types['builtin_interfaces/msg/Time'](sec=stamp // 1_000_000_000,
                                                   nanosec=stamp % 1_000_000_000),
        frame_id='velodyne')
    data = np.ascontiguousarray(points, dtype='<f4')
    return types['sensor_msgs/msg/PointCloud2'](
        header=header, height=1, width=len(data), fields=fields, is_bigendian=False,
        point_step=16, row_step=16 * len(data),
        data=np.frombuffer(data.tobytes(), dtype=np.uint8), is_dense=True)


def imu_message(typestore, stamp, values):
    # OXTS packet: ..., ax ay az (11-13, m/s^2, body x forward y left z up),
    # ..., wx wy wz (17-19, rad/s, body frame).
    types = typestore.types
    vector = types['geometry_msgs/msg/Vector3']
    zero = np.zeros(9)
    return types['sensor_msgs/msg/Imu'](
        header=types['std_msgs/msg/Header'](
            stamp=types['builtin_interfaces/msg/Time'](sec=stamp // 1_000_000_000,
                                                       nanosec=stamp % 1_000_000_000),
            frame_id='imu'),
        orientation=types['geometry_msgs/msg/Quaternion'](x=0., y=0., z=0., w=1.),
        orientation_covariance=np.r_[-1., zero[1:]],
        angular_velocity=vector(x=values[17], y=values[18], z=values[19]),
        angular_velocity_covariance=zero,
        linear_acceleration=vector(x=values[11], y=values[12], z=values[13]),
        linear_acceleration_covariance=zero)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('date')
    parser.add_argument('drive')
    parser.add_argument('out', type=Path)
    parser.add_argument('--first', type=int, default=0)
    parser.add_argument('--count', type=int, default=None)
    parser.add_argument('--image-step', type=int, default=2)
    parser.add_argument('--jpeg-quality', type=int, default=92)
    parser.add_argument('--voxel', type=float, default=0.1)
    args = parser.parse_args()
    name = f'{args.date}_drive_{args.drive}'
    args.out.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(HttpFile(f'{BASE}/{args.date}_calib.zip')) as calib:
        (args.out / 'calib').mkdir(exist_ok=True)
        for member in calib.namelist():
            if member.endswith('.txt'):
                (args.out / 'calib' / Path(member).name).write_bytes(calib.read(member))

    sync = zipfile.ZipFile(HttpFile(f'{BASE}/{name}/{name}_sync.zip'))
    sync_images = zipfile.ZipFile(HttpFile(f'{BASE}/{name}/{name}_sync.zip'))
    prefix = f'{args.date}/{name}_sync/'
    velo_stamps = stamps(sync.read(prefix + 'velodyne_points/timestamps.txt'))
    frames = range(args.first, min(len(velo_stamps), args.first + (args.count or len(velo_stamps))))
    images = args.out / 'image_02'
    images.mkdir(exist_ok=True)
    (images / 'timestamps.txt').write_bytes(sync.read(prefix + 'image_02/timestamps.txt'))

    extract = zipfile.ZipFile(HttpFile(f'{BASE}/{name}/{name}_extract.zip'))
    xprefix = f'{args.date}/{name}_extract/oxts/'
    oxts_stamps = stamps(extract.read(xprefix + 'timestamps.txt'))
    window = (velo_stamps[frames[0]] - 2_000_000_000, velo_stamps[frames[-1]] + 1_000_000_000)

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    writer = Writer(args.out / 'bag', version=8)
    writer.set_compression(CompressionMode.MESSAGE, CompressionFormat.ZSTD)
    writer.open()
    velo = writer.add_connection('/kitti/velo', 'sensor_msgs/msg/PointCloud2', typestore=typestore)
    imu = writer.add_connection('/kitti/imu', 'sensor_msgs/msg/Imu', typestore=typestore)

    # The archives store each folder's files in no particular order, so members are read in
    # archive order (one sequential pass over the range) and written to the bag as they come;
    # bag readers return messages sorted by time.
    def in_archive_order(archive, names):
        offsets = {i.filename: i.header_offset for i in archive.infolist()}
        return sorted(names, key=lambda item: offsets[item[0]])

    wanted = [(f'{xprefix}data/{i:010d}.txt', i) for i, stamp in enumerate(oxts_stamps)
              if window[0] <= stamp <= window[1]]
    for member, i in in_archive_order(extract, wanted):
        values = np.array(extract.read(member).split(), dtype=float)
        writer.write(imu, oxts_stamps[i], typestore.serialize_cdr(
            imu_message(typestore, oxts_stamps[i], values), 'sensor_msgs/msg/Imu'))

    written = 0
    for member, i in in_archive_order(
            sync, [(f'{prefix}velodyne_points/data/{i:010d}.bin', i) for i in frames]):
        raw = np.frombuffer(sync.read(member), dtype='<f4').reshape(-1, 4)
        cloud = voxel_downsample(raw, args.voxel)
        writer.write(velo, velo_stamps[i], typestore.serialize_cdr(
            cloud_message(typestore, velo_stamps[i], cloud), 'sensor_msgs/msg/PointCloud2'))
        written += 1
        if written % 200 == 0:
            print(f'{written}/{len(frames)} LiDAR frames', flush=True)

    shots = [(f'{prefix}image_02/data/{i:010d}.png', i) for i in frames
             if (i - args.first) % args.image_step == 0]
    for n, (member, i) in enumerate(in_archive_order(sync_images, shots), start=1):
        Image.open(io.BytesIO(sync_images.read(member))).convert('RGB').save(
            images / f'{i:010d}.jpg', quality=args.jpeg_quality)
        if n % 200 == 0:
            print(f'{n}/{len(shots)} image frames', flush=True)
    writer.close()
    print(f'{written} LiDAR frames, {len(wanted)} IMU samples, {len(shots)} images')
    return 0


if __name__ == '__main__':
    sys.exit(main())
