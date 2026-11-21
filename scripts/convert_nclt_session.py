#!/usr/bin/env python3
"""Convert one NCLT session to a compact ROS 2 bag, streamed.

The University of Michigan North Campus Long-Term dataset (robots.engin.umich.edu/nclt, ODbL 1.0 /
DbCL 1.0; Carlevaris-Bianco, Ushani and Eustice, IJRR 2016) has 27 sessions of one campus over
15 months, recorded from a Segway with an upside-down Velodyne HDL-32E and a Microstrain
3DM-GX3-45 IMU. This reads ``YYYY-MM-DD_vel.tar.gz`` front to back from a file or over HTTP,
never storing it, and writes a ROS 2 bag (zstd-compressed messages) with:

  /nclt/points  sensor_msgs/PointCloud2, one per 0.1 s of Velodyne packets, in the Velodyne
                frame: x, y, z, intensity (float32) and t (uint32, ns after the header stamp,
                which is the scan's first packet; NCLT gives one time per packet); with
                --point-step N only every N-th return is kept
  /nclt/imu     sensor_msgs/Imu from ms25.csv (acceleration and angular rate, IMU frame,
                z down); orientation unset

With --start / --duration only that window (seconds after the first packet) is written, or with
--from-utime / --to-utime the window between two NCLT UTIMEs.
`bench_report/data/long_term_2026-11/rko_lio_nclt.yaml` holds the matching extrinsics.

  convert_nclt_session.py VEL_TAR MS25_CSV OUT_DIR [--point-step 2] [--start S] [--duration D]

VEL_TAR is a path or an https URL of YYYY-MM-DD_vel.tar.gz; MS25_CSV is the session's ms25.csv
from YYYY-MM-DD_sen.tar.gz.
"""

import argparse
import struct
import sys
import tarfile
import time
import urllib.request
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import CompressionFormat, CompressionMode, Writer
from rosbags.typesys import Stores, get_typestore

MAGIC = b'\x9c\xad' * 4
HIT = np.dtype([('x', '<u2'), ('y', '<u2'), ('z', '<u2'), ('i', 'u1'), ('l', 'u1')])
POINTS, IMU = '/nclt/points', '/nclt/imu'
FLOAT32, UINT32 = 7, 6
SCAN_US = 100_000


class Resuming:
    """File-like reader over an HTTP URL that resumes with Range requests after errors or a
    connection closed before the end of the file."""

    def __init__(self, url: str, retries: int = 50):
        self.url, self.retries, self.offset, self.size = url, retries, 0, None
        self._open()

    def _open(self):
        request = urllib.request.Request(self.url, headers={'Range': f'bytes={self.offset}-'})
        self.handle = urllib.request.urlopen(request, timeout=120)
        if self.size is None:
            total = self.handle.headers.get('Content-Range', '').rpartition('/')[2]
            self.size = int(total) if total.isdigit() else \
                int(self.handle.headers.get('Content-Length', 0)) or None

    def read(self, size=-1):
        chunks, wanted = [], size
        for attempt in range(self.retries):
            try:
                data = self.handle.read(wanted)
            except OSError as error:  # connection reset, timeout
                print(f'stream error at {self.offset}: {error}; resuming', file=sys.stderr)
                data = None
            if data:
                self.offset += len(data)
                chunks.append(data)
                if wanted < 0 or len(data) == wanted:
                    return b''.join(chunks)
                wanted -= len(data)
                continue
            if self.size is not None and self.offset >= self.size:
                return b''.join(chunks)
            if data is not None:
                print(f'stream closed at {self.offset} of {self.size}; resuming', file=sys.stderr)
            time.sleep(min(60, 2 ** min(attempt, 6)))
            self._open()
        raise RuntimeError('too many stream errors')


def packets(stream):
    """(utime_us, hits) of each packet in velodyne_hits.bin; xyz in metres, Velodyne frame."""
    while True:
        header = stream.read(24)
        if len(header) < 24:
            return
        if header[:8] != MAGIC:
            raise ValueError('velodyne_hits.bin: bad packet magic')
        count, utime = struct.unpack('<IQ', header[8:20])
        data = stream.read(8 * count)
        if len(data) < 8 * count:
            return
        yield utime, np.frombuffer(data, dtype=HIT)


def cloud_message(typestore, stamp_us, utimes, hits, step):
    hits, utimes = hits[::step], utimes[::step]
    out = np.zeros(len(hits), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                     ('intensity', '<f4'), ('t', '<u4')])
    for axis in 'xyz':
        out[axis] = hits[axis] * .005 - 100.
    out['intensity'] = hits['i']
    out['t'] = (utimes - stamp_us) * 1000
    keep = (out['x'] != 0) | (out['y'] != 0) | (out['z'] != 0)
    out = out[keep]
    types = typestore.types
    fields = [types['sensor_msgs/msg/PointField'](name=n, offset=o, datatype=d, count=1)
              for n, o, d in (('x', 0, FLOAT32), ('y', 4, FLOAT32), ('z', 8, FLOAT32),
                              ('intensity', 12, FLOAT32), ('t', 16, UINT32))]
    header = types['std_msgs/msg/Header'](
        stamp=types['builtin_interfaces/msg/Time'](sec=stamp_us // 1_000_000,
                                                   nanosec=stamp_us % 1_000_000 * 1000),
        frame_id='velodyne')
    return types['sensor_msgs/msg/PointCloud2'](
        header=header, height=1, width=len(out), fields=fields, is_bigendian=False,
        point_step=20, row_step=20 * len(out), data=np.frombuffer(out.tobytes(), dtype=np.uint8),
        is_dense=True)


def imu_message(typestore, row):
    types = typestore.types
    utime = int(row[0])
    vector = types['geometry_msgs/msg/Vector3']
    zero = np.zeros(9)
    return types['sensor_msgs/msg/Imu'](
        header=types['std_msgs/msg/Header'](
            stamp=types['builtin_interfaces/msg/Time'](sec=utime // 1_000_000,
                                                       nanosec=utime % 1_000_000 * 1000),
            frame_id='imu'),
        orientation=types['geometry_msgs/msg/Quaternion'](x=0., y=0., z=0., w=1.),
        orientation_covariance=np.r_[-1., zero[1:]],
        angular_velocity=vector(x=row[7], y=row[8], z=row[9]), angular_velocity_covariance=zero,
        linear_acceleration=vector(x=row[4], y=row[5], z=row[6]),
        linear_acceleration_covariance=zero)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('vel_tar')
    parser.add_argument('ms25', type=Path)
    parser.add_argument('out_dir', type=Path)
    parser.add_argument('--point-step', type=int, default=2)
    parser.add_argument('--start', type=float, default=0.)
    parser.add_argument('--duration', type=float, default=float('inf'))
    parser.add_argument('--from-utime', type=int,
                        help='window start as an NCLT UTIME (us); overrides --start')
    parser.add_argument('--to-utime', type=int,
                        help='window end as an NCLT UTIME (us); overrides --duration')
    args = parser.parse_args()

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    imu = np.loadtxt(args.ms25, delimiter=',', ndmin=2)
    if imu.shape[1] != 10 or not np.isfinite(imu).all():
        raise ValueError(f'{args.ms25}: expected 10 finite columns')
    imu = imu[np.r_[True, np.diff(imu[:, 0]) > 0]]
    source = Resuming(args.vel_tar) if args.vel_tar.startswith('https://') else \
        open(args.vel_tar, 'rb')
    writer = Writer(args.out_dir, version=8)
    writer.set_compression(CompressionMode.MESSAGE, CompressionFormat.ZSTD)
    writer.open()
    points = writer.add_connection(POINTS, 'sensor_msgs/msg/PointCloud2', typestore=typestore)
    imus = writer.add_connection(IMU, 'sensor_msgs/msg/Imu', typestore=typestore)
    first = begin = end = None
    imu_index = scans = 0
    pending_hits, pending_times, scan_start = [], [], None

    def flush():
        nonlocal scans, imu_index
        if not pending_hits:
            return
        times = np.concatenate(pending_times)
        stamp = int(times[0])  # the first packet, so point times start at 0
        # IMU samples up to the end of this scan go first, in time order.
        limit = scan_start + SCAN_US
        while imu_index < len(imu) and imu[imu_index, 0] <= limit:
            if imu[imu_index, 0] >= begin:
                row = imu[imu_index]
                writer.write(imus, int(row[0]) * 1000,
                             typestore.serialize_cdr(imu_message(typestore, row),
                                                     'sensor_msgs/msg/Imu'))
            imu_index += 1
        message = cloud_message(typestore, stamp, times, np.concatenate(pending_hits),
                                args.point_step)
        writer.write(points, limit * 1000,
                     typestore.serialize_cdr(message, 'sensor_msgs/msg/PointCloud2'))
        scans += 1

    with tarfile.open(fileobj=source, mode='r|gz') as archive:
        for member in archive:
            if not member.name.endswith('velodyne_hits.bin'):
                continue
            for utime, hits in packets(archive.extractfile(member)):
                if first is None:
                    first = utime
                    begin = args.from_utime if args.from_utime is not None else \
                        first + int(args.start * 1e6)
                    end = args.to_utime if args.to_utime is not None else \
                        begin + args.duration * 1e6
                if utime < begin:
                    continue
                if utime >= end:
                    break
                if scan_start is None or utime >= scan_start + SCAN_US:
                    flush()
                    pending_hits, pending_times = [], []
                    scan_start = begin + (utime - begin) // SCAN_US * SCAN_US
                pending_hits.append(hits)
                pending_times.append(np.full(len(hits), utime, dtype=np.int64))
            break
    flush()
    writer.close()
    print(f'{scans} scans, first packet {first}, window start {begin}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
