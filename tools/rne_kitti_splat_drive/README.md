# KITTI splat drive (RobotNativeEngine)

KITTI raw drive 0018 is mapped by lidar_slam from LiDAR and IMU only. The optimized trajectory,
through the KITTI camera calibration, poses the camera images: `scripts/export_kitti_colmap.py`
writes a COLMAP scene per segment, with `--scale`, a frame range, and initial points sampled
near that segment's cameras. Seven overlapping 3D Gaussian Splatting segments trained on those
scenes form a metric, gravity-aligned copy of the 2.2 km street.

The simulator side lives in
[RobotNativeEngine](https://github.com/rsasaki0109/RobotNativeEngine)
(`examples/160_kitti_splat_drive`, documented in `docs/KITTI_SPLAT_DRIVE.md`). It covers:
- example 160: the drive;
- example 161: a closed loop on visual localization of the rendered camera, finishing all
  2.2 km with 0.25 m mean error;
- example 162: traffic with movable splat actors.

The renderer fixes this work needed (splat colour encoded twice, and splats drawn sqrt(2)
too wide) are merged in RNE.

The README GIF and its numbers are described in `docs/readme-media.md`.
KITTI is CC BY-NC-SA 3.0, so this is for non-commercial use only.
