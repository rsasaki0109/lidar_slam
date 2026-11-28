# lidarslam (ROS-free core)

The mapping core of [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) without a
ROS 2 install. ROS 2 stays one way to run it; this directory is the other.

Status: preview (`docs/roadmap/ros-free-core-2026-11.md`). The Python package reads sensor logs
(rosbag2 MCAP/SQLite3, ROS 1 bags), runs the RKO-LIO odometry and the graph_based_slam offline
backend, and installs the `lidarslam-map` product command. Maps of an RKO-LIO profile are
byte-identical to the ROS 2 offline path. Install from source, or from a manylinux wheel that
bundles every C++ library: `.github/workflows/wheels.yml` builds them for Python 3.10-3.13 on
x86_64 and aarch64 (glibc 2.28+) as workflow artifacts (not published). The x86_64 wheel gives
the same map bytes as the source install; aarch64 maps differ in the last bits (GCC fuses
multiply-adds there) and are checked by the test suite.

## Python package

```bash
pip install ./lidarslam_core          # from a clone of the repository (apt packages below)
lidarslam-map doctor                  # map engine: core, no ROS 2 needed
lidarslam-map start /path/to/rosbag2  # verified Autoware map bundle
lidarslam-map start /path/to/day2 --join-map ./output/day1   # join a repeat visit to a stored map
python -m lidarslam inspect /path/to/rosbag2
```

The multi-session join (`lidarslam-map merge --place-recognition`, `start --join-map`) runs on
the package too: `python -m lidarslam backend` replays each run's recorded backend input and
`python -m lidarslam join` takes the parameters of the ROS 2 `graph_slam_multisession_join`
(`--params FILE -p name:=value`) and writes the same files.

A log with a camera becomes a scene for 3D Gaussian Splatting, posed by LiDAR SLAM instead of
structure from motion (`pip install "./lidarslam_core[gsplat]"` for Pillow):

```bash
python -m lidarslam map LOG --odometry-params P --backend-params P --out RUN
python -m lidarslam colmap LOG --run RUN --image-topic /camera/image_raw/compressed --out SCENE \
    [--base-frame base_link] [--every 2] [--scale 0.5]
```

`SCENE` is COLMAP's text model (`images/`, `sparse/0/cameras.txt images.txt points3D.txt`), as
3DGS trainers read it. Each image is posed at its header stamp: the run's optimised keyframes,
odometry carried in between, and the camera's optical frame on `--base-frame` from the log's
`/tf_static` (or `--extrinsic x y z qx qy qz qw`). Intrinsics come from the CameraInfo next to
the image topic; a distorted camera is written as `OPENCV`, `FULL_OPENCV` or `OPENCV_FISHEYE`.
The initial points are the run's `map.pcd` within `--max-depth` of a camera, coloured from the
images.

`--depth-lidar-topic /points` also writes `SCENE/depth_priors/<image name>.bin`: the LiDAR scan
nearest each image stamp projected into it (nearest point per pixel; `u32` width and height,
then `f32` camera-z depths in metres, 0 where no point falls), for depth-supervised training
such as visloc-rs `gsplat_train --depth-prior SCENE/depth_priors`. The scan is placed by the
run's poses at its own stamp, so a LiDAR and a camera that are not triggered together still
line up. The LiDAR mount comes from `/tf_static` (`--lidar-frame` when the scans' `frame_id`
is not in it) or `--lidar-extrinsic x y z qx qy qz qw`.

### Tutorial: a GrandTour mission to depth-supervised 3DGS

From a rosbag to a trained splat on a legged robot outside KITTI: GrandTour (ETH Zürich,
`leggedrobotics/grand_tour_dataset` on Hugging Face, MIT) mission `2024-12-09-09-34-43`, the
Hesai XT32, the STIM320 IMU and the front HDR fisheye camera. Fetch `data/stim320_imu.tar`,
`data/hesai_points.tar` and `data/hdr_front.tar` (zarr) into `MISSION/` and `images/hdr_front.tar`
(jpeg) into `IMAGES/`, then:

```bash
# 1. LiDAR + IMU -> rosbag2 (with ROS 2 sourced; prep_con.sh below does it without ROS 2)
python bench_report/scripts/p1_grandtour_to_ros2.py --imu stim320_imu --lidar hesai_points MISSION BAG
# 2. map it; params.yaml = configs/sota_v7/rko_lio_grandtour_hs_G15g_2026-10-28.yaml with the rig's
#    topics and extrinsics, as bench_report/data/refine_relative_2026-11-26/tools/prep_con.sh writes it
python -m lidarslam map BAG --odometry-params params.yaml \
    --backend-params lidarslam/param/lidarslam.yaml --out RUN
# 3. a 60 s window with the camera and /tf_static (base_link = stim320_imu -> hdr_front, hesai_lidar)
python -I tools/gaussian_splatting/grandtour_camera_bag.py MISSION IMAGES BAG CAM_BAG T0 T0+60
# 4. the scene, with LiDAR depth priors
python -m lidarslam colmap CAM_BAG --run RUN --image-topic /hdr_front/image_raw/compressed \
    --out SCENE --every 2 --scale 0.4 --depth-lidar-topic /hesai_points
# 5. train (visloc-rs), without and with the prior
gsplat_train --data SCENE --steps 7000 --out base.ply
gsplat_train --data SCENE --steps 7000 --depth-prior SCENE/depth_priors --depth-weight 10 --depth-every 4 --out w10.ply
```

Step 4 writes 300 `OPENCV_FISHEYE` 768×512 views with LiDAR depth on 5.3 % of the pixels (32
beams, one scan per image). GrandTour stores each sensor's `transform` as the inverse of
box_base → sensor; step 3 inverts it, so the scans fall on the image (the LiDAR is not triggered
with the camera; the run's poses place each scan at its own stamp).

On this window (262 training / 38 held-out views, every 8th):

| | held-out PSNR | rendered depth vs LiDAR, median relative error |
|---|---:|---:|
| `base.ply` (photometric only) | 38.67 dB | 93 % |
| `w10.ply` (`--depth-prior`, weight 10) | 38.11 dB | 92 % |

This window is a dim hall and the images are dark (mean pixel value 7-18 of 255),
so the photometric loss is small everywhere and both models explain the images with a dark haze
0.3-1 m in front of each camera: the high PSNR is not a sign of a good geometry, and at weight 10
on 5 % of the pixels (the loss is averaged over all pixels) the prior does not hold the geometry
either (on KITTI, 10 % of the pixels, the same weight takes the depth error from 6 % to 2.5 %).
The depth is compared as camera z, each view rendered as a pinhole canvas around the fisheye.
The run is a walk-through of the pipeline, not a quality
claim. A brighter window or a higher `--depth-weight` is the next thing to try.

`lidarslam-map` is the product command of the ROS 2 install: the package carries its scripts,
schemas and parameter files under `lidarslam/share/lidarslam`, in the tree the ROS 2 package
installs, from the same file lists (`lidarslam/product-runtime-files.txt`,
`lidarslam/product-schema-files.txt`).

```python
from lidarslam.io import SensorLog

with SensorLog('/path/to/rosbag2') as log:
    for record in log.records(['/points', '/imu']):
        print(record.topic, record.log_time_ns, type(record.message).__name__)
```

## C++ core (CMake, no ROS)

```bash
sudo apt install build-essential cmake libeigen3-dev libpcl-dev libg2o-dev libzstd-dev \
  libtbb-dev robin-map-dev libgtest-dev
cmake -S lidarslam_core -B build-core -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j && ctest --test-dir build-core
```
