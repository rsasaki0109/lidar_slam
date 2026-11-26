#!/bin/bash
# Convert CON-1/CON-2 to rosbag2 and write RKO-LIO params (as p3_ms_outdoor_con_input.sh, without ROS).
set -euo pipefail
REPO=/home/user/lidar_slam_ros2; S=$REPO/bench_report/scripts
PY=${PY:-python3}  # an environment with lidarslam built from develop
PRISM="0.3088378015503127 -0.038312997211241984 -0.2349441159711169 -0.005724388155828644 -0.0013879431347362404 -0.004742721620119043 0.9999714053840392"
for short in con-1 con-2; do
  D=/home/user/bench/refine171/data/$short; BAG=$D/bag
  # shellcheck disable=SC2086
  "$PY" -I "$S/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --extrinsics "$D/z" $PRISM > "$D/extrinsics.txt"
  if [[ ! -d $BAG ]]; then
    rm -rf "$BAG.part"
    "$PY" -I /home/user/bench/refine171/gt_to_bag.py "$D/z" "$BAG.part" stim320_imu hesai_points > "$D/convert.log"
    mv "$BAG.part" "$BAG"
  fi
  "$PY" -I - "$REPO/configs/sota_v7/rko_lio_grandtour_hs_G15g_2026-10-28.yaml" "$D/extrinsics.txt" "$D/params.yaml" <<'PY'
import sys
import yaml
g = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
ext = {k: v for k, v in (yaml.safe_load(open(sys.argv[2])) or {}).items() if k.startswith('extrinsic_')}
rig = {'base_frame': 'base_link', 'odom_frame': 'odom', 'lidar_frame': '', 'imu_frame': '',
       'lidar_topic': '/hesai_points', 'imu_topic': '/stim320_imu', 'deskewed_scan_voxel_size': 0.1,
       'imu_acceleration_unit': 'auto'}
yaml.safe_dump({**rig, **ext, **g}, open(sys.argv[3], 'w'), sort_keys=False)
PY
  echo "$short prepared"
done
