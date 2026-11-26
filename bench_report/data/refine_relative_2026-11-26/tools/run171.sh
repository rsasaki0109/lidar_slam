#!/bin/bash
# #171 screen (plan.txt): record backend inputs, then replay variants.
set -u
B=/home/user/bench/refine171; REPO=/home/user/lidar_slam_ros2
PY=${PY:-python3}  # an environment with lidarslam built from develop
P=$REPO/lidarslam/param/lidarslam.yaml
for s in con-1 con-2; do
  D=$B/data/$s
  [ -d $D/backend_input ] || $PY -m lidarslam map $D/bag --odometry-params $D/params.yaml --backend-params $P \
     --out $D/map_run --backend-input $D/backend_input > $D/map.log 2>&1 || echo "$s map failed"
done
variant() { case $1 in
  DEF) ;; OFF) echo "-p refine_window_max_adjacent_change_m:=0.0 -p refine_window_max_adjacent_change_deg:=0.0" ;;
  RA1) echo "-p refine_window_max_adjacent_change_m:=0.0 -p refine_window_max_adjacent_change_deg:=0.0 -p refine_relative_translation_sigma:=0.01 -p refine_relative_rotation_sigma_rad:=0.00175" ;;
  RA2) echo "-p refine_window_max_adjacent_change_m:=0.0 -p refine_window_max_adjacent_change_deg:=0.0 -p refine_relative_translation_sigma:=0.02 -p refine_relative_rotation_sigma_rad:=0.0035" ;;
  RA5) echo "-p refine_window_max_adjacent_change_m:=0.0 -p refine_window_max_adjacent_change_deg:=0.0 -p refine_relative_translation_sigma:=0.05 -p refine_relative_rotation_sigma_rad:=0.0087" ;;
  RA10) echo "-p refine_window_max_adjacent_change_m:=0.0 -p refine_window_max_adjacent_change_deg:=0.0 -p refine_relative_translation_sigma:=0.10 -p refine_relative_rotation_sigma_rad:=0.0175" ;;
  RA2G) echo "-p refine_relative_translation_sigma:=0.02 -p refine_relative_rotation_sigma_rad:=0.0035" ;;
esac; }
for s in con-1 con-2; do for v in DEF OFF RA1 RA2 RA5 RA10 RA2G; do for pert in 0 1um; do
  [ $pert = 1um ] && [ $v = OFF ] && continue
  O=$B/runs/$s/$v/$pert; [ -s $O/done ] && continue; mkdir -p $O
  extra=$(variant $v); [ $pert = 1um ] && extra="$extra -p submap_cloud_max_error_m:=0.000001"
  # shellcheck disable=SC2086
  $PY -m lidarslam backend $B/data/$s/backend_input --params $P --out $O/out -p offline_map_bundle:=false $extra > $O/run.log 2>&1
  echo "exit=$?" > $O/done; echo "$s $v $pert $(cat $O/done)"
done; done; done
echo RUN171 DONE
