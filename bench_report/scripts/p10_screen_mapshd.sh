#!/usr/bin/env bash
# MapsHD's eight Phase 1 trials for the G24 screen (bench_report/data/phase1_global/screen_g24/plan.txt), without ROS:
# per scenario, download (zenodo record 21318128, md5 as p1_mapshd_get.sh), extract the trials, convert with
# mapshd_to_ros2_rosbags.py, run G22 and G24 (p10_screen_pair.sh), score (front_score.py), delete.
# Writes $W/mapshd.csv: trial,config,ape,md5,peak_rss_kb,wall_s
set -uo pipefail
R=${R:-/home/user/lidar_slam}; S=$R/bench_report/scripts; W=${W:-/home/user/screen/mapshd}
VENV=${VENV:-/tmp/claude-0/-home-user-lidar-slam/4b11ccc8-557f-56c0-a590-ad3546c86573/scratchpad/venv}
PY=$VENV/bin/python
declare -A MD5=([calibration-room]=f965681f362701e48b4631e95e54bd10 [flat-room]=b698db851aba5abfc0e5f63bd424bfa9
                [long-corridor]=aebeeab1f6805dd03b821ea33e2d375a [underground-garage]=09da4a85a01d406ae97a3b651d6d8cd2
                [outdoor-urk]=e747e46e39ce18cdec25e818b9dde813)
declare -A TRIALS=([calibration-room]="trial-1 trial-2" [flat-room]="trial-1" [long-corridor]="trial-1"
                   [outdoor-urk]="trial-1" [underground-garage]="trial-1 trial-2 trial-3")
mkdir -p "$W/dl" "$W/x" "$W/bags" "$W/gt" "$W/runs"
for sc in ${@:-calibration-room flat-room long-corridor outdoor-urk underground-garage}; do
  f=$W/dl/$sc.7z
  [ -f "$f" ] || curl -sS -L --fail -m 7200 -o "$f" "https://zenodo.org/records/21318128/files/${sc}.7z?download=1" \
    || { echo "$sc: download failed"; continue; }
  [ "$(md5sum "$f" | cut -d' ' -f1)" == "${MD5[$sc]}" ] || { echo "$sc: MD5 MISMATCH"; rm -f "$f"; continue; }
  (cd "$W/x" && "$PY" -I -c "
import py7zr, sys
want = ['/' + t + '/' for t in sys.argv[2].split()]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if any(w in n + '/' for w in want)]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$f" "${TRIALS[$sc]}") || { echo "$sc: extract failed"; continue; }
  rm -f "$f"
  for t in ${TRIALS[$sc]}; do
    tr=${sc}_${t}; tdir=$W/x/$sc/$t; bag=$W/bags/$tr
    mkdir -p "$W/gt/$tr"; cp "$tdir/ground-truth/ground_truth.tum" "$W/gt/$tr/"
    [ -d "$bag" ] || (cd /tmp && "$PY" -I "$S/mapshd_to_ros2_rosbags.py" "$tdir/hdmapping-raw" "$bag" > "$W/bags/convert_$tr.log" 2>&1) \
      || { echo "$tr: conversion failed"; rm -rf "$bag"; continue; }
    rm -rf "$tdir"
    bash "$S/p10_screen_pair.sh" "$R/bench_report/params/mapshd_gyro002_jerk1000.yaml" "$bag" "$W/runs/$tr" \
      /livox/pointcloud /livox/imu
    for c in G22 G24; do
      o=$W/runs/$tr/$c
      ape=$(EVO_APE=$VENV/bin/evo_ape PATH=$VENV/bin:$PATH "$PY" "$S/front_score.py" "$tr" "$o/front/odometry_tum.txt" \
            "$W/gt/$tr/ground_truth.tum" 2>/dev/null | cut -d, -f2)
      echo "$tr,$c,$ape,$(md5sum < "$o/front/odometry_tum.txt" | cut -c1-8),$(grep Maximum "$o/time.txt" | awk '{print $NF}'),$(cat "$o/wall.txt")" \
        | tee -a "$W/mapshd.csv"
    done
    rm -rf "$bag"
  done
  rm -rf "$W/x/$sc"
done
echo "mapshd done"
