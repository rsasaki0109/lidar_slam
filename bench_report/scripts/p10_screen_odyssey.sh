#!/usr/bin/env bash
# Odyssey's seven development drives for the G24 screen (bench_report/data/phase1_global/screen_g24/plan.txt), without
# ROS: each archive is streamed once and teed into the bag converter (p8_odyssey_stream_to_ros2.py) and the reference
# extraction (p8_odyssey_fetch_reference.py); G22 and G24 run (p10_screen_pair.sh) on the rig of H9
# (g_params_odyssey.yaml); the ATE is p8_blind_score_h9.py's (estimate at the reference stamps, SE(3), no time offset).
# Writes $W/odyssey.csv: drive,config,ate,md5,peak_rss_kb,wall_s
set -uo pipefail
R=${R:-/home/user/lidar_slam}; S=$R/bench_report/scripts; W=${W:-/home/user/screen/odyssey}
VENV=${VENV:-/tmp/claude-0/-home-user-lidar-slam/4b11ccc8-557f-56c0-a590-ad3546c86573/scratchpad/venv}; PY=$VENV/bin/python
A=$R/bench_report/data/blind_v7_run2_2026-11-10/odyssey_archives.txt
mkdir -p "$W"
for d in ${@:-Tunnel2 UndergroundCarPark2 UndergroundCarPark1 Beltway3 Beltway1 Beltway2 Marketplace3}; do
  url=$(awk -v s="$d.tar.gz" '$1 == s {print $3}' "$A"); bytes=$(awk -v s="$d.tar.gz" '$1 == s {print $2}' "$A")
  o=$W/$d; mkdir -p "$o"
  if [ ! -f "$o/ok" ]; then
    rm -rf "$o/bag" "$o/bag.spool" "$o/ref"
    "$PY" -I "$S/p8_http_stream.py" "$url" 2> "$o/stream.err" \
      | tee >(cd /tmp && "$PY" -I "$S/p8_odyssey_fetch_reference.py" "$o/ref" > "$o/ref.log" 2>&1) \
      | (cd /tmp && "$PY" -I "$S/p8_odyssey_stream_to_ros2.py" - "$o/bag" --summary "$o/summary.json" > "$o/convert.log" 2>&1)
    sleep 5
    got=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(d["bytes_read"] if d["complete"] else 0)' "$o/summary.json" 2>/dev/null)
    [ "$got" == "$bytes" ] && [ -s "$o/ref/lidar_poses_ref.txt" ] && touch "$o/ok" || { echo "$d: fetch failed ($got of $bytes)"; continue; }
  fi
  bash "$S/p10_screen_pair.sh" "$R/bench_report/data/blind_v7_run2_2026-11-10/g_params_odyssey.yaml" "$o/bag" "$o/runs" \
    /ouster/points /m300/imu
  for c in G22 G24; do
    t=$o/runs/$c/front/odometry_tum.txt
    a=$("$PY" -I - "$t" "$o/ref" <<'PYEOF'
import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location('h9', '/home/user/lidar_slam/bench_report/scripts/p8_blind_score_h9.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
traj, ref = sys.argv[1:3]
stamps = np.loadtxt(f'{ref}/lidar_stamps.txt', dtype=np.int64); poses = np.loadtxt(f'{ref}/lidar_poses_ref.txt', ndmin=2)
t0 = int(stamps[0]); ref_t = (stamps - t0) * 1e-9; ref_p = poses[:, [3, 7, 11]]
sec = np.array([(int(x.split('.')[0]) * 10**9 + int((x.split('.')[1] if '.' in x else '0').ljust(9, '0')[:9]) - t0) * 1e-9
                for x in (l.split(None, 1)[0] for l in open(traj) if l.strip())])
est = np.loadtxt(traj, ndmin=2)
print(f'{m.ate(sec, est[:, 1:4], ref_t, ref_p):.5f}')
PYEOF
)
    echo "$d,$c,$a,$(md5sum < "$t" | cut -c1-8),$(grep Maximum "$o/runs/$c/time.txt" | awk '{print $NF}'),$(cat "$o/runs/$c/wall.txt")" | tee -a "$W/odyssey.csv"
  done
  rm -rf "$o/bag"
done
echo "odyssey done"
