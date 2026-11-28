#!/usr/bin/env bash
# G22 and a candidate side by side on one bag through the pip package (bench_report/data/phase1_global/screen_g24/plan.txt).
# usage: p10_screen_pair.sh RIG_YAML BAG OUT_DIR LIDAR_TOPIC IMU_TOPIC
#   RIG_YAML: the family's rig/parameter file; the G file is laid on top of it (as p7_g16_phase1.sh's rig()).
# Env: CAND (candidate file; default the G24 candidate), BASE (default G22), PY (python with lidarslam).
# Writes OUT_DIR/{G22,CAND}/front/odometry_tum.txt, params.yaml, time.txt (/usr/bin/time -v), wall.txt.
set -uo pipefail
RIG=$1; BAG=$2; OUT=$3; LT=$4; IT=$5
R=${R:-/home/user/lidar_slam}
PY=${PY:-/tmp/claude-0/-home-user-lidar-slam/4b11ccc8-557f-56c0-a590-ad3546c86573/scratchpad/venv/bin/python}
BASE=${BASE:-$R/configs/sota_v7/rko_lio_global_G22_2026-11-10.yaml}
CAND=${CAND:-$R/bench_report/data/phase1_global/rko_lio_candidate_G24_2026-11-28.yaml}
for pair in "G22:$BASE" "G24:$CAND"; do
  name=${pair%%:*}; g=${pair#*:}; o=$OUT/$name
  [ -s "$o/front/odometry_tum.txt" ] && continue
  rm -rf "$o"; mkdir -p "$o"
  "$PY" -I - "$RIG" "$g" > "$o/params.yaml" <<'EOF'
import sys, yaml
def flat(p):
    d = yaml.safe_load(open(p))
    if isinstance(d, dict) and len(d) == 1 and isinstance(next(iter(d.values())), dict) \
            and 'ros__parameters' in next(iter(d.values())):
        d = next(iter(d.values()))['ros__parameters']
    return d
merged = {**flat(sys.argv[1]), **flat(sys.argv[2])}
yaml.safe_dump({'/**': {'ros__parameters': merged}}, sys.stdout, sort_keys=True)
EOF
  start=$(date +%s.%N)
  /usr/bin/time -v -o "$o/time.txt" "$PY" -m lidarslam odometry "$BAG" --params "$o/params.yaml" --out "$o/front" \
    --lidar-topic "$LT" --imu-topic "$IT" > "$o/stdout.log" 2>&1 || echo "exit $?" >> "$o/stdout.log"
  echo "$(echo "$(date +%s.%N) - $start" | bc)" > "$o/wall.txt"
done
