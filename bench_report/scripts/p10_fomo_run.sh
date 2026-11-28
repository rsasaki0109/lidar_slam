#!/usr/bin/env bash
# One FoMo drive through G22 (frontend) and the default walking backend, then the score
# (bench_report/data/fomo_dev_2026-11-28/plan.txt).
# usage: p10_fomo_run.sh NAME DRIVE [VARIANT_YAML]
#   NAME   e.g. red_2025-08-20 (bag at $W/bags/NAME, rig keys at $W/meta/rig_NAME.yaml)
#   DRIVE  e.g. 2025-08-20/red_2025-08-20-10-42 (for gt.txt, fetched into $W/gt/NAME/ after the run)
#   VARIANT_YAML  optional extra keys over G22 (a flat or ros__parameters file); its stem names the run
# Env: W (work dir, default /home/user/fomo_work), R (repository), PY (python with lidarslam), SKIP_BACKEND=1
set -euo pipefail
NAME=$1; DRIVE=$2; VAR=${3:-}
W=${W:-/home/user/fomo_work}; R=${R:-/home/user/lidar_slam}
PY=${PY:-/tmp/claude-0/-home-user-lidar-slam/4b11ccc8-557f-56c0-a590-ad3546c86573/scratchpad/venv/bin/python}
TAG=G22${VAR:+_$(basename "${VAR%.*}")}
OUT=$W/runs/$NAME/$TAG
mkdir -p "$OUT" "$W/gt/$NAME"
# merged parameter file: G22, the rig, the run's own keys, then the variant
"$PY" -I - "$R/configs/sota_v7/rko_lio_global_G22_2026-11-10.yaml" "$W/meta/rig_$NAME.yaml" ${VAR:+"$VAR"} \
  > "$OUT/params.yaml" <<'EOF'
import sys, yaml
merged = {}
for p in sys.argv[1:]:
    d = yaml.safe_load(open(p))
    if isinstance(d, dict) and len(d) == 1 and isinstance(next(iter(d.values())), dict) \
            and 'ros__parameters' in next(iter(d.values())):
        d = next(iter(d.values()))['ros__parameters']
    merged.update(d)
merged.update({'odom_frame': 'odom', 'lidar_frame': '', 'imu_frame': '', 'publish_deskewed_scan': False})
yaml.safe_dump({'/**': {'ros__parameters': merged}}, sys.stdout, sort_keys=True)
EOF
cd "$OUT"
start=$(date +%s.%N)
/usr/bin/time -v "$PY" -m lidarslam odometry "$W/bags/$NAME" --params params.yaml --out front \
  --lidar-topic /points --imu-topic /imu > front.log 2> time_front.txt || echo "odometry exit $?" >> front.log
echo "wall_front $(echo "$(date +%s.%N) - $start" | bc)" > run_info.txt
if [ -z "${SKIP_BACKEND:-}" ]; then
  start=$(date +%s.%N)
  /usr/bin/time -v "$PY" -m lidarslam map "$W/bags/$NAME" --odometry-params params.yaml \
    --backend-params "$R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml" --out map \
    --lidar-topic /points --imu-topic /imu > map.log 2> time_map.txt || echo "map exit $?" >> map.log
  echo "wall_map $(echo "$(date +%s.%N) - $start" | bc)" >> run_info.txt
fi
B=https://fomo-dataset.s3.us-west-2.amazonaws.com/data/$DRIVE
for f in gt.txt gt_covariance.csv; do
  [ -s "$W/gt/$NAME/$f" ] || curl -sf "$B/$f" -o "$W/gt/$NAME/$f"
done
ls front map 2>/dev/null > outputs.txt || true
