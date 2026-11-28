#!/usr/bin/env bash
# Upstream RKO-LIO (PyPI rko_lio, its defaults: bench_report/data/blind_v7_2026-11-08/rko_upstream_default.yaml) on one
# FoMo drive, with the rig's extrinsics so that the output is the base_link pose like G22's.
# usage: p10_fomo_upstream.sh NAME     (bag $W/bags/NAME, rig $W/meta/rig_NAME.yaml; writes $W/runs/NAME/upstream/)
set -euo pipefail
NAME=$1
W=${W:-/home/user/fomo_work}; R=${R:-/home/user/lidar_slam}
VENV=${VENV:-/tmp/claude-0/-home-user-lidar-slam/4b11ccc8-557f-56c0-a590-ad3546c86573/scratchpad/venv}
OUT=$W/runs/$NAME/upstream
rm -rf "$OUT"; mkdir -p "$OUT"
"$VENV/bin/python" -I - "$W/meta/rig_$NAME.yaml" "$R/bench_report/data/blind_v7_2026-11-08/rko_upstream_default.yaml" \
  "$OUT/config.yaml" <<'EOF'
import sys, yaml
rig = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
cfg = yaml.safe_load(open(sys.argv[2]))
for k in ('extrinsic_imu2base_quat_xyzw_xyz', 'extrinsic_lidar2base_quat_xyzw_xyz'):
    cfg[k] = [float(v) for v in rig[k]]
yaml.safe_dump(cfg, open(sys.argv[3], 'w'), sort_keys=False)
EOF
cd "$OUT"
start=$(date +%s.%N)
/usr/bin/time -v -o time.txt "$VENV/bin/rko_lio" -d rosbag --imu /imu --lidar /points -c config.yaml -o results -n run \
  "$W/bags/$NAME" > stdout.log 2>&1 || echo "exit $?" >> stdout.log
echo "wall $(echo "$(date +%s.%N) - $start" | bc)" > run_info.txt
f=$(find results -name '*.txt' | head -1)
[ -n "$f" ] && grep -v '^#' "$f" > traj.tum
