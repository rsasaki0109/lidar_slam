#!/usr/bin/env bash
# The lifelong chain built again with the live defaults (tilt correction, height check) into
# stores/lifelong_tilt (preregistered, roadmap). Same base, updates, bags and options as
# lifelong_chain.sh. Run from the bench directory; scored by consistency_score.py.
set -u
R=$(cd "$(dirname "$0")/../../../.." && pwd)
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
SESSIONS="2012-01-22:live2_base 2012-02-02:2012-02-02_in 2012-02-05:live_base 2012-02-18:2012-02-18
2012-03-17:2012-03-17_in 2012-03-25:2012-03-25_livein 2012-03-31:2012-03-31_live2in 2012-05-11:2012-05-11
2012-05-26:2012-05-26_trim81 2012-06-15:2012-06-15 2012-08-04:2012-08-04_live2in 2012-08-20:2012-08-20_livein
2012-09-28:2012-09-28_in 2012-10-28:2012-10-28_in 2012-11-04:2012-11-04 2012-11-16:2012-11-16
2012-11-17:2012-11-17 2012-12-01:2012-12-01_livein 2013-02-23:2013-02-23_live2in"
mkdir -p lifelong_tilt
[ -f stores/lifelong_tilt/store.json ] || cp -a stores/d_store stores/lifelong_tilt
for entry in $SESSIONS; do
  day=${entry%%:*}; bag=${entry#*:}
  [ -d lifelong_tilt/run_$day ] && continue
  s=$(date +%s)
  python3 -I -m lidarslam live bags/$bag $O --out lifelong_tilt/run_$day --store stores/lifelong_tilt \
    --relocalize-anywhere --detect-changes > lifelong_tilt/log_$day.txt 2>&1
  code=$?
  rm -rf lifelong_tilt/run_$day/pointcloud_map lifelong_tilt/run_$day/map.pcd
  python3 -I - "$day" "$code" "$(( $(date +%s) - s ))" <<'PY' >> lifelong_tilt/chain.jsonl
import json, sys
day, code, wall = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
log = open(f'lifelong_tilt/log_{day}.txt').read()
summary = json.JSONDecoder().raw_decode(log[log.rindex('\n{\n') + 1:])[0] if '\n{\n' in log else {}
print(json.dumps({'day': day, 'exit': code, 'wall_s': wall,
                  'committed': 'store_session' in summary,
                  'store_refused': summary.get('store_refused'),
                  'tilt_correction': summary.get('store_tilt_correction'),
                  'height_check': summary.get('store_height_check')}))
PY
  tail -1 lifelong_tilt/chain.jsonl
done
