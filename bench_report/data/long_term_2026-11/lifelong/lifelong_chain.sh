# Lifelong chain on NCLT: one store started from 2012-01-15, then every in-area window of the
# evaluation square in date order, each with --relocalize-anywhere --detect-changes (in line).
# Bags: the windows converted for the earlier NCLT checks (bench_report/data/long_term_2026-11/nclt).
set -u
cd /home/user/bench/nclt
R=/home/user/lidar_slam
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
SESSIONS="2012-01-22:live2_base 2012-02-02:2012-02-02_in 2012-02-05:live_base 2012-02-18:2012-02-18
2012-03-17:2012-03-17_in 2012-03-25:2012-03-25_livein 2012-03-31:2012-03-31_live2in 2012-05-11:2012-05-11
2012-05-26:2012-05-26_trim81 2012-06-15:2012-06-15 2012-08-04:2012-08-04_live2in 2012-08-20:2012-08-20_livein
2012-09-28:2012-09-28_in 2012-10-28:2012-10-28_in 2012-11-04:2012-11-04 2012-11-16:2012-11-16
2012-11-17:2012-11-17 2012-12-01:2012-12-01_livein 2013-02-23:2013-02-23_live2in"
mkdir -p lifelong
[ -f stores/lifelong/store.json ] || cp -a stores/d_store stores/lifelong
for entry in $SESSIONS; do
  day=${entry%%:*}; bag=${entry#*:}
  [ -d lifelong/run_$day ] && continue
  s=$(date +%s)
  python3 -I -m lidarslam live bags/$bag $O --out lifelong/run_$day --store stores/lifelong --relocalize-anywhere --detect-changes > lifelong/log_$day.txt 2>&1
  code=$?
  python3 -I - "$day" "$code" "$(( $(date +%s) - s ))" <<'PY' >> lifelong/chain.jsonl
import json, re, sys
day, code, wall = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
log = open(f'lifelong/log_{day}.txt').read()
start = re.search(r'"store_start_stamp_sec": ([0-9.]+)', log)
anchor = re.search(r'"store_anchor_stamp_sec": ([0-9.]+)', log)
print(json.dumps({'day': day, 'exit': code, 'wall_s': wall,
                  'anchor_after_s': float(anchor.group(1)) - float(start.group(1)) if anchor and start else None}))
PY
  tail -1 lifelong/chain.jsonl
done
