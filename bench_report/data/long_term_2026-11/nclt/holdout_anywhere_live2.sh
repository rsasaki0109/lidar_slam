# Second live check of --relocalize-anywhere (backend loop search on a worker) (preregistered in
# docs/roadmap/long-term-mapping-2026-11.md before this script was run). Bags: convert_window.sh
# with anywhere_live2_windows.json (live2_base, DAY_live2in, DAY_live2out).
set -u
cd /home/user/bench/nclt
R=/home/user/lidar_slam
D=$R/bench_report/data/long_term_2026-11/nclt
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
BASE=2012-01-22
DAYS="2012-03-31 2012-08-04 2013-02-23"
python3 -I $D/gt_npy.py /home/user/bench/nclt full $BASE $DAYS
[ -f stores/live2_store/store.json ] || python3 -I -m lidarslam live bags/live2_base $O --out runs/live2_base --store stores/live2_store > logs/run_live2_base.txt 2>&1
rt() {
  rm -rf stores/any_$2 runs/any_$2; cp -a stores/live2_store stores/any_$2
  python3 -I -m lidarslam live bags/$1 $O --out runs/any_$2 --store stores/any_$2 --relocalize-anywhere --realtime > logs/run_any_$2.txt 2>&1
}
for day in $DAYS; do
  rt ${day}_live2in live2_${day}_in & rt ${day}_live2out live2_${day}_out & wait
  for part in in out; do
    name=live2_${day}_$part
    python3 -I $R/scripts/evaluate_relocalize_anywhere.py --base-map runs/live2_base/map.pcd \
      --base-odometry runs/live2_base/odometry_tum.txt --base-gt gt/$BASE.npy \
      --session-map runs/any_$name/map.pcd --session-odometry runs/any_$name/odometry_tum.txt \
      --session-gt gt/$day.npy --anchor stores/any_$name/store.json --label $name > eval_$name.json
    python3 -I - "$name" <<'PY' >> holdout_anywhere_live2.jsonl
import json, re, sys
name = sys.argv[1]
row = json.loads(open(f'eval_{name}.json').readline())
log = open(f'logs/run_any_{name}.txt').read()
timing = json.load(open(f'runs/any_{name}/live_timing.json'))
start = re.search(r'"store_start_stamp_sec": ([0-9.]+)', log)
anchor = re.search(r'"store_anchor_stamp_sec": ([0-9.]+)', log)
row['anchor_after_s'] = float(anchor.group(1)) - float(start.group(1)) if anchor and start else None
row['data_s'] = timing['data_seconds']
row['lag_seconds'] = timing['lag_seconds']
if row['anchored']:
    row['correct'] = (row['converged_apart_m'] <= 0.25 and row['converged_apart_deg'] <= 0.5
                      and row['gt_offset_rms_m'] <= 4.0)
print(json.dumps(row))
PY
  done
done
cat holdout_anywhere_live2.jsonl
