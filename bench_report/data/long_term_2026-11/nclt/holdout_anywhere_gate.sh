# Check of --relocalize-anywhere with the Scan Context gate at 0.26, in a new part of the campus
# (preregistered in docs/roadmap/long-term-mapping-2026-11.md before this script was run). Bags:
# convert_window.sh with anywhere_gate_windows.json (gate_base, DAY_gatein, DAY_gateout).
set -u
cd /home/user/bench/nclt
R=/home/user/lidar_slam
D=$R/bench_report/data/long_term_2026-11/nclt
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
BASE=2012-01-08
DAYS="2012-02-04 2012-02-12 2012-02-19 2012-04-29 2013-01-10 2013-04-05"
python3 -I $D/gt_npy.py /home/user/bench/nclt full $BASE $DAYS
[ -f stores/gate_store/store.json ] || python3 -I -m lidarslam live bags/gate_base $O --out runs/gate_base --store stores/gate_store > logs/run_gate_base.txt 2>&1
rt() {
  rm -rf stores/any_$2 runs/any_$2; cp -a stores/gate_store stores/any_$2
  python3 -I -m lidarslam live bags/$1 $O --out runs/any_$2 --store stores/any_$2 --relocalize-anywhere --realtime > logs/run_any_$2.txt 2>&1
}
for day in $DAYS; do
  rt ${day}_gatein gate_${day}_in & rt ${day}_gateout gate_${day}_out & wait
  for part in in out; do
    name=gate_${day}_$part
    python3 -I $R/scripts/evaluate_relocalize_anywhere.py --base-map runs/gate_base/map.pcd \
      --base-odometry runs/gate_base/odometry_tum.txt --base-gt gt/$BASE.npy \
      --session-map runs/any_$name/map.pcd --session-odometry runs/any_$name/odometry_tum.txt \
      --session-gt gt/$day.npy --anchor stores/any_$name/store.json --label $name \
      --gt-starts $(cat $D/anywhere_gt_starts.txt) > eval_$name.json
    python3 -I - "$name" "$day" "$BASE" <<'PY' >> holdout_anywhere_gate.jsonl
import json, re, sys
import numpy as np
from scipy.spatial import cKDTree
name, day, base = sys.argv[1:4]
row = json.loads(open(f'eval_{name}.json').readline())
log = open(f'logs/run_any_{name}.txt').read()
timing = json.load(open(f'runs/any_{name}/live_timing.json'))
start = re.search(r'"store_start_stamp_sec": ([0-9.]+)', log)
anchor = re.search(r'"store_anchor_stamp_sec": ([0-9.]+)', log)
start = float(start.group(1)) if start else None
row['anchor_after_s'] = float(anchor.group(1)) - start if anchor and start else None
# Entering the store: the first ground-truth position of the run within 5 m (XY) of a
# ground-truth position of the base session's window.
windows = json.load(open('/home/user/lidar_slam/bench_report/data/long_term_2026-11/nclt/anywhere_gate_windows.json'))
b0, b1 = windows['base_window_utime']
bg = np.load(f'gt/{base}.npy')
bg = bg[(bg[:, 0] >= b0) & (bg[:, 0] <= b1)]
sg = np.load(f'gt/{day}.npy')
odometry = np.loadtxt(f'runs/any_{name}/odometry_tum.txt', ndmin=2)
t0, t1 = odometry[0, 0] * 1e6, odometry[-1, 0] * 1e6
sg = sg[(sg[:, 0] >= t0) & (sg[:, 0] <= t1)]
near = cKDTree(bg[:, 1:3]).query(sg[:, 1:3])[0] <= 5.0
row['enters_store_after_s'] = (float(sg[np.argmax(near), 0]) * 1e-6 - start
                               if near.any() and start else None)
row['anchor_after_entering_s'] = (row['anchor_after_s'] - row['enters_store_after_s']
                                  if row['anchor_after_s'] is not None and
                                  row['enters_store_after_s'] is not None else None)
row['data_s'] = timing['data_seconds']
row['lag_seconds'] = timing['lag_seconds']
if row['anchored']:
    row['correct_v2'] = ('best_apart_m' in row and row['best_apart_m'] <= 0.25 and
                         row['best_apart_deg'] <= 0.5 and row['gt_offset_rms_m'] <= 4.0)
print(json.dumps(row))
PY
    rm -rf stores/any_$name/sessions stores/any_$name/export
  done
done
cat holdout_anywhere_gate.jsonl
