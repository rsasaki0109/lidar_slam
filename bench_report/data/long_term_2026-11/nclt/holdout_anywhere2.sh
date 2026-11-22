# Second holdout check of live --relocalize-anywhere (preregistered in
# docs/roadmap/long-term-mapping-2026-11.md before this script was run).
# Bags: convert_window.sh with anywhere_holdout2_windows.json (DAY_in, DAY_out); store: h_store.
set -u
cd /home/user/bench/nclt
R=/home/user/lidar_slam
D=$R/bench_report/data/long_term_2026-11/nclt
DAYS="2012-02-02 2012-03-17 2012-09-28 2012-10-28"
python3 -I $D/gt_npy.py /home/user/bench/nclt full $DAYS
for day in $DAYS; do for part in in out; do
  name=${day}_$part
  bash $D/run_anywhere.sh h_store $name h2_$name
  python3 -I $R/scripts/evaluate_relocalize_anywhere.py --base-map runs/h_base/map.pcd \
    --base-odometry runs/h_base/odometry_tum.txt --base-gt gt/2012-02-18.npy \
    --session-map runs/any_h2_$name/map.pcd --session-odometry runs/any_h2_$name/odometry_tum.txt \
    --session-gt gt/$day.npy --anchor stores/any_h2_$name/store.json --label $name \
    > eval_h2_$name.json
  python3 -I - "$name" <<'PY' >> holdout_anywhere2.jsonl
import json, re, sys
name = sys.argv[1]
row = json.loads(open(f'eval_h2_{name}.json').readline())
log = open(f'logs/run_any_h2_{name}.txt').read()
timing = json.load(open(f'runs/any_h2_{name}/live_timing.json'))
start = re.search(r'"store_start_stamp_sec": ([0-9.]+)', log)
anchor = re.search(r'"store_anchor_stamp_sec": ([0-9.]+)', log)
row['anchor_after_s'] = float(anchor.group(1)) - float(start.group(1)) if anchor and start else None
row['data_s'] = timing['data_seconds']
row['processing_s'] = timing['processing_wall_seconds']
if row['anchored']:
    row['correct'] = (row['converged_apart_m'] <= 0.25 and row['converged_apart_deg'] <= 0.5
                      and row['gt_offset_rms_m'] <= 4.0)
print(json.dumps(row))
PY
done; done
cat holdout_anywhere2.jsonl
