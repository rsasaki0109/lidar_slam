set -eu
cd /home/user/bench/nclt
export PYTHONPATH=/home/user/ws/core_dev/pkg
O="--odometry-params /home/user/lidar_slam_ros2/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params /home/user/lidar_slam_ros2/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
BASE=2012-02-18; UPD=2012-11-16; R1=2012-11-04; R2=2012-11-17
python3 -m lidarslam live bags/$BASE $O --out runs/h_base --store stores/h_store > logs_run_h_base.txt 2>&1
python3 -m lidarslam live bags/$UPD $O --out runs/h_upd_alone > logs_run_h_upd_alone.txt 2>&1
SKIP=$(python3 -c "
import json, numpy as np
from scipy.spatial import cKDTree
W = json.load(open('windows.json'))['windows_utime']
gb = np.load('gt/$BASE.npy'); gu = np.load('gt/$UPD.npy')
b = gb[(gb[:, 0] >= W['$BASE'][0]) & (gb[:, 0] <= W['$BASE'][1])]
u = gu[(gu[:, 0] >= W['$UPD'][0]) & (gu[:, 0] <= W['$UPD'][1])]
near = np.flatnonzero(cKDTree(b[:, 1:3]).query(u[:, 1:3])[0] < 1.5)
print(int((u[near[0], 0] - W['$UPD'][0]) / 1e6))")
echo "trim $SKIP s"
python3 trim.py bags/$UPD bags/${UPD}_trim $SKIP
START=$(python3 -c "
from rosbags.rosbag2 import Reader
with Reader('bags/${UPD}_trim') as r: print(r.start_time / 1e9)")
P=$(python3 -c "import prior; print(prior.trimmed_prior('h_base', '$BASE', 'h_upd_alone', '$UPD', $START))")
python3 prior.py h_base $BASE h_upd_alone $UPD h_store_from_world.json > /dev/null
echo "prior $P"
for arm in A B C; do
  case $arm in A) F="--detect-changes --filter-new-points";; B) F="--detect-changes --refine-visibility";; C) F="";; esac
  cp -a stores/h_store stores/h_$arm
  python3 -m lidarslam live bags/${UPD}_trim $O --out runs/h_upd_$arm --store stores/h_$arm --initial-pose $P $F > logs_run_h_$arm.txt 2>&1
  python3 -c "
import json; d = json.load(open('stores/h_$arm/store.json')); print('$arm anchored', 'store_from_session' in d['sessions'][-1])"
done
for r in 1 2; do
  day=$([ $r = 1 ] && echo $R1 || echo $R2)
  python3 -m lidarslam live bags/$day $O --out runs/h_ref$r > logs_run_h_ref$r.txt 2>&1
  python3 -c "
import json, numpy as np, prior
sfw = np.array(json.load(open('h_store_from_world.json')))
json.dump((sfw @ prior.world_from_run('h_ref$r', '$day')).tolist(), open('h_ref${r}_init.json', 'w'))"
done
for arm in A B C; do
  python3 /home/user/lidar_slam_ros2/scripts/evaluate_nclt_map_update.py --before runs/h_base/map.pcd --after stores/h_$arm/export/map.pcd --path runs/h_upd_$arm/trajectory_store.tum --ref runs/h_ref1/map.pcd runs/h_ref1/odometry_tum.txt h_ref1_init.json --ref runs/h_ref2/map.pcd runs/h_ref2/odometry_tum.txt h_ref2_init.json --match 0.5 > holdout_$arm.json
  python3 -c "
import json; d = json.load(open('holdout_$arm.json')); print('$arm', {k: v for k, v in d.items() if k != 'references'})"
done
