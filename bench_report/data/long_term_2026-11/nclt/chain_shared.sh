# chain_shared.sh P BASE UPD R1 R2 : one NCLT map-update chain (arms A, B, C from the stored base
# store stores/${P}_store and run runs/${P}_base), scored with both references placed once by ICP
# against arm C's export (old and new structure), the same placement for every arm.
set -eu
cd /home/user/bench/nclt
R=/home/user/lidar_slam_ros2
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
P=$1; BASE=$2; UPD=$3; R1=$4; R2=$5
run() { [ -f runs/$1/odometry_tum.txt ] || python3 -I -m lidarslam live bags/$2 $O --out runs/$1 > logs/run_$1.txt 2>&1; }
run ${P}_upd_alone $UPD & run ${P}_ref1 $R1 & run ${P}_ref2 $R2 & wait
SKIP=$(python3 -I -c "
import json, numpy as np
from scipy.spatial import cKDTree
W = json.load(open('windows.json'))['windows_utime']
gb = np.load('gt/$BASE.npy'); gu = np.load('gt/$UPD.npy')
b = gb[(gb[:, 0] >= W['$BASE'][0]) & (gb[:, 0] <= W['$BASE'][1])]
u = gu[(gu[:, 0] >= W['$UPD'][0]) & (gu[:, 0] <= W['$UPD'][1])]
near = np.flatnonzero(cKDTree(b[:, 1:3]).query(u[:, 1:3])[0] < 1.5)
print(int((u[near[0], 0] - W['$UPD'][0]) / 1e6))")
echo "trim $SKIP s"
[ -f bags/${UPD}_trim${SKIP}/metadata.yaml ] || python3 -I $R/bench_report/data/long_term_2026-11/nclt/trim.py bags/$UPD bags/${UPD}_trim${SKIP} $SKIP
START=$(python3 -I -c "
from rosbags.rosbag2 import Reader
with Reader('bags/${UPD}_trim${SKIP}') as r: print(r.start_time / 1e9)")
PRIOR=$(python3 -c "import prior; print(prior.trimmed_prior('${P}_base', '$BASE', '${P}_upd_alone', '$UPD', $START))")
python3 prior.py ${P}_base $BASE ${P}_upd_alone $UPD ${P}_store_from_world.json > /dev/null
echo "prior $PRIOR"
arm() {
  case $1 in A) F="--detect-changes --filter-new-points";; B) F="--detect-changes --refine-visibility";; C) F="";; esac
  rm -rf stores/${P}_$1 runs/${P}_upd_$1; cp -a stores/${P}_store stores/${P}_$1
  python3 -I -m lidarslam live bags/${UPD}_trim${SKIP} $O --out runs/${P}_upd_$1 --store stores/${P}_$1 --initial-pose $PRIOR $F > logs/run_${P}_$1.txt 2>&1
}
arm A & arm B & arm C & wait
for r in 1 2; do
  day=$([ $r = 1 ] && echo $R1 || echo $R2)
  python3 -c "
import json, numpy as np, prior
sfw = np.array(json.load(open('${P}_store_from_world.json')))
json.dump((sfw @ prior.world_from_run('${P}_ref$r', '$day')).tolist(), open('${P}_ref${r}_init.json', 'w'))"
done
for a in A B C; do
  python3 -I $R/scripts/evaluate_nclt_map_update.py --before runs/${P}_base/map.pcd --after stores/${P}_$a/export/map.pcd --path runs/${P}_upd_$a/trajectory_store.tum --ref runs/${P}_ref1/map.pcd runs/${P}_ref1/odometry_tum.txt ${P}_ref1_init.json --ref runs/${P}_ref2/map.pcd runs/${P}_ref2/odometry_tum.txt ${P}_ref2_init.json --match 0.5 --align-target stores/${P}_C/export/map.pcd > ${P}_shared_$a.json &
  python3 -I $R/scripts/evaluate_nclt_map_update.py --before runs/${P}_base/map.pcd --after stores/${P}_$a/export/map.pcd --path runs/${P}_upd_$a/trajectory_store.tum --ref runs/${P}_ref1/map.pcd runs/${P}_ref1/odometry_tum.txt ${P}_ref1_init.json --ref runs/${P}_ref2/map.pcd runs/${P}_ref2/odometry_tum.txt ${P}_ref2_init.json --match 0.5 > ${P}_own_$a.json &
done; wait
for a in A B C; do python3 -I -c "
import json
for k in ('own', 'shared'):
    d = json.load(open('${P}_' + k + '_$a.json'))
    print('$a', k, {x: d[x] for x in ('ghost_points', 'G', 'S', 'N', 'new_points', 'persistent_points')}, [round(r['correction_m'], 2) for r in d['references']])"; done
