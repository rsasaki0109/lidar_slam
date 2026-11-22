# run_anywhere.sh STORE BAG NAME : live --relocalize-anywhere on a copy of STORE, plus the session alone
set -u
cd /home/user/bench/nclt
R=/home/user/lidar_slam
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
rm -rf stores/any_$3 runs/any_$3; cp -a stores/$1 stores/any_$3
s=$(date +%s)
python3 -I -m lidarslam live bags/$2 $O --out runs/any_$3 --store stores/any_$3 --relocalize-anywhere > logs/run_any_$3.txt 2>&1
echo "$3 exit $? $(( $(date +%s) - s )) s"
