# rec.sh STORE BAG NAME : record every keyframe's anywhere-search input (gate study)
set -u
cd /home/user/bench/nclt
R=/home/user/lidar_slam
D=$R/bench_report/data/long_term_2026-11/nclt
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
rm -rf gate/st_$3 gate/run_$3; cp -a stores/$1 gate/st_$3
s=$(date +%s)
python3 -I $D/record_anywhere_queries.py gate/q/$3.npz live bags/$2 $O --out gate/run_$3 --store gate/st_$3 --relocalize-anywhere > gate/log_$3.txt 2>&1
echo "$3 exit $? $(( $(date +%s) - s )) s"
rm -rf gate/st_$3
