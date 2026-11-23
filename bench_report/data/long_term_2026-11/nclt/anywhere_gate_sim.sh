# sim.sh STORE BAG NAME : gate simulation of a recorded session, then drop its saved inputs
set -u
cd /home/user/bench/nclt
start=$(grep -o '"store_start_stamp_sec": [0-9.]*' gate/log_$3.txt | grep -o '[0-9.]*$')
python3 -I /home/user/lidar_slam/bench_report/data/long_term_2026-11/nclt/gate_study.py --store stores/$1 --queries gate/q/$3.npz --start-stamp $start --gates 0.18 0.20 0.22 0.24 0.26 --ranks gate/ranks_$3.jsonl --label $3 > gate/sim_$3.jsonl 2> gate/sim_$3.err
echo "$3 exit $?"
