#!/usr/bin/env bash
# The two sessions the height check refuses, appended again with the tilt correction (the live
# defaults) to a copy of the lifelong store cut before them (preregistered, roadmap). Run from the
# bench directory; scored by refused_rerun_score.py.
set -u
R=$(cd "$(dirname "$0")/../../../.." && pwd)
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
for entry in 2012-03-31:2012-03-31_live2in:7 2012-11-16:2012-11-16:16; do
  IFS=: read -r day bag k <<< "$entry"
  d=refused/$day
  [ -f "$d/done" ] && continue
  rm -rf "$d" && mkdir -p "$d/store/sessions"
  for i in $(seq 0 $((k - 1))); do cp -al "stores/lifelong/sessions/$(printf %04d "$i")" "$d/store/sessions/"; done
  python3 -I -c "import json,sys;s=json.load(open('stores/lifelong/store.json'));s['sessions']=s['sessions'][:$k];json.dump(s,open('$d/store/store.json','w'),indent=1)"
  python3 -I -m lidarslam live "bags/$bag" $O --out "$d/run" --store "$d/store" \
    --relocalize-anywhere --detect-changes > "$d/live.log" 2>&1
  echo "$day live exit $?" >> refused/progress.txt
  rm -rf "$d/store/export" "$d/run/pointcloud_map" "$d/run/map.pcd"
  touch "$d/done"
done
