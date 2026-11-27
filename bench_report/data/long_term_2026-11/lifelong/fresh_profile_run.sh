#!/usr/bin/env bash
# The preregistered height-profile check on sessions outside the lifelong chain: convert each
# session's lifelong-square pass (fresh_windows.json), append it to its own copy of the lifelong
# store rebuilt with the profile (stores/lifelong_profile) without correction, keep the committed
# session, drop the bag and the export. Run from the bench directory; scored by
# lifelong/fresh_profile_score.py.
set -u
R=$(cd "$(dirname "$0")/../../../.." && pwd)
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
mkdir -p fresh_profile
for d in $(python3 -c "import json;print(' '.join(k for k,v in json.load(open('fresh_windows.json')).items() if v))"); do
  [ -f "fresh_profile/$d/done" ] && continue
  mkdir -p "fresh_profile/$d" "sen/$d"
  [ -s "sen/$d/ms25.csv" ] || curl -s "$B/sensor_data/${d}_sen.tar.gz" | tar -xzO --wildcards "*/ms25.csv" > "sen/$d/ms25.csv"
  w=$(python3 -c "import json;w=json.load(open('fresh_windows.json'))['$d']['utime'];print(w[0],w[1])")
  set -- $w
  rm -rf "fresh_profile/$d/bag"
  python3 -I "$R/scripts/convert_nclt_session.py" "$B/velodyne_data/${d}_vel.tar.gz" "sen/$d/ms25.csv" \
    "fresh_profile/$d/bag" --point-step 6 --from-utime "$1" --to-utime "$2" > "fresh_profile/$d/convert.log" 2>&1
  rm -rf "fresh_profile/$d/store" && cp -al stores/lifelong_profile "fresh_profile/$d/store"
  python3 -I -m lidarslam live "fresh_profile/$d/bag" $O --out "fresh_profile/$d/run" --store "fresh_profile/$d/store" \
    --relocalize-anywhere --detect-changes --max-height-drift 0 --no-tilt-correction --no-height-profile > "fresh_profile/$d/live.log" 2>&1
  echo "$d live exit $?" >> fresh_profile/progress.txt
  rm -rf "fresh_profile/$d/bag" "fresh_profile/$d/store/export" "fresh_profile/$d/run/pointcloud_map" "fresh_profile/$d/run/map.pcd"
  touch "fresh_profile/$d/done"
done
