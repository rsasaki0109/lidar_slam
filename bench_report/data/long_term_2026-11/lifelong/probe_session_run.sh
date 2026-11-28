#!/usr/bin/env bash
# Development (roadmap, "Why 2012-04-29 contradicts the removals"): append one session outside
# the chain (nclt/fresh_windows.json) to a hard-linked copy of stores/lifelong_trace (E) as in
# trace_audit_run.sh, and keep the appended session (probe/$DAY/session) and its live log for
# probe_session.py. Run from the bench directory: probe_session_run.sh DAY
set -eu
R=$(cd "$(dirname "$0")/../../../.." && pwd)
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
d=$1
W=probe/$d
rm -rf "$W" && mkdir -p "$W" "sen/$d"
[ -s "sen/$d/ms25.csv" ] || curl -s "$B/sensor_data/${d}_sen.tar.gz" | tar -xzO --wildcards "*/ms25.csv" > "sen/$d/ms25.csv"
set -- $(python3 -c "import json;w=json.load(open('fresh_windows.json'))['$d']['utime'];print(w[0],w[1])")
python3 -I "$R/scripts/convert_nclt_session.py" "$B/velodyne_data/${d}_vel.tar.gz" "sen/$d/ms25.csv" \
  "$W/bag" --point-step 6 --from-utime "$1" --to-utime "$2" > "$W/convert.log" 2>&1
cp -al stores/lifelong_trace "$W/E"
python3 -I -m lidarslam live "$W/bag" $O --out "$W/run" --store "$W/E" \
  --relocalize-anywhere --detect-changes --max-height-drift 0 > "$W/live.log" 2>&1 || true
python3 -c "import json;s=json.load(open('$W/E/store.json'))['sessions'];print(json.dumps(s[20]) if len(s)>20 else 'not committed')" > "$W/session_entry.json"
[ -d "$W/E/sessions/0020" ] && cp -r "$W/E/sessions/0020" "$W/session"
rm -rf "$W/E" "$W/bag" "$W/run/pointcloud_map" "$W/run/map.pcd"
