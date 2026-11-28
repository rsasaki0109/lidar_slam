#!/usr/bin/env bash
# The preregistered audit of the change evidence of two lifelong stores (roadmap, "Tracing the
# change evidence where the session is committed"): each session outside the chain
# (nclt/fresh_windows.json) is appended, with the live defaults, to its own hard-linked copy of
# stores/lifelong_profile (D, traced at the anchor) and of stores/lifelong_trace (E, traced after
# the corrections); trace_audit_score.py scores the store's removals against the appended
# session's evidence, and the copy is deleted. Run from the bench directory.
set -u
R=$(cd "$(dirname "$0")/../../../.." && pwd)
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
mkdir -p trace_audit
for d in $(python3 -c "import json;print(' '.join(k for k,v in json.load(open('fresh_windows.json')).items() if v))"); do
  [ -f "trace_audit/$d.done" ] && continue
  mkdir -p "trace_audit/$d" "sen/$d"
  [ -s "sen/$d/ms25.csv" ] || curl -s "$B/sensor_data/${d}_sen.tar.gz" | tar -xzO --wildcards "*/ms25.csv" > "sen/$d/ms25.csv"
  w=$(python3 -c "import json;w=json.load(open('fresh_windows.json'))['$d']['utime'];print(w[0],w[1])")
  set -- $w
  rm -rf "trace_audit/$d/bag"
  python3 -I "$R/scripts/convert_nclt_session.py" "$B/velodyne_data/${d}_vel.tar.gz" "sen/$d/ms25.csv" \
    "trace_audit/$d/bag" --point-step 6 --from-utime "$1" --to-utime "$2" > "trace_audit/$d/convert.log" 2>&1
  for s in lifelong_profile:D lifelong_trace:E; do
    store=${s%%:*}; label=${s#*:}
    rm -rf "trace_audit/$d/$label" && cp -al "stores/$store" "trace_audit/$d/$label"
    python3 -I -m lidarslam live "trace_audit/$d/bag" $O --out "trace_audit/$d/run_$label" \
      --store "trace_audit/$d/$label" --relocalize-anywhere --detect-changes --max-height-drift 0 \
      > "trace_audit/$d/live_$label.log" 2>&1
    echo "$d $label live exit $?" >> trace_audit/progress.txt
    python3 -I "$R/bench_report/data/long_term_2026-11/lifelong/trace_audit_score.py" \
      "trace_audit/$d/$label" "$d" "$label" >> trace_audit/score.jsonl
    rm -rf "trace_audit/$d/$label" "trace_audit/$d/run_$label/pointcloud_map" "trace_audit/$d/run_$label/map.pcd"
  done
  rm -rf "trace_audit/$d/bag"
  touch "trace_audit/$d.done"
done
