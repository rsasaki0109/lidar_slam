#!/usr/bin/env bash
# Development (roadmap, "Where the removals of store E are contradicted"): append each session
# outside the chain (nclt/fresh_windows.json, except 2012-04-29, which overlaps the store little, and
# 2013-04-05, whose download is slow) to its own
# hard-linked copy of stores/lifelong_trace (E) as in trace_audit_run.sh, and keep only the
# appended session's change evidence (removal_breakdown/$d.npz) for removal_breakdown.py.
# Run from the bench directory.
set -u
R=$(cd "$(dirname "$0")/../../../.." && pwd)
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
O="--odometry-params $R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml --backend-params $R/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml"
W=removal_breakdown
mkdir -p $W
for d in $(python3 -c "import json;print(' '.join(k for k,v in json.load(open('fresh_windows.json')).items() if v and k not in ('2012-04-29', '2013-04-05')))"); do
  [ -s "$W/$d.npz" ] && continue
  mkdir -p "$W/$d" "sen/$d"
  [ -s "sen/$d/ms25.csv" ] || curl -s "$B/sensor_data/${d}_sen.tar.gz" | tar -xzO --wildcards "*/ms25.csv" > "sen/$d/ms25.csv"
  w=$(python3 -c "import json;w=json.load(open('fresh_windows.json'))['$d']['utime'];print(w[0],w[1])")
  set -- $w
  rm -rf "$W/$d/bag" "$W/$d/E" "$W/$d/run"
  python3 -I "$R/scripts/convert_nclt_session.py" "$B/velodyne_data/${d}_vel.tar.gz" "sen/$d/ms25.csv" \
    "$W/$d/bag" --point-step 6 --from-utime "$1" --to-utime "$2" > "$W/$d/convert.log" 2>&1
  cp -al stores/lifelong_trace "$W/$d/E"
  python3 -I -m lidarslam live "$W/$d/bag" $O --out "$W/$d/run" --store "$W/$d/E" \
    --relocalize-anywhere --detect-changes --max-height-drift 0 > "$W/$d/live.log" 2>&1
  echo "$d live exit $?" >> $W/progress.txt
  e="$W/$d/E/sessions/0020/map_change_evidence.npz"
  [ -s "$e" ] && cp "$e" "$W/$d.tmp.npz" && mv "$W/$d.tmp.npz" "$W/$d.npz"
  rm -rf "$W/$d"
done
