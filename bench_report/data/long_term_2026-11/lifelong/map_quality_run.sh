#!/usr/bin/env bash
# The preregistered map-quality comparison of two lifelong stores (roadmap, "The map quality of
# the corrected lifelong store"): the export of stores/lifelong (A) and of stores/lifelong_trace
# (E) are scored three times each by graph_based_slam's map_quality_report with its frozen metric
# profile and a 0.1 m downsample (as scripts/run_map_quality_check.sh), and every run of a map
# must give a byte-identical report. Run from the bench directory:
#   map_quality_run.sh <map_quality_report binary> > map_quality.jsonl
# The binary is the ROS one (ros2 run graph_based_slam map_quality_report) or the same source
# built without ROS:
#   g++ -O2 -std=c++17 -fopenmp -Igraph_based_slam/include -I/usr/include/eigen3 \
#     -I/usr/include/pcl-1.14 graph_based_slam/src/map_quality_report_main.cpp \
#     -o map_quality_report $(pkg-config --libs pcl_io pcl_common)
set -eu
BIN=$1
for s in lifelong:A lifelong_trace:E; do
  store=${s%%:*}; label=${s#*:}
  sums=""
  for run in 1 2 3; do
    out=map_quality/$label/run$run
    rm -rf "$out" && mkdir -p "$out"
    "$BIN" --input "stores/$store/export/map.pcd" --output-dir "$out" --downsample 0.1 > "$out.log" 2>&1
    sums="$sums $(md5sum "$out/map_quality_report.yaml" | cut -d' ' -f1)"
  done
  python3 -I - "$label" "map_quality/$label/run1/map_quality_report.yaml" $sums <<'PY'
import json, sys
label, path, *sums = sys.argv[1:]
r = {'label': label, 'deterministic': len(set(sums)) == 1}
lines = open(path).read().split('\n')
section = ''
for line in lines:
    key, _, value = line.strip().partition(':')
    if not value.strip():
        section = key
    elif section in ('mean_map_entropy', 'plane_metrics', 'density', 'map_quality_report'):
        r[f'{section}.{key}'] = value.strip()
print(json.dumps(r))
PY
done
