#!/usr/bin/env bash
# Odometry of every lifelong update with rko_lio_nclt.yaml (A) and with
# moving_start_gravity_window_sec: 10.0 (G); see the preregistered check in
# docs/roadmap/long-term-mapping-2026-11.md. Run from the bench directory of lifelong_chain.sh.
set -euo pipefail
R=$(cd "$(dirname "$0")/../../../.." && pwd)
P=$R/bench_report/data/long_term_2026-11/rko_lio_nclt.yaml
mkdir -p gravity
cp "$P" gravity/A.yaml
{ cat "$P"; echo "moving_start_gravity_window_sec: 10.0"; } > gravity/G.yaml
SESSIONS=$(sed -n '/^SESSIONS="/,/"$/p' "$R/bench_report/data/long_term_2026-11/lifelong/lifelong_chain.sh" | tr -d '"' | sed 's/^SESSIONS=//')
for entry in $SESSIONS; do
  day=${entry%%:*}; bag=${entry#*:}
  for arm in A G; do
    [ -f "gravity/$arm/$day/odometry_tum.txt" ] && continue
    python3 -I -m lidarslam odometry "bags/$bag" --params "gravity/$arm.yaml" --out "gravity/$arm/$day" \
      > "gravity/$arm-$day.log" 2>&1 &
  done
  wait
done
for arm in A G; do
  python3 -I "$R/bench_report/data/long_term_2026-11/lifelong/odometry_height.py" . "gravity/$arm" --label "$arm"
done
