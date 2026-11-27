#!/usr/bin/env python3
"""Measure the odometry's drift per difficulty setting on fixed routes (no exploration policy).

The env's difficulty (pilaster spacing, IMU noise scale) is set so that RKO-LIO's drift on the
development worlds matches a target stated beforehand (docs/roadmap/active-slam-2026-11.md),
without running any exploration policy: every run drives the world's fixed route
(worlds.ROUTES) on the true pose and reports the odometry's mean drift over 50 m segments.

  python3 tools/rne_active_slam_env/calibrate_difficulty.py --out /tmp/calib
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import itertools
import json
from pathlib import Path
import statistics
import subprocess
import sys

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def run_route(config: dict, world: str, seed: int, spacing: int, imu: float, out: Path) -> dict:
    """Drive one route (or read its earlier result)."""
    metrics_file = out / 'metrics.json'
    if not metrics_file.is_file():
        out.mkdir(parents=True, exist_ok=True)
        command = [
            sys.executable, str(HERE / 'run_closed_loop.py'), '--route', '--world', world,
            '--seed', str(seed), '--out', str(out), '--max-seconds', '900',
            '--divergence-m', '1000', '--pilaster-spacing', str(spacing),
            '--imu-noise-scale', str(imu),
            '--rko-param', str(ROOT / config['rko_param']),
            '--lidarslam-param', str(ROOT / config['lidarslam_param']),
        ]
        with open(out / 'run.log', 'w', encoding='utf-8') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    if not metrics_file.is_file():
        return {'world': world, 'seed': seed, 'pilaster_spacing_m': spacing,
                'imu_noise_scale': imu, 'odometry_drift_50m_pct': None}
    return json.loads(metrics_file.read_text())


def main(argv: list[str] | None = None) -> int:
    """Run the sweep and write calibration.json / calibration.md."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=HERE / 'eval_config.yaml')
    parser.add_argument('--spacings', type=int, nargs='+', default=[0, 12, 24, -1],
                        help='pilaster spacings [m] (0: as built, -1: none)')
    parser.add_argument('--imu-noise-scales', type=float, nargs='+', default=[1.0, 5.0])
    parser.add_argument('--seeds', type=int, nargs='+', default=[1])
    parser.add_argument('--jobs', type=int, default=3)
    args = parser.parse_args(argv)

    config = yaml.safe_load(args.config.read_text())
    worlds = config['sets']['development']['worlds']
    out = args.out.resolve()
    cells = list(itertools.product(worlds, args.seeds, args.spacings, args.imu_noise_scales))
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(lambda c: run_route(
            config, c[0], c[1], c[2], c[3], out / f'{c[0]}_s{c[1]}_p{c[2]}_i{c[3]:g}'), cells))
    rows = []
    for spacing, imu in itertools.product(args.spacings, args.imu_noise_scales):
        drifts = {r['world']: r.get('odometry_drift_50m_pct') for r in results
                  if r['pilaster_spacing_m'] == spacing and r['imu_noise_scale'] == imu}
        known = [d for d in drifts.values() if d is not None]
        rows.append({'pilaster_spacing_m': spacing, 'imu_noise_scale': imu, 'drift_pct': drifts,
                     'median_drift_pct': round(statistics.median(known), 4) if known else None})
    (out / 'calibration.json').write_text(json.dumps(rows, indent=2) + '\n')
    lines = ['| pilaster spacing [m] | IMU noise scale | ' + ' | '.join(worlds) +
             ' | median |', '| --- | --- | ' + ' | '.join('---' for _ in worlds) + ' | --- |']
    for row in rows:
        lines.append(f"| {row['pilaster_spacing_m']} | {row['imu_noise_scale']:g} | " +
                     ' | '.join(str(row['drift_pct'].get(w)) for w in worlds) +
                     f" | {row['median_drift_pct']} |")
    (out / 'calibration.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
