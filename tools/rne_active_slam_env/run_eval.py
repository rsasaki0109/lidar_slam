#!/usr/bin/env python3
"""Run the closed-loop active SLAM evaluation table and judge it (roadmap P0).

Runs every world x seed x policy of a set from eval_config.yaml with run_closed_loop.py (one
process per run; runs with a metrics.json are kept, so an interrupted table resumes), then
writes summary.json and summary.md: per world and policy the medians, and the paired comparison
of ``active`` against ``frontier_info`` with the success rule of the roadmap.

The holdout set is run once, after the freeze: it needs --frozen, a clean git tree, and writes
the commit and the config's SHA-256 next to the results.

  python3 tools/rne_active_slam_env/run_eval.py --set development --out /tmp/eval_dev
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def run_one(config: dict[str, Any], world: str, seed: int, policy: str, out: Path) -> dict:
    """Run one cell of the table (or read its earlier result)."""
    metrics_file = out / 'metrics.json'
    if metrics_file.is_file():
        return json.loads(metrics_file.read_text())
    # Per-world settings (the calibrated difficulty, a budget for the world's size).
    config = {**config, **config.get('world_overrides', {}).get(world, {})}
    command = [
        sys.executable, str(HERE / 'run_closed_loop.py'), '--world', world, '--seed', str(seed),
        '--policy', policy, '--out', str(out), '--max-seconds', str(config['max_seconds']),
        '--divergence-m', str(config['divergence_m']),
        '--inflation-radius', str(config['inflation_radius']),
        '--revisit-trigger-sigma', str(config['revisit_trigger_sigma']),
        '--revisit-release-sigma', str(config['revisit_release_sigma']),
        '--uncertainty-weight', str(config['uncertainty_weight']),
        '--min-predicted-drop', str(config['min_predicted_drop']),
        *(['--absolute-gain'] if config.get('absolute_gain') else []),
        '--pilaster-spacing', str(config.get('pilaster_spacing', 0)),
        '--rko-param', str(ROOT / config['rko_param']),
        '--lidarslam-param', str(ROOT / config['lidarslam_param']),
    ]
    out.mkdir(parents=True, exist_ok=True)
    with open(out / 'run.log', 'w', encoding='utf-8') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    if result.returncode != 0 or not metrics_file.is_file():
        return {'world': world, 'seed': seed, 'policy': policy, 'error': result.returncode,
                'failed': True}
    return json.loads(metrics_file.read_text())


def _median(values: list[float]) -> float | None:
    values = [v for v in values if v is not None]
    return round(statistics.median(values), 4) if values else None


def judge(results: list[dict[str, Any]], worlds: list[str], seeds: list[int]) -> dict[str, Any]:
    """Return the per-world medians and the success rule of active vs frontier_info."""
    by_key = {(r['world'], r['seed'], r['policy']): r for r in results}
    summary: dict[str, Any] = {'worlds': {}}
    passes = []
    for world in worlds:
        cells: dict[str, Any] = {}
        for policy in sorted({r['policy'] for r in results}):
            runs = [by_key.get((world, s, policy)) for s in seeds]
            runs = [r for r in runs if r is not None]
            cells[policy] = {
                'runs': len(runs),
                'failed': sum(1 for r in runs if r.get('failed')),
                'median_final_map_ate_rmse_m': _median(
                    [r.get('final_map_ate_rmse_m') for r in runs]),
                'median_coverage': _median([r.get('coverage') for r in runs]),
                'median_distance_m': _median([r.get('distance_m') for r in runs]),
                'median_live_error_rmse_m': _median(
                    [r.get('live_position_error_rmse_m') for r in runs]),
                'median_coverage_90_seconds': _median(
                    [r.get('coverage_90_seconds') for r in runs]),
                'revisit_goal_frames': sum(r.get('revisit_goal_frames') or 0 for r in runs),
            }
        reductions, coverage_diffs, wins = [], [], 0
        for seed in seeds:
            a = by_key.get((world, seed, 'active'))
            b = by_key.get((world, seed, 'frontier_info'))
            if not a or not b:
                continue
            ate_a, ate_b = a.get('final_map_ate_rmse_m'), b.get('final_map_ate_rmse_m')
            if ate_a is not None and ate_b:
                reductions.append((ate_b - ate_a) / ate_b)
                wins += int(ate_a < ate_b)
            if a.get('coverage') is not None and b.get('coverage') is not None:
                coverage_diffs.append(100.0 * (a['coverage'] - b['coverage']))
        failed_a = cells.get('active', {}).get('failed', 0)
        failed_b = cells.get('frontier_info', {}).get('failed', 0)
        rule = {
            'median_ate_reduction': _median(reductions),
            'active_lower_ate_seeds': wins, 'paired_seeds': len(reductions),
            'median_coverage_difference_points': _median(coverage_diffs),
            'failed_active': failed_a, 'failed_frontier_info': failed_b,
        }
        n = len(seeds)
        rule['pass'] = bool(
            rule['median_ate_reduction'] is not None and rule['median_ate_reduction'] >= 0.15 and
            wins >= round(0.7 * n) and
            rule['median_coverage_difference_points'] is not None and
            rule['median_coverage_difference_points'] >= -5.0 and failed_a <= failed_b)
        passes.append(rule['pass'])
        summary['worlds'][world] = {'policies': cells, 'active_vs_frontier_info': rule}
    summary['pass'] = bool(passes) and all(passes)
    return summary


def markdown(summary: dict[str, Any], header: str) -> str:
    lines = [f'# {header}', '']
    for world, data in summary['worlds'].items():
        lines += [f'## {world}', '',
                  '| policy | runs | failed | median final-map ATE [m] | median coverage | '
                  'median distance [m] | median live error RMSE [m] | median t(90 %) [s] | '
                  'revisit goal frames |',
                  '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
        for policy, c in data['policies'].items():
            lines.append(
                f"| {policy} | {c['runs']} | {c['failed']} | {c['median_final_map_ate_rmse_m']} "
                f"| {c['median_coverage']} | {c['median_distance_m']} | "
                f"{c['median_live_error_rmse_m']} | {c['median_coverage_90_seconds']} | "
                f"{c['revisit_goal_frames']} |")
        r = data['active_vs_frontier_info']
        lines += ['', 'active vs frontier_info: median ATE reduction '
                  f"{r['median_ate_reduction']}, active lower in {r['active_lower_ate_seeds']} "
                  f"of {r['paired_seeds']} seeds, median coverage difference "
                  f"{r['median_coverage_difference_points']} points, failed "
                  f"{r['failed_active']} vs {r['failed_frontier_info']}: "
                  f"**{'pass' if r['pass'] else 'fail'}**", '']
    lines.append(f"Overall: **{'pass' if summary['pass'] else 'fail'}**")
    return '\n'.join(lines) + '\n'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--set', required=True, choices=['development', 'tunnels', 'holdout'])
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=HERE / 'eval_config.yaml')
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--frozen', action='store_true',
                        help='holdout: the configuration is frozen and committed')
    args = parser.parse_args(argv)

    config_bytes = args.config.read_bytes()
    config = yaml.safe_load(config_bytes)
    provenance = {'config_sha256': hashlib.sha256(config_bytes).hexdigest()}
    commit = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True,
                            text=True, check=False).stdout.strip()
    dirty = subprocess.run(['git', '-C', str(ROOT), 'status', '--porcelain'],
                           capture_output=True, text=True, check=False).stdout.strip()
    provenance.update({'commit': commit, 'dirty_tree': bool(dirty)})
    if args.set == 'holdout' and (not args.frozen or dirty):
        print('error: the holdout set runs once, at a frozen and committed configuration: pass '
              '--frozen and commit every change first', file=sys.stderr)
        return 2

    chosen = config['sets'][args.set]
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    cells = [(w, s, p) for w in chosen['worlds'] for s in chosen['seeds']
             for p in config['policies']]
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(
            lambda c: run_one(config, c[0], c[1], c[2], out / f'{c[0]}_s{c[1]}_{c[2]}'), cells))
    summary = judge(results, chosen['worlds'], chosen['seeds'])
    summary['provenance'] = provenance
    summary['set'] = args.set
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (out / 'summary.md').write_text(markdown(summary, f'Active SLAM evaluation: {args.set}'))
    print((out / 'summary.md').read_text())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
