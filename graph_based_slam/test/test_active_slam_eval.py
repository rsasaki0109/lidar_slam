# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the closed-loop active SLAM evaluation tools (tools/rne_active_slam_env)."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'tools' / 'rne_active_slam_env'
np = pytest.importorskip('numpy')
pytest.importorskip('yaml')
sys.path.insert(0, str(TOOLS))


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


worlds = _load('worlds')
harness = _load('run_closed_loop')
evaluation = _load('run_eval')


def test_final_map_ate_removes_only_a_rigid_offset(tmp_path: Path) -> None:
    stamps = [float(k) for k in range(11)]
    truth = [(1.0 * k, 0.5 * math.sin(k)) for k in range(11)]
    yaw, shift = 0.3, (2.0, -1.0)
    rows = []
    for t, (x, y) in zip(stamps, truth):
        ex = math.cos(yaw) * x - math.sin(yaw) * y + shift[0]
        ey = math.sin(yaw) * x + math.cos(yaw) * y + shift[1]
        rows.append(f'{t} {ex} {ey} 0 0 0 0 1')
    tum = tmp_path / 'traj.tum'
    tum.write_text('\n'.join(rows) + '\n', encoding='utf-8')
    ate = harness.final_map_ate(tum, stamps, truth)
    assert ate['poses'] == 11
    assert ate['rmse_m'] == pytest.approx(0.0, abs=1e-9)
    assert ate['raw_rmse_m'] > 1.0
    # A real error is kept after the alignment.
    rows[5] = '5.0 100 100 0 0 0 0 1'
    tum.write_text('\n'.join(rows) + '\n', encoding='utf-8')
    assert harness.final_map_ate(tum, stamps, truth)['rmse_m'] > 1.0


def _run(world, seed, policy, ate, coverage, failed=False):
    return {'world': world, 'seed': seed, 'policy': policy, 'final_map_ate_rmse_m': ate,
            'coverage': coverage, 'failed': failed, 'distance_m': 100.0}


def test_success_rule_needs_reduction_wins_coverage_and_failures() -> None:
    seeds = list(range(101, 111))
    results = []
    for s in seeds:
        results.append(_run('w', s, 'frontier_info', 1.0, 0.9))
        # 8 of 10 seeds 30 % lower, 2 seeds 10 % higher; coverage 2 points lower.
        results.append(_run('w', s, 'active', 0.7 if s < 109 else 1.1, 0.88))
    summary = evaluation.judge(results, ['w'], seeds)
    rule = summary['worlds']['w']['active_vs_frontier_info']
    assert rule['active_lower_ate_seeds'] == 8
    assert rule['median_ate_reduction'] == pytest.approx(0.3)
    assert rule['median_coverage_difference_points'] == pytest.approx(-2.0)
    assert rule['pass'] and summary['pass']

    worse_coverage = [dict(r, coverage=0.8) if r['policy'] == 'active' else r for r in results]
    assert not evaluation.judge(worse_coverage, ['w'], seeds)['pass']
    few_wins = [dict(r, final_map_ate_rmse_m=1.1) if r['policy'] == 'active' and r['seed'] > 104
                else r for r in results]
    assert not evaluation.judge(few_wins, ['w'], seeds)['pass']
    more_failures = [dict(r, failed=True) if r['policy'] == 'active' and r['seed'] == 101 else r
                     for r in results]
    assert not evaluation.judge(more_failures, ['w'], seeds)['pass']


def test_worlds_have_ten_free_starts_and_a_free_region() -> None:
    for name, build in worlds.WORLDS.items():
        world = build()
        assert len(world['starts']) == 10, name
        obstacles = [b for b in world['boxes'] if b['max'][2] > 0.05]
        for x, y, _ in world['starts']:
            for b in obstacles:
                cx = min(max(x, b['min'][0]), b['max'][0])
                cy = min(max(y, b['min'][1]), b['max'][1])
                assert math.hypot(x - cx, y - cy) >= 0.6, (name, x, y)
        coverage = harness.Coverage(world['free_region'])
        assert coverage.free.sum() > 1000, name
    assert set(worlds.HOLDOUT).isdisjoint(worlds.DEVELOPMENT)
    w = worlds.world_for('branches', 103)
    assert w['start'] == worlds.world_branches()['starts'][3] and w['seed'] == 103
