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

"""Tests for the pose-uncertainty timeline GIF renderer (tools/readme_media)."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'tools' / 'readme_media' / 'render_pose_uncertainty.py'
PIL = pytest.importorskip('PIL')
SPEC = importlib.util.spec_from_file_location('render_pose_uncertainty', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

HEADER = ('step,stamp_sec,submaps,loop_edges,id,fixed,x,y,z,sigma_translation_max,'
          'sigma_translation_min,weak_direction_x,weak_direction_y,weak_direction_z,'
          'sigma_rotation_max_deg,log_det_covariance\n')


def _write_timeline(directory: Path) -> Path:
    """
    Write four graph states of a square walk.

    A loop edge 0-3 appears at step 2, and step 3 replaces it with 1-4 (a new loop edge
    although the count stays 1).
    """
    rows = []
    positions = [(0, 0), (2, 0), (2, 2), (0.2, 0.1), (2.1, 0.1)]
    sigmas = {0: [0.0, 0.1], 1: [0.0, 0.1, 0.2], 2: [0.0, 0.05, 0.06, 0.02],
              3: [0.0, 0.05, 0.06, 0.05, 0.07]}
    loops = {0: 0, 1: 0, 2: 1, 3: 1}
    for step, values in sigmas.items():
        for vertex, sigma in enumerate(values):
            x, y = positions[vertex]
            rows.append(f'{step},{100.0 + step:.9f},{len(values)},{loops[step]},{vertex},'
                        f'{1 if vertex == 0 else 0},{x},{y},0,{sigma},{sigma / 2},0,1,0,1.0,0\n')
    timeline = directory / 'pose_uncertainty_timeline.csv'
    timeline.write_text(HEADER + ''.join(rows), encoding='utf-8')
    (directory / 'pose_uncertainty_timeline_loops.csv').write_text(
        'step,from,to\n2,0,3\n3,1,4\n', encoding='utf-8')
    return timeline


def test_load_timeline_and_loop_closures(tmp_path: Path) -> None:
    states = MODULE.load_timeline(_write_timeline(tmp_path))
    assert [s.step for s in states] == [0, 1, 2, 3]
    assert [len(s.sigma) for s in states] == [2, 3, 4, 5]
    assert states[2].loops == [(0, 3)] and states[3].loops == [(1, 4)]
    assert states[3].newest_sigma == pytest.approx(0.07)
    # A replaced edge is a new loop closure even when the count does not rise.
    assert MODULE.loop_closure_steps(states) == [2, 3]
    durations = MODULE.frame_durations(states, 100, 900)
    assert durations == [100, 100, 900, 1800]


def test_loop_closures_from_counts_without_the_loop_file(tmp_path: Path) -> None:
    timeline = _write_timeline(tmp_path)
    (tmp_path / 'pose_uncertainty_timeline_loops.csv').unlink()
    states = MODULE.load_timeline(timeline)
    assert MODULE.loop_closure_steps(states) == [2]


def test_ramp_is_monotonic_light_to_dark() -> None:
    lightness = [sum(MODULE.ramp_color(v, 1.0)) for v in (0.0, 0.25, 0.5, 0.75, 1.0)]
    assert lightness == sorted(lightness, reverse=True)
    assert MODULE.ramp_color(-1.0, 1.0) == MODULE.RAMP[0]
    assert MODULE.ramp_color(5.0, 1.0) == MODULE.RAMP[-1]
    assert MODULE.ramp_color(0.3, 0.0) == MODULE.RAMP[0]


def test_ascii_pcd_is_read_and_filtered(tmp_path: Path) -> None:
    pcd = tmp_path / 'map.pcd'
    pcd.write_text(
        'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n1 2 0.5\n3 4 5\n5 6 -0.2\n',
        encoding='ascii')
    xy = MODULE.load_pcd_xy(pcd, -1.0, 1.0, 1000)
    assert xy.tolist() == [[1.0, 2.0], [5.0, 6.0]]


def test_gif_is_written_and_reproducible(tmp_path: Path) -> None:
    timeline = _write_timeline(tmp_path)
    digests = []
    for name in ('a.gif', 'b.gif'):
        out = tmp_path / name
        subprocess.run([sys.executable, str(SCRIPT), '--timeline', str(timeline), '--out',
                        str(out)], check=True, capture_output=True)
        digests.append(hashlib.sha256(out.read_bytes()).hexdigest())
    assert digests[0] == digests[1]
    with PIL.Image.open(tmp_path / 'a.gif') as image:
        assert image.n_frames == 4
        assert image.size == (960, 540)
