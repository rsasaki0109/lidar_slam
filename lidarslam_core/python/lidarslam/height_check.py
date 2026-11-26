"""Refuse an update whose height disagrees with the store before it joins the map.

A session is placed in the store by one rigid transform from its anchor, so vertical drift of
its odometry reaches the map: a lifelong NCLT session climbed 5 m above the earlier sessions
in its last 100 s (docs/roadmap/long-term-mapping-2026-11.md, "Vertical drift in the lifelong
store"). Before such a session is committed, its submap positions are compared with those of
the stored sessions where the paths overlap.
"""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from lidarslam import _core
from lidarslam.relocalization import tracked_poses

RADIUS_M = 1.5
MIN_NEIGHBOURS = 3
MIN_COMPARED = 20


def submap_positions(directories, transforms) -> list[np.ndarray]:
    """The store-frame submap positions of each session (N x 3), as export places them."""
    directories = [str(directory) for directory in directories]
    poses = [tracked_poses(directory) for directory in directories]
    matcher = _core.StoreRelocalizer(directories, [np.asarray(t, dtype=float) for t in transforms],
                                     [{}] * len(directories), poses)
    return [np.array([np.asarray(pose)[:3, 3] for pose in session]).reshape(-1, 3)
            for session in matcher.session_poses()]


def height_differences(reference: np.ndarray, update: np.ndarray, radius: float = RADIUS_M,
                       min_neighbours: int = MIN_NEIGHBOURS) -> np.ndarray:
    """For each update position with at least ``min_neighbours`` reference positions within
    ``radius`` (XY): its height minus their median height."""
    cells = defaultdict(list)
    for index, (x, y) in enumerate(np.floor(reference[:, :2] / radius).astype(np.int64)):
        cells[(x, y)].append(index)
    differences = []
    for position in update:
        x, y = np.floor(position[:2] / radius).astype(np.int64)
        candidates = [i for dx in (-1, 0, 1) for dy in (-1, 0, 1) for i in cells.get((x + dx, y + dy), ())]
        if not candidates:
            continue
        near = np.asarray(candidates)
        near = near[np.sum((reference[near, :2] - position[:2]) ** 2, axis=1) <= radius * radius]
        if len(near) >= min_neighbours:
            differences.append(position[2] - float(np.median(reference[near, 2])))
    return np.asarray(differences, dtype=float)


def check(aligned, directory: str | Path, transform, limit: float) -> dict[str, Any]:
    """Compares the written session in ``directory`` (store transform ``transform``) with the
    aligned stored sessions. ``refused`` when at least MIN_COMPARED submaps overlap and the 95th
    percentile of their |height difference| exceeds ``limit`` metres."""
    directories = [path for path, _ in aligned] + [str(directory)]
    transforms = [t for _, t in aligned] + [transform]
    positions = submap_positions(directories, transforms)
    differences = height_differences(np.concatenate(positions[:-1]), positions[-1])
    result: dict[str, Any] = {'compared_submaps': int(len(differences)), 'limit_m': limit,
                              'refused': False}
    if len(differences):
        result['median_dz_m'] = round(float(np.median(differences)), 3)
        result['p95_abs_dz_m'] = round(float(np.percentile(np.abs(differences), 95)), 3)
        result['refused'] = (len(differences) >= MIN_COMPARED and
                             result['p95_abs_dz_m'] > limit)
    return result
