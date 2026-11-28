"""Refuse an update whose height disagrees with the store before it joins the map.

A session is placed in the store by one rigid transform from its anchor, so vertical drift of
its odometry reaches the map: a lifelong NCLT session climbed 5 m above the earlier sessions
in its last 100 s (docs/roadmap/long-term-mapping-2026-11.md, "Vertical drift in the lifelong
store"). Before such a session is committed, its submap positions are compared with those of
the stored sessions where the paths overlap.

Most of that drift is a tilt of the session's anchor: registered over its first few keyframes,
roll and pitch are weakly constrained, and 0.25-1.75 degrees amount to metres at the far end.
:func:`correct` fits one rotation (roll, pitch about the first submap) and height offset to the
overlap of the whole session; on NCLT sessions the fit had not seen it brought the updates above
1.0 m (95th percentile of |dz|) from three of five to one ("Check of the tilt and height
correction on sessions the chain has not used").

A rigid correction cannot remove drift that grows inside the session (2012-03-31 climbs 5 m in
its last 100 s). :func:`profile` then lowers each submap by a height offset that is piecewise
linear in submap order, with knots every PROFILE_SPACING submaps, fitted robustly to the height
differences of the compared submaps. The corrected poses are written
as ``store_submap_poses.npy``, which take precedence over the rigid transform (lidarslam.store).
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
TILT_STEP_DEG = 0.25
MAX_TILT_DEG = 4.0
MIN_KEPT = 0.8  # a tilt must keep this share of the compared submaps
PROFILE_SPACING = 30  # submaps between the knots of a height profile
PROFILE_MIN = 10  # compared submaps a knot needs
PROFILE_HUBER_M = 0.3  # residuals beyond this get Huber weights in the profile fit


def submap_poses(directories, transforms) -> list[np.ndarray]:
    """The store-frame submap poses of each session (N x 4 x 4), as export places them."""
    directories = [str(directory) for directory in directories]
    poses = [tracked_poses(directory) for directory in directories]
    matcher = _core.StoreRelocalizer(directories, [np.asarray(t, dtype=float) for t in transforms],
                                     [{}] * len(directories), poses)
    return [np.array([np.asarray(pose) for pose in session]).reshape(-1, 4, 4)
            for session in matcher.session_poses()]


def submap_positions(directories, transforms) -> list[np.ndarray]:
    """The store-frame submap positions of each session (N x 3), as export places them."""
    return [poses[:, :3, 3] for poses in submap_poses(directories, transforms)]


class HeightReference:
    """Reference submap positions indexed once in an XY grid of ``radius`` cells."""

    def __init__(self, reference: np.ndarray, radius: float = RADIUS_M,
                 min_neighbours: int = MIN_NEIGHBOURS):
        self.reference = np.asarray(reference, dtype=float).reshape(-1, 3)
        self.radius = radius
        self.min_neighbours = min_neighbours
        self.cells = defaultdict(list)
        for index, (x, y) in enumerate(np.floor(self.reference[:, :2] / radius).astype(np.int64)):
            self.cells[(x, y)].append(index)
        self.cells = {key: np.asarray(value) for key, value in self.cells.items()}

    def differences(self, update: np.ndarray, with_index: bool = False):
        """For each update position with at least ``min_neighbours`` reference positions within
        ``radius`` (XY): its height minus their median height (and, ``with_index``, the indices
        of those positions)."""
        radius, reference = self.radius, self.reference
        differences, indices = [], []
        for i, position in enumerate(np.asarray(update, dtype=float).reshape(-1, 3)):
            x, y = np.floor(position[:2] / radius).astype(np.int64)
            parts = [self.cells[key] for key in ((x + dx, y + dy) for dx in (-1, 0, 1)
                                                  for dy in (-1, 0, 1)) if key in self.cells]
            if not parts:
                continue
            near = np.concatenate(parts)
            offsets = reference[near, :2] - position[:2]
            near = near[np.sum(offsets ** 2, axis=1) <= radius * radius]
            if len(near) >= self.min_neighbours:
                differences.append(position[2] - float(np.median(reference[near, 2])))
                indices.append(i)
        differences = np.asarray(differences, dtype=float)
        return (differences, np.asarray(indices, dtype=np.int64)) if with_index else differences


def height_differences(reference: np.ndarray, update: np.ndarray, radius: float = RADIUS_M,
                       min_neighbours: int = MIN_NEIGHBOURS) -> np.ndarray:
    """For each update position with at least ``min_neighbours`` reference positions within
    ``radius`` (XY): its height minus their median height."""
    return HeightReference(reference, radius, min_neighbours).differences(update)


def tilt_matrix(roll: float, pitch: float, offset: float, about) -> np.ndarray:
    """The store-frame correction (4 x 4): rotate by ``roll`` and ``pitch`` (radians, one
    rotation vector about x and y) about ``about``, then lower by ``offset`` metres."""
    from scipy.spatial.transform import Rotation
    about = np.asarray(about, dtype=float)
    correction = np.eye(4)
    correction[:3, :3] = Rotation.from_rotvec([roll, pitch, 0.0]).as_matrix()
    correction[:3, 3] = about - correction[:3, :3] @ about - [0.0, 0.0, offset]
    return correction


def fit_tilt(reference: np.ndarray, update: np.ndarray) -> dict[str, Any] | None:
    """The roll and pitch (TILT_STEP_DEG grid within MAX_TILT_DEG, about the first update
    position, keeping MIN_KEPT of the compared positions) and height offset (the median dz after
    the tilt) that minimise the 95th percentile of |dz - median dz|. None with fewer than
    MIN_COMPARED compared positions."""
    update = np.asarray(update, dtype=float).reshape(-1, 3)
    index = HeightReference(reference)
    before = index.differences(update)
    if len(before) < MIN_COMPARED:
        return None
    grid = np.radians(np.arange(-MAX_TILT_DEG, MAX_TILT_DEG + 1e-9, TILT_STEP_DEG))
    best = None
    for roll in grid:
        for pitch in grid:
            tilt = tilt_matrix(roll, pitch, 0.0, update[0])
            d = index.differences(update @ tilt[:3, :3].T + tilt[:3, 3])
            if len(d) < MIN_KEPT * len(before):
                continue
            median = float(np.median(d))
            score = float(np.percentile(np.abs(d - median), 95))
            if best is None or score < best[0]:
                best = (score, float(roll), float(pitch), median)
    _, roll, pitch, offset = best
    correction = tilt_matrix(roll, pitch, offset, update[0])
    after = index.differences(update @ correction[:3, :3].T + correction[:3, 3])
    return {'roll_deg': round(float(np.degrees(roll)), 2),
            'pitch_deg': round(float(np.degrees(pitch)), 2), 'offset_m': round(offset, 3),
            'compared_submaps': int(len(before)),
            'p95_abs_dz_before_m': round(float(np.percentile(np.abs(before), 95)), 3),
            'p95_abs_dz_after_m': round(float(np.percentile(np.abs(after), 95)), 3),
            'correction': correction}


def correct(aligned, directory: str | Path, transform) -> dict[str, Any]:
    """Fits :func:`fit_tilt` for the written session in ``directory`` (store transform
    ``transform``) against the aligned stored sessions. ``store_from_session`` is the corrected
    transform (``transform`` when too few submaps overlap: ``corrected`` is then False)."""
    directories = [path for path, _ in aligned] + [str(directory)]
    positions = submap_positions(directories, [t for _, t in aligned] + [transform])
    fit = fit_tilt(np.concatenate(positions[:-1]), positions[-1])
    transform = np.asarray(transform, dtype=float)
    if fit is None:
        return {'corrected': False, 'store_from_session': transform.tolist()}
    correction = fit.pop('correction')
    return {'corrected': True, **fit, 'store_from_session': (correction @ transform).tolist()}


def fit_profile(reference: np.ndarray, update: np.ndarray) -> dict[str, Any] | None:
    """The height offset of each update position (see the module docstring): piecewise linear
    between knots (every PROFILE_SPACING positions, within the compared stretch) with at least
    PROFILE_MIN compared positions within PROFILE_SPACING, constant beyond the outer ones,
    fitted to the compared height differences by least squares with Huber weights
    (PROFILE_HUBER_M). None with fewer than MIN_COMPARED compared positions or no knot."""
    update = np.asarray(update, dtype=float).reshape(-1, 3)
    index = HeightReference(reference)
    before, compared = index.differences(update, with_index=True)
    if len(before) < MIN_COMPARED:
        return None
    # Knots stay within the compared stretch, so the offset is held, not extrapolated, beyond it.
    first, last = int(compared[0]), int(compared[-1])
    knots = sorted({int(np.clip(knot, first, last))
                    for knot in range(0, len(update) + PROFILE_SPACING - 1, PROFILE_SPACING)})
    knots = [knot for knot in knots
             if np.count_nonzero(np.abs(compared - knot) <= PROFILE_SPACING) >= PROFILE_MIN]
    if not knots:
        return None

    def basis(positions):
        return np.stack([np.interp(positions, knots, np.eye(len(knots))[k])
                         for k in range(len(knots))], axis=1)

    design = basis(compared)
    weights = np.ones(len(before))
    for _ in range(10):
        root = np.sqrt(weights)[:, None]
        values = np.linalg.lstsq(design * root, before * root[:, 0], rcond=None)[0]
        residual = np.abs(design @ values - before)
        weights = np.minimum(1.0, PROFILE_HUBER_M / np.maximum(residual, 1e-9))
    offsets = basis(np.arange(len(update))) @ values
    lowered = update - np.outer(offsets, [0.0, 0.0, 1.0])
    after = index.differences(lowered)
    return {'knots': len(knots), 'compared_submaps': int(len(before)),
            'max_abs_offset_m': round(float(np.max(np.abs(offsets))), 3),
            'p95_abs_dz_before_m': round(float(np.percentile(np.abs(before), 95)), 3),
            'p95_abs_dz_after_m': round(float(np.percentile(np.abs(after), 95)), 3),
            'offsets': offsets}


def profile(aligned, directory: str | Path, transform) -> dict[str, Any]:
    """Fits :func:`fit_profile` for the written session in ``directory`` (store transform
    ``transform``) against the aligned stored sessions and writes the lowered submap poses to
    ``store_submap_poses.npy`` there. ``corrected`` is False (nothing written) when the fit
    returns None."""
    directories = [path for path, _ in aligned] + [str(directory)]
    poses = submap_poses(directories, [t for _, t in aligned] + [transform])
    reference = np.concatenate([p[:, :3, 3] for p in poses[:-1]])
    fit = fit_profile(reference, poses[-1][:, :3, 3])
    if fit is None:
        return {'corrected': False}
    lowered = poses[-1].copy()
    lowered[:, 2, 3] -= fit.pop('offsets')
    np.save(Path(directory) / 'store_submap_poses.npy', lowered, allow_pickle=False)
    return {'corrected': True, **fit}


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
