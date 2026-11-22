"""Search pacing of ``live --relocalize-anywhere`` (StartupRelocalization), with a fake matcher.

A search that registers candidates and finds nothing doubles the wait before the next search, in
data time, up to ANYWHERE_MAX_INTERVAL_SEC; a search with no candidate under the Scan Context gate
leaves the wait as it is, and a found candidate resets it.
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip('lidarslam._core')
from lidarslam import relocalization  # noqa: E402
from lidarslam.relocalization import StartupRelocalization  # noqa: E402


class FakeMatcher:
    """match_anywhere returns ``found`` (or None); rank_anywhere reports ``best`` as the
    closest Scan Context distance."""

    def __init__(self):
        self.searches: list[float] = []
        self.best = 0.1
        self.found = None
        self.stamp = 0.0

    def match_anywhere(self, points):
        self.searches.append(self.stamp)
        return self.found

    def rank_anywhere(self, points, count):
        return [(self.best, 0, 0)]


def searcher() -> tuple[StartupRelocalization, FakeMatcher]:
    search = StartupRelocalization.__new__(StartupRelocalization)
    matcher = FakeMatcher()
    search.__dict__.update(
        prior=None, anywhere=True, _candidate=None, _last_search=-np.inf,
        _search_interval=relocalization.ANYWHERE_INTERVAL_SEC, matcher=matcher,
        _surfaces=None, first_stamp=0.0, expired=False, transform=None, anchor_stamp=None,
        track=False, _odometry=[], _observations=[], _initial_odometry=None)
    return search, matcher


def feed(search: StartupRelocalization, matcher: FakeMatcher, start: float, end: float) -> None:
    eye = np.eye(4)
    for stamp in np.arange(start, end, 0.5):
        matcher.stamp = float(stamp)
        search.observe(float(stamp), 0, eye, eye, eye, None, np.zeros((10, 3), np.float32))


def test_failed_registrations_back_off_to_the_maximum() -> None:
    search, matcher = searcher()
    feed(search, matcher, 0.0, 200.0)
    gaps = np.diff(matcher.searches)
    assert list(gaps[:2]) == [4.0, 8.0]
    assert set(gaps[2:]) == {relocalization.ANYWHERE_MAX_INTERVAL_SEC}


def test_searches_without_a_candidate_keep_the_interval() -> None:
    search, matcher = searcher()
    matcher.best = relocalization.ANYWHERE_DESCRIPTOR_GATE + 0.01
    feed(search, matcher, 0.0, 20.0)
    assert set(np.diff(matcher.searches)) == {relocalization.ANYWHERE_INTERVAL_SEC}


def test_a_found_candidate_resets_the_interval() -> None:
    search, matcher = searcher()
    feed(search, matcher, 0.0, 40.0)
    assert search._search_interval == relocalization.ANYWHERE_MAX_INTERVAL_SEC
    matcher.found = (0, np.eye(4))
    feed(search, matcher, 40.0, 100.0)
    # The first candidate is kept and resets the wait; the next search confirms it and anchors.
    assert search.transform is not None
    assert matcher.searches[-1] - matcher.searches[-2] == relocalization.ANYWHERE_INTERVAL_SEC
