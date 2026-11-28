# Keep One Map Over Time

A long-term map store keeps the mapping sessions of one place. Each new session finds itself in
the stored map without an initial pose, is added to it, and can remove what has gone. The store
exports one Autoware map bundle (`pointcloud_map/`, `map_projector_info.yaml`) from all its
sessions. It runs without ROS 2 (`pip install lidarslam`).

The store is experimental. The results below are from NCLT (one campus, 20 sessions over
fourteen months); the [long-term mapping plan](roadmap/long-term-mapping-2026-11.md) has every
check and its preregistered rule.

## Start a store

The first session creates the store:

```bash
python -m lidarslam live first.bag --out run1 --store STORE \
  --odometry-params ODOMETRY.yaml --backend-params BACKEND.yaml
```

`live` plays the log as a live session and adds the session to `STORE` when it finishes.

## Add a session

```bash
python -m lidarslam live later.bag --out run2 --store STORE \
  --odometry-params ODOMETRY.yaml --backend-params BACKEND.yaml \
  --relocalize-anywhere --detect-changes
```

- `--relocalize-anywhere` searches the whole store for where the session is, for as long as it
  runs, so the session can start anywhere. `--start-near-first-submap` (search within 5 m of the
  first stored submap) and `--initial-pose` are the alternatives when the start is known.
- `--detect-changes` traces the session's scans through the stored map and removes old points
  it sees through. `--refine-visibility` also holds back new points seen only briefly and is the
  recommended change mode ([Review Map Updates](map-review.md)).

A session that never anchors is still written, in its own frame, but the export of a store needs
one common frame.

## Heights are fitted to the store

Odometry drifts in height, and an anchor transform fitted at the start can leave a session tilted
or offset by a metre or more further on. Before an anchored session is committed, `live`
compares its submap heights with the stored sessions where their paths overlap (within 1.5 m)
and, by default:

1. fits a roll, pitch and height offset to the session (`--no-tilt-correction` turns it off);
2. fits a height offset along the session, every 30 submaps (`--no-height-profile` turns it off);
3. refuses the session if the remaining height difference is above 1.5 m at the 95th
   percentile (`--max-height-drift M`; `0` turns the check off). A refused session stays
   uncommitted.

Each step needs at least 20 compared submaps, so a session that overlaps the store only briefly
is added at its anchor transform, unchecked. The change evidence is traced where the session is
finally placed, after these corrections.

On NCLT, the corrections brought the height consistency of the 20-session store to 0.11 m (the
median over sessions of the 95th percentile of the height difference), and on sessions the chain
had not used they cut the median from 0.21 m (tilt only) to 0.11 m. Tracing the change evidence
after the corrections cut the removed voxels that a later session still sees from 22% to 17%.

## Inspect, export, undo

```bash
python -m lidarslam store info STORE         # the committed sessions
python -m lidarslam store export STORE       # the Autoware bundle in STORE/export
python -m lidarslam store rollback STORE     # take the latest session off the map
```

Sessions are append-only; a rollback leaves the session's files in place. What a session changed
can be reviewed and edited, by a person or an agent, before it is used:
[Review Map Updates](map-review.md).

## Limits

- **The change evidence makes mistakes.** On NCLT, 17-19% of the removals a later session
  decided were kept by it. Most are removals decided closest to the rule, and some are real
  changes: an older session still sees what a newer one removed.
- **A brief overlap is neither corrected nor checked.** Below 20 compared submaps the height
  fits and the height check do not run. NCLT 2012-04-29 (14 submaps) needed no correction, but
  a session that drifts in a brief overlap goes in as it is.
- **Map-quality metrics do not show the corrections.** The corrected store's entropy is slightly
  lower and more of it is planar, but its plane patches are thicker on average, so the
  preregistered comparison was mixed.
- **Relocalization costs search time.** `--relocalize-anywhere` in real time searches fewer
  keyframes, and a session that is in the store only briefly may not anchor.
