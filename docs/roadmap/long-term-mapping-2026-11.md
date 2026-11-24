# Long-term mapping: update a stored map while driving (2026-11)

Goal: a robot that drives through a place it has mapped before updates the stored map as it
goes. Where the place changed (furniture moved, a partition taken down, a wall added), the map
shows the place as it is now: new structure appears, and structure the robot sees is gone is
removed automatically. Nothing the robot did not see changes. The old map is kept, so an update
can be undone.

Decisions (maintainer, 2026-11-18): the update runs live, while the robot drives (not as a step
after the run); stale points are removed automatically (no human review step); the method is
developed and evaluated on public data only (this is open-source work with no robot of its own).

## Where the project is today

| Part | What exists | What is missing for this goal |
| --- | --- | --- |
| Map of one run | `lidarslam-map run/start` (ROS 2 or `--engine core`): an Autoware bundle (`pointcloud_map/`, metadata, trajectories) plus the recorded backend input | the bundle is the end product; nothing loads it back as a map to extend |
| Several runs | `lidarslam-map merge [--place-recognition]` and `start --join-map` (`scripts/map_merge.py`, `scripts/map_join.py`) | runs after the fact, never live. Each run's `map.pcd` is stacked with one rigid x, y, z, yaw transform, and in a voxel shared by two runs the **first** run's point is kept (`_deduplicate_records`, `scripts/map_merge.py`): a moved desk stays where it was, and new things only fill empty space |
| Place recognition between runs | `graph_slam_multisession_join` / `python -m lidarslam join` (`multisession_join.hpp`, `docs/architecture/multisession-join.md`): Scan Context candidates, small_gicp, max-clique, refusals (ambiguous sets, self-similar sessions, whole-map check), a joined pose graph | offline only, over finished runs. The re-optimized joined trajectories are not used for the merged points |
| Removing stale points | none. The 2026-11-08 trial (`bench_report/data/map_update_2026-11-08/plan.txt`, rays of a later run clear an earlier run's points) failed its preregistered rule | its test counted removed people as errors, and it cast rays from one frame in five |
| Live mapping | the ROS 2 nodes (`rko_lio`, `graph_based_slam`) | no ROS-free streaming API (task: ROS-free live mapping); no way to start from a stored map |
| Engine boundary | `GraphSlamApplication`: "the ordered, ROS-free workflow boundary shared by live and replay adapters" | a live adapter outside ROS 2 |

The product scripts reach each other through subprocesses (`lidarslam-map` dispatches to a dozen
scripts; `start` is `sensor_setup_wizard.py`, 3,935 lines, and also drives the join), and every
multi-session step exists twice (ROS 2 executables and `python -m lidarslam`). This design adds
the long-term mode to the package and the C++ core, not to that layer, and moves the join out of
it (see "Architecture cleanup").

## Design

### 1. The map store is the source of truth

A long-term map is a directory, `store/`, not an Autoware bundle. The bundle becomes an export.

```
store/
  store.json          format version, frame, the list of sessions (id, time, sensor setup, state)
  sessions/<id>/      keyframes: pose in the store frame, compressed deskewed scan (the backend's
                      cloud_compression format), Scan Context descriptor; the session's
                      self-similarity record (the join's fingerprint cache)
  graph/              the pose graph over all sessions: per-session odometry/loop edges and the
                      inter-session edges, with each edge's source
  evidence/           per-submap voxel evidence (below)
  export/             the Autoware bundle written from the store after each session
```

Sessions are append-only. A session is written to `sessions/<id>/` while it runs and becomes part
of the map only when `store.json` lists it as committed (one atomic rename). Rolling back an
update removes that entry; the older sessions' data was never changed.

### 2. A live session

1. **Load** the store: keyframes and submaps of the committed sessions (compressed in memory, as
   the backend keeps submaps today), their descriptors, the graph.
2. **Relocalize.** The first keyframes of the new session are matched against the stored
   keyframes with the join's place recognition and its refusals. Until a match is accepted the
   session maps in its own frame and its keyframes wait; when it is accepted, the session is
   anchored and the waiting keyframes are processed with the anchor. A start pose given by the
   caller (as Autoware's initial pose) narrows the search but is checked the same way. If no
   match is accepted the session is mapped and stored as a new, separate map island; it is never
   forced onto the old map.
3. **Track and close loops.** RKO-LIO odometry as today. The backend (`GraphSlamApplication`)
   holds the stored keyframes as nodes with their stored poses as priors and searches loops
   between new and stored submaps with the existing loop search. The graph is optimized
   incrementally; stored poses move only as far as their priors allow.
4. **Update the map (change detection).** For each new keyframe, the scan's rays are traced (3D
   DDA, C++) through the stored submaps near it. Evidence is kept **per submap, in the submap's
   own frame** (as SEAM, arXiv:2609.18819, anchors it), so a later pose correction moves a
   submap with its evidence and never invalidates it. Per voxel:
   - *hits* and *misses* of the current session, counted per frame;
   - the stored points' state: present, or removed by session `<id>` (a tombstone, so it can be
     undone).

   Rules:
   - **Removed (stale):** a stored voxel that rays of the current session crossed, short of their
     end point by a margin, in at least `k` frames from at least two viewpoints, with no hit of
     the session in it or its neighbours. Rays at grazing angles to a stored surface do not count.
   - **Added:** a voxel the current session hit in at least `m` frames.
   - **Not static (people, carts):** a voxel the current session both hit and saw through in
     different frames. It is left out of the map whichever session it came from, so people in the
     old map are removed as well.
   - **Not seen:** no change.

   The decision for a voxel is the latest session's: "the last session that looked wins".
5. **Publish.** After each keyframe the map change (voxels added and removed, in the store
   frame) is available to the caller: a Python callback, and later a ROS 2 topic.
6. **Commit.** At the end the session, its graph edges and its evidence are written and listed in
   `store.json`; the Autoware bundle is exported from the store.

### 3. Engine and API

- C++ (ROS-free, in `graph_based_slam`): the store, the live long-term adapter around
  `GraphSlamApplication`, the evidence map and ray tracing.
- Python (`lidarslam` package): `lidarslam.live.Session(store, params)` with `add_imu`,
  `add_scan`, `changes()`, `commit()`; `lidarslam-map live <store> <log> [--realtime]` replays a
  log at sensor rate through it (this is also the ROS-free live mapping task, simulated by
  real-time playback). A ROS 2 node over the same C++ adapter comes after the evaluation.
- The offline path (`run`, `start --engine core`) stays as it is and keeps its bytes.

### Architecture cleanup (in scope)

- Multi-session code moves into the package: one Python implementation of joining and merging
  over the C++ core, used by `merge`, `start --join-map` and the store. The ROS 2 executables stay
  as adapters; the duplicate subprocess paths in `map_join.py` go.
- `merge` is rebuilt on the store: `merge A B` = a store with session A, updated offline with
  session B (the same code as a live update, fed from a finished run). Its "first session wins"
  stacking goes.
- `sensor_setup_wizard.py` stops driving the join (`--join-map` becomes "update this store").
- Each step keeps the single-run outputs byte-identical (checked as in the ROS-free milestones).

## Evaluation (preregistered; fixed before any run of the method)

### Data

**LUH long-term indoor localization dataset**
([data.uni-hannover.de](https://data.uni-hannover.de/dataset/long-term-indoor-localization-in-dynamic-environment),
CC BY 4.0; paper doi:10.5194/isprs-archives-XLVIII-1-W6-2025-155-2025). A 40 x 10 m entrance hall,
recorded from a push cart (Ouster OS1, its IMU on `/ouster/imu`, Hesai Pandar64; ROS 1 bags) in
three layouts:

| Layout | Sequences | Contents |
| --- | --- | --- |
| A, extreme occluded | S0 (5.3 min), S1 (2.7 min) | eight poster walls forming a corridor |
| B, semi occluded | S2 (1.4), S3 (2.0), S4 (2.6) | L-shaped poster walls, tables, chairs |
| C, free space | S5 (2.2), S6 (2.0), S7 (2.0) | the empty hall, 10-15 pedestrians |

Reference: total-station positions of every sequence (prism tracked, offset to the LiDAR solved,
in the TLS frame) and a terrestrial laser scan of the **empty** hall (no poster walls, furniture
or people). Only the Ouster and its IMU are used (one sensor, as the product runs). The bags are
stream-converted to ROS 2 keeping those two topics, and handled as untrusted downloads.

### Split

Development uses S1, S3 and S6 only (their trajectories and the TLS scan may be looked at while
developing). Holdout: S0, S2, S4, S5, S7; their total-station trajectories stay unopened until the
method and its parameters are frozen and recorded in this file.

Update chains (base session first, each later session a live update of the store):

- development: S1 → S3 → S6, and S6 → S3 → S1
- holdout: S0 → S4 → S7, S7 → S4 → S0, S2 → S5, S5 → S2

### Measures

The store frame is aligned to the TLS frame once per chain, by the base session's estimated
positions against its total-station positions (Umeyama, rigid). Points are compared inside the
hall's floor area, between 0.1 m above the floor and 2.5 m (the poster walls' height band), and
only where the update session's own scans reach (within 10 m of its path).

- **G, ghosts after an update to layout C.** The share of map points farther than 0.10 m from
  the TLS scan. Everything that is not the empty hall is a ghost here: poster walls, furniture
  and people.
- **S, static structure kept.** The share of TLS points (same region) that have a map point within
  0.10 m, after the update, divided by the same share before it.
- **N, new structure after an update from layout C to A or B.** Reference: the map of the other
  sequence of the same layout, mapped alone, keeping its points farther than 0.20 m from the TLS
  scan (the poster walls and furniture). N is the share of those reference points with a map
  point within 0.10 m after the update. Uses S0/S1 and S2/S3/S4 as each other's reference; in
  development only S1 ↔ S3 cross references are used (no holdout map is built before the holdout
  run).
- **R, relocalization.** The update session anchors to the store within the first 10 s of data,
  with no wrong anchor (a wrong anchor: the anchored pose more than 0.5 m or 5 degrees from the
  total-station position and the TLS-registered heading).
- **L, localization.** Position RMSE of the update session's live (causal) poses against the
  total station, compared with the same sequence mapped alone.
- **T, run time.** Live mode at sensor rate on the development machine: no input dropped,
  processing time per scan below the scan period on average; peak memory reported.

Baseline for G, S and N: today's `lidarslam-map merge --place-recognition` on the same sessions.

### Decision rule (holdout)

The long-term mode becomes the product's update path (and `merge` is rebuilt on it) if, on every
holdout chain:

- G ≤ 0.02 after every update to layout C, and below the baseline's G;
- S ≥ 0.97 for every update;
- N ≥ 0.90 for every update from layout C to A or B;
- R holds for every update session;
- L: RMSE no more than 1.2 x the same sequence mapped alone, and ≤ 0.10 m;
- T holds.

If a criterion fails, the method stays an opt-in mode, the failure is recorded here with its
cause, and the next design is written as a new preregistration. Thresholds and measures in this
section do not change after the first holdout run.

### Data and machine notes

- The bags are 5-19 GB each (both LiDARs, bz2 chunks); the machine has about 11 GB free.
  `scripts/convert_luh_longterm_bag.py` reads a bag front to back over HTTP (resuming with Range
  requests, checking the dataset's md5) and writes a ROS 2 bag, so the original is never stored.
- What the bags hold (checked on the first 200 MB of S3): `/ouster/points` from an OS1-128 at
  2048 columns and 10 Hz (frame `ouster_lidar`; 262,144 points per scan, 22 % of them zero-range
  returns; fields x, y, z, intensity, `t` (uint32, ns after the header stamp, 0-100 ms),
  reflectivity, ring, ambient, range), `/ouster/imu` at about 100 Hz (frame `ouster/ouster_imu`),
  `/hesai/pandar`. The converter keeps x, y, z, intensity and `t`, drops zero-range returns and
  keeps every second column (the OS1's 1024-column mode): about 1.4 MB per scan with zstd
  instead of 12.6 MB.
- The bags have no `/tf_static`. The LiDAR-IMU extrinsic comes from the Ouster's documented
  sensor frames: the LiDAR frame turned 180 degrees about z, at (-0.006253, 0.011775, 0.028535) m
  in the IMU frame. On S3 (development) RKO-LIO odometry with it is closer to the total station
  than with no rotation (position RMSE 0.033 vs 0.041 m), so it is kept.
- The ground-truth files write each stamp with its tenth of a second fused on
  (`17410941326` is 1741094132.6 s); 10 Hz, one row per scan.

### L0 status (2026-11-18)

S3 converted (7.70 GB read, md5 as published, 1,189 scans and 11,886 IMU messages, 1.7 GB, 48
min at about 2.7 MB/s). RKO-LIO odometry alone (lidarslam 0.12.0 from PyPI, no ROS 2, voxel 0.25
m, the walking preset's registration settings): position RMSE 0.033 m, median 0.033 m, max 0.103
m against the total station over 1,183 matched scans and 44.5 m of path; 38 s for 119 s of data
on the development machine, 183 MB peak memory.

### L1 status (2026-11-18)

`lidarslam.live.Session` takes IMU and LiDAR messages one at a time and hands out every scan's
live pose as it is processed: the backend's causal pose (`OfflinePipeline::setLivePoseListener`,
the odometry corrected by the latest optimization the background worker had finished by the
scan's stamp). `python -m lidarslam live <log> ... --realtime` plays a log into a session at the
sensor's rate and writes `trajectory_live.tum` and `live_timing.json`. The CI test
(`lidarslam_core/test/test_python_live.py`) checks that the live poses are the causal trajectory
and that the session's files are byte-identical to `lidarslam map` with the causal output on.

S3 at sensor rate (development machine, 4 cores, walking backend preset): 118.7 s of data
streamed in 118.9 s with no message held back; a scan's live pose came 0.028 s after the scan's
data time (median; p95 0.26 s, max 0.67 s); the backend's remaining stages took 2.5 s after the
last message; peak memory 310 MB. Live poses against the total station: 0.033 m RMSE over 44.5
m (no loop edges in this sequence, so the live poses equal the odometry). The playback started
77 s after the command (opening the log and the session); to be looked at with the store (L2).

### L2 status (2026-11-18)

The store exists (`lidarslam.store.Store`, format version 1). A live session given a store
(`live.Session(..., store=)`, `python -m lidarslam live <log> ... --store STORE`) writes itself
to `sessions/<id>/` when it finishes and is then listed in `store.json` (one rename): the
backend's submaps in `submaps.bin` (stamp, odometry pose, optimized pose, the scan compressed as
the backend keeps it), `pose_graph.g2o`, and the parameters it was mapped with. `export/` is the
Autoware bundle written from the stored sessions (`python -m lidarslam store export STORE`);
`store rollback` takes the latest session off the list and leaves its files. `graph/` and
`evidence/` of the layout above come with L3 and L4. The C++ side is
`graph_based_slam/map_store.hpp` (session files, export) and `map_export::writeBundle`, now the
one map-bundle writer of both the backend and the store.

Check: the store's export of one session is byte-identical to the bundle the backend wrote. CI
(`lidarslam_core/test/test_python_store.py`, streamed and assembled map paths) and S3 (29
submaps; `map.pcd`, `map_bundle.yaml`, `map_projector_info.yaml`, `pose_graph.g2o` and the 15
`pointcloud_map/` files all identical). The S3 session takes 22 MB in the store; the export 4.8
MB. A store of two sessions is refused for export until they share a frame (L3), unless asked to
stack them.

The 77 s before playback in the L1 run did not come back: S3 at sensor rate with the store took
122.7 s from the command for 118.7 s of data (0.4 s before the first message, 3.4 s for the
backend's last stages and the store), with live poses 0.026 s after their scan's data time
(median; p95 0.033 s, max 0.045 s). The L1 run had shared the machine with the S6 conversion.

### L3 development status (2026-11-19)

`lidarslam.live.Session(..., store=..., initial_pose=...)` and `python -m lidarslam live
--store ... --initial-pose X Y Z QX QY QZ QW` verify a prior for the new odometry frame in the
store frame. Each scan window uses only messages received up to the decision. The initial
estimate must satisfy the 2 m and 20 degree bounds. A second distinct reference submap has to
support the same estimate. The confirmation search is centered on the estimate when the first
search returns one supporting submap. Stores written by the live path keep thinned reference
windows with their session files. Stores from L2 can be read using their individual submap scans.
An accepted session records `store_from_session` and exposes causal poses in `trajectory_store.tum`.

With the compiled matcher and recorded S3/S6 startup clouds, 76 of 91 perturbed nearby priors
joined for S3 → S6 and 60 of 91 for S6 → S3. All 278 wrong-site or wrong-heading priors and the
additional prior centered on the 4.75 m false match were refused.

Sensor-rate development chains with priors generated from the development ground truth:

| Chain | Update | Anchor wall time | Causal RMSE | Standalone RMSE |
| --- | --- | ---: | ---: | ---: |
| S1 → S3 → S6 | S3 | 1.58 s | 5.42 cm | 3.75 cm |
| S1 → S3 → S6 | S6 | 1.61 s | 3.42 cm | 1.55 cm |
| S6 → S3 → S1 | S3 | 1.68 s | 4.74 cm | 3.75 cm |
| S6 → S3 → S1 | S1 | 1.69 s | 8.40 cm | 3.71 cm |

All four first store poses met the position and heading tolerances. The standalone columns
use the same matched timestamps, with a rigid fit of each session's standalone causal path. The fixed
anchor exceeds the 1.2× localization ratio in all four updates. A causal replay of candidate
loop factors through g2o reduced RMSE in three of the four updates. The S6 and S1 updates in
the first and second chains still exceeded the ratio. The causal factor replay used recorded
keyframe windows and is a development experiment.

`live --track-store` (Python `live.Session(..., track_store=True)`) enables experimental live
tracking after startup. It verifies later windows against the store and adds the accepted
matches as robust inter-session factors. Observed reference submaps are fixed and active
session submaps are connected by odometry edges. The live pose stream uses the most recent
causal solution. Each accepted session saves its graph and absolute submap poses. Export and
later relocalization use those poses. This path still requires `--initial-pose`. Automatic
place recognition and a shared graph with movable stored poses and priors remain open in L3.

Sensor-rate replay through `live --track-store` on both development chains:

| Chain | Update | Accepted factors | Anchor wall time | Causal RMSE | Standalone RMSE | Ratio |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| S1 → S3 → S6 | S3 | 9 | 1.74 s | 4.11 cm | 3.75 cm | 1.10× |
| S1 → S3 → S6 | S6 | 22 | 1.64 s | 3.49 cm | 1.56 cm | 2.24× |
| S6 → S3 → S1 | S3 | 25 | 1.73 s | 4.27 cm | 3.75 cm | 1.14× |
| S6 → S3 → S1 | S1 | 30 | 1.67 s | 4.62 cm | 3.75 cm | 1.23× |

Each first store pose was within 0.025 m and 0.21 degrees. The driver reported zero trailing
frames, and median scan-to-pose lag was 0.02–0.03 s with p95 0.54–0.81 s. The two S3 updates
met the localization ratio. S6 had the tightest standalone comparator and slightly higher RMSE
than with the fixed anchor. Causally fitting one rigid transform to the accumulated, already
accepted factors in an offline diagnostic yielded 3.66 cm and 2.74 cm on the forward chain,
and 4.16 cm and 3.65 cm on the reverse chain. The diagnostic reused the live graph's accepted
factors. A live version would need a new replay because different corrections change later
matching priors. S6 still exceeds the ratio in the diagnostic.

`live --start-near-first-submap` is an experimental startup mode for a recording whose
operator independently knows it begins within 5 m of the first stored submap. It ranks Scan
Context matches in that area, then applies the existing geometric checks to the top eight
initializations. Conflicting verified poses cause a refusal. It derives the initializer from
the incoming LiDAR and the stored keyframes. A development replay of archived windows without
the area restriction found wrong-site aliases 8.68 m and 11.77 m away. Restricting the search
gave a correct, consistent result for all four archived development updates. The restriction
is an input assumption for the mode.

With incoming windows thinned to the density of the stored archive, both full development
chains ran at sensor rate using the launch-area search and the L4 persistence filter. No
ground-truth pose was supplied to the live runs. Scoring paired the live and standalone
estimates at common total-station timestamps:

| Update | Anchor data time | Causal RMSE | Standalone ratio | Active time / scan | Peak RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| S1 → S3 | 1.20 s | 4.07 cm | 1.09× | 70.8 ms | 813 MiB |
| S1 → S3 → S6 | 1.20 s | 1.99 cm | 1.28× | 77.4 ms | 950 MiB |
| S6 → S3 | 1.20 s | 4.32 cm | 1.16× | 68.6 ms | 881 MiB |
| S6 → S3 → S1 | 1.30 s | 4.20 cm | 1.12× | 63.6 ms | 996 MiB |

First-pose error stayed below 0.03 m and 0.20 degrees. There were zero trailing frames, with
one initializing scan per run. The earlier, unthinned startup probe refused S6 → S3. The
S6 update still exceeds the 1.2× L criterion.

Replaying the recorded factors at several uniform gains only reduced S6 to 1.97 cm. A candidate
that applied 10% of the position correction and 55% of the rotation correction looked promising
with recorded factors. On full unpaced sensor replay it gave 1.88 cm (1.208×) on S6 and
4.50 cm (1.202×) on the reverse S1 update. The sensor-rate runs above use the existing
35% correction for both, which remains the live setting.

On the S6 update, 11 of the 21 corrections after startup increased the error through the next
correction compared with keeping the preceding transform for that same interval. The correction
at 61.8 s raised interval RMSE from 2.52 to 3.41 cm and stayed in effect for 10.7 s. The one at
8.0 s raised it from 1.48 to 3.69 cm over 3.0 s. These comparisons use development ground truth
after playback and left the subsequent matching decisions fixed.

A shared-surface prototype builds a target from stored keyframe windows. It keeps points
within 0.10 m of a point in another aligned session, estimates local normals, and registers
each new window against the resulting near-horizontal and near-vertical surfaces. It uses
the existing verified startup anchor. The target is built without TLS or ground truth. On
recorded windows, using all stored planar surfaces without the agreement filter gave 2.27 cm
for S6 and 4.30 cm for S1 on the reverse chain. Shared surfaces gave 1.98 cm and 4.27 cm.
Using 35% of each correction at its keyframe gave 1.93 cm and 3.94 cm. These trials used the
same two committed reference sessions as the original live tracking runs.

Sensor-rate replay used the final update in each chain, with the committed output of each S3
run as store input:

| Final update | Live graph RMSE | Shared surfaces | Shared surfaces, 35% correction | Standalone ratio with 35% |
| --- | ---: | ---: | ---: | ---: |
| S1 → S3 → S6 | 3.49 cm | 1.99 cm | 1.93 cm | 1.24× |
| S6 → S3 → S1 | 4.62 cm | 4.32 cm | 3.98 cm | 1.06× |

The trials anchored within 1.73 s of playback start and reported zero trailing frames. The
35% variant had p95 scan-to-pose lag of 0.67 s on S6 and 0.53 s on S1. Building the shared
target took about 3.0–3.2 s in a separate timing probe. S6 remains above the 1.2× ratio.
The trial applied each correction directly to live poses.

`python -m lidarslam live --track-shared-surfaces` (Python `track_shared_surfaces=True`) now
includes shared-surface measurements in the tracking graph. It starts with the existing verified
anchor and builds the target from windows in aligned, committed sessions. When there are fewer
than two usable sessions it uses submap tracking. A match to the shared target is a robust SE(3)
edge from a fixed store-frame vertex (vertex 0 in `store_tracking_graph.g2o`). The optimizer
retains the full measurement and uses odometry edges between active submaps. Live output applies
35% of the graph's correction about the matched keyframe. Committing saves the graph's absolute
submap poses in `store_submap_poses.npy`, which both export and later relocalization read. The
session metadata records `shared_surface_observations`.

Full-rate tests appended a fourth session in a fresh process using the three-session store.
Both used a repeat of the S3 development recording as a revisit:

| Store and update | Shared factors | Anchor wall time | Causal RMSE | Standalone ratio |
| --- | ---: | ---: | ---: | ---: |
| S1 → S3 → S6 | 43 | 1.70 s | 1.98 cm | 1.27× |
| S1 → S3 → S6 → S3 | 28 | 1.56 s | 3.56 cm | 0.95× |
| S6 → S3 → S1 | 51 | 1.70 s | 4.16 cm | 1.11× |
| S6 → S3 → S1 → S3 | 28 | 1.61 s | 4.04 cm | 1.08× |

There were no trailing frames in the full runs. The new process read the exact absolute poses
from the third session, built a different shared target, and left its graph and submap files
byte-identical after committing the fourth. The S6 ratio is still above 1.2×.

A second test split the S6 development recording into disjoint intervals (0–43 s and 47–85 s).
The early interval contributed 14 shared factors, then the later interval reopened that store
in a separate process and appended 12 shared factors. With a prior derived from standalone S6
odometry and the development alignment, the later interval anchored 6.2 s after the first live
pose timestamp (7.26 s wall time after playback began). It had 2.24 cm causal RMSE over 314
matched poses and two pending LiDAR frames at the clip end. Both intervals come from one
development recording. The holdout remains unopened.

### L4 development status (2026-11-19)

An offline S3 → S6 ray-update probe used a fixed store frame, sampled TLS points and approximate
hall bounds. Requiring repeated free-space observations from separated viewpoints for clearing
old points retained more TLS coverage than treating every cross-frame hit and miss as dynamic.
The conservative variant retained 1.02× the base TLS coverage, with a ghost fraction of 0.479
in that probe. Tightening the approximate area to the TLS walls yielded 1.00× coverage and a
ghost fraction of 0.382. Of the 6,990 retained old ghost voxels in that area, 4,070 had enough
free-space observations from separated viewpoints but were protected by neighbouring hits, and
2,846 received a hit themselves. The same neighbour rule protected 537 old voxels within 0.10 m of
the TLS reference after repeated free-space observations. Requiring persistent neighbour hits
reduced G to 0.285. Excluding neighbours with repeated free-space observations lowered it to
0.229. Also allowing deletion after a single hit when repeated free-space evidence was available
yielded G = 0.146 and S = 0.998. These development variants used one scan per keyframe in the same
approximate area. With one scan in five, the same comparison was G = 0.429 and S = 1.009 for the
conservative rule, and G = 0.268 and S = 1.005 for the last variant. The planned per-submap evidence
and live change callbacks were outside that probe.

A second probe counted a ray crossing as free-space evidence only in a frame with no hit in
that voxel, then compared hit and free-space counts once the minimum number of separated
free-space observations was met. At one scan per keyframe, the best tested setting with S ≥ 0.97
gave G = 0.102 and S = 0.975. At one scan in five, the best such setting gave G = 0.091 and
S = 0.971. The two settings differ. In the keyframe run, 1,039 of the 1,166 remaining ghosts
came from the old map, at a median distance of 0.192 m from the sampled TLS. Both runs use the
same approximate area and fixed store frame. No L4 parameters have been frozen.

Of those 1,039 retained old ghosts, 706 were within 0.25 m of the sampled TLS and 856 were below
1 m above the floor. Only 24–27 passed a conservative test for a nearby parallel TLS wall
when the TLS neighbourhood used for the normal estimate was varied. This test leaves the other
points unclassified. For the same keyframe run, counting hits on an old voxel only within
0.09 m of its representative point yielded G = 0.100 and S = 0.983 at the best tested setting
with S ≥ 0.97. The same setting with one scan in five gave G = 0.088 and S = 0.988. A different
setting at that sampling rate gave G = 0.075 and S = 0.973. These are offline results in the
same approximate area. The ghost fraction remains above the 0.02 target.

`python -m lidarslam live --detect-changes` (Python `detect_changes=True`) is an experimental
old-map removal path. Once the prior is verified, it traces one deskewed scan per accepted
backend keyframe, including queued keyframes from startup. The C++ traversal uses a 0.15 m
world-grid index of old local-voxel centroids, a 0.30 m ray endpoint margin and a 40 m range
cap. Hits within 0.09 m of a centroid take precedence over free space in the same scan. A
voxel is suppressed when free-space votes outnumber hits by at least 8:1, with at least eight
free-space votes and viewpoints at least 0.50 m apart. Two or more hits with at least as many
hits as misses can restore an earlier suppression. Unobserved voxels keep their previous state.

`map_change_evidence.npz` belongs to the updating session. It records reference session IDs,
local submap voxel indices, per-scan vote counts, separated viewpoints and the resulting
decision. `Session.changes()` exposes the accumulating evidence. Export folds committed
decisions in session order. Rollback drops the latest decisions and regenerates the map from
the earlier sessions without rewriting their scans. This version applies masks to export;
tracking still uses the archived submaps. It uses the same world grid for candidate lookup and
ray traversal. Local ray intersection, grazing-angle rejection and change callbacks remain open.

Both development chains completed with the removal path enabled. The tables use the exported
0.10 m maps, the approximate wall bounds above, and a subsample of the TLS points. The control
exports use the same session poses with removal masks disabled.

| Chain update | Local voxels marked removed | S | G after update to C |
| --- | ---: | ---: | ---: |
| S1 → S3 | 41,724 | 1.171 | — |
| S1 → S3 → S6 | 84,457 | 1.018 | 0.451 |
| S6 → S3 | 21,491 | 1.013 | — |
| S6 → S3 → S1 | 48,889 | 1.005 | — |

At the S6 update, G was 0.653 for the unmasked control and 0.493 when S6 was mapped alone.
Old-map removal leaves substantial ghost returns in the new scans. For S6 → S3, the development
cross-layout N proxy was 0.198, versus 0.368 in the unmasked control. That proxy uses the S1
reference, so it is not a same-layout holdout measurement. The prescribed
`merge --place-recognition` baseline declined to join the S3 update in both directions and
produced no baseline maps. Both development stores were rolled back two steps in copies of
their indexes. At each step the exported bundle matched its saved predecessor byte for byte,
and the committed session files remained identical. All four runs reported zero trailing
frames. The concurrent replays and baseline jobs shared the machine, and the last updates
ran slower than sensor time. These results do not establish the runtime gate.

`python -m lidarslam live --filter-new-points` (Python `filter_new_points=True`) adds an
experimental persistence filter on top of old-map removal. It collects one localized scan
about every 0.5 s. Each 0.15 m store-frame cell can receive one hit or one free-space vote
per sampled scan, with hits taking precedence. Free-space votes begin after the first hit.
A new voxel is retained when a representative within 0.15 m has at least three hits and
at least as many hits as misses. Filtering covers new submap voxels within 40 m of the sensor
and requires at least three sampled scans. The sampled scans and keyframe poses are causal.
At commit, the capture-time pose maps each decision to an immutable submap-local voxel, so
it still masks the same points when export uses the optimized pose.

The updating session stores its suppression mask as the optional `new_rows` array in
`map_change_evidence.npz`. Older evidence files can still be read. Later observations can
restore a suppressed voxel, and rolling back the update discards its mask. A stationary
transient that receives repeated consistent hits can still pass this filter.

Both sensor-rate development chains completed with no trailing frames. An export of the same
sessions and poses that ignored only the `new_rows` arrays served as the old-removal-only
control. Scoring used the same sampled TLS and approximate wall bounds as above.

| Chain update | New local voxels suppressed | S | Absolute TLS coverage | Control coverage |
| --- | ---: | ---: | ---: | ---: |
| S1 → S3 | 24,205 | 1.132 | 85.48% | 88.44% |
| S1 → S3 → S6 | 47,656 | 1.025 | 91.76% | 92.35% |
| S6 → S3 | 20,763 | 1.010 | 92.54% | 92.78% |
| S6 → S3 → S1 | 30,542 | 1.002 | 92.99% | 93.51% |

S6 ghost fraction fell from 0.451 to 0.188. Of the 7,338 remaining ghost points, 1,963 had a
neighbour within 0.06 m only in the old submap export, 2,836 only in the new submap export,
and 2,538 in both. One had no neighbour in either component export. This is a proximity
diagnostic because the combined export downsamples the joint point cloud. The S6 → S3
cross-layout N proxy fell from 0.198 to 0.121. Of the 1,948 reference points that lost coverage,
1,174 were then more than 0.30 m from the map. Both stores again rolled back to the two prior
exports byte for byte with their session files intact. The runtime and quality gates for
holdout have not been established.

A sequential timing comparison replayed the same 43 s S6 development clip into copies of
the same two-session store. With old removal only and with the persistence filter, streaming
wall time was 43.00 s in both runs. Whole-process CPU time was 49.8 s and 53.3 s, and peak RSS
was 706 MiB and 735 MiB, respectively. Scan-to-pose p95 was 0.49 s and 0.45 s. The clip ended
with two pending LiDAR frames in each run. In the full-chain runs with the persistence filter,
scan-to-pose p95 ranged from 0.40 to 0.64 s.

`live --refine-visibility` is a separate L4 experiment for new submaps. It caches the 0.5 s
scan samples used by the persistence filter. The DDA first locates candidate voxels. The voxel
centroid must then lie within 0.085 m of the ray and before its endpoint margin for a free-space
vote. A hit within 0.10 m takes precedence in the same scan.

The new submap candidate index is complete before replaying the cached rays at commit.
This counts free space even when observed before a candidate's first hit. New voxels within
40 m remain when they have at least two hit votes and at least as many hits as misses.
The updating session stores submap-local masks and records `map_change_refined_visibility`
in the index.

In a same-recording diagnostic for S3, the first 58 s supplied candidate submaps and the
reference came from submaps captured after 64 s. With the same old-voxel rule and sampled
TLS, counting all misses reduced G from 0.192 to 0.181 and reduced reference coverage from
0.404 to 0.374. This time split uses archived poses and is a geometry diagnostic.

The initial prototype also used the same scan samples to score old voxels, with a four-to-one
free-space to hit threshold. The S1 → S3 → S6 and S6 → S3 updates were replayed at sensor rate
from the same pre-update stores and with the same launch-area startup as the persistence-filter
control.
Their saved submap pose arrays matched the control exactly. With the same development polygon
and all TLS points, G after S6 fell from 0.1533 to 0.1350. Absolute static coverage was 0.9180
and 0.9152, respectively. The S6 → S3 cross-layout N proxy fell from 0.109 to 0.091. Both
replays had zero trailing frames and averaged 73–87 ms per
scan. S6's post-replay processing and export took 55.5 s, compared with 19.4 s in the control
run. Each rollback restored the pre-update export byte for byte.

Recomposing the masks at the same poses isolated the two changes. Updating only the old-voxel
rule gave S6 G = 0.1544. Updating only the new-voxel rule gave G = 0.1337 and raised the S3
cross-layout N proxy from 0.0910 with both changes to 0.0982. This was the basis for keeping
the old removal policy and applying the refinement to new submaps.

The resulting implementation was replayed again at sensor rate on both updates. Its map
exports matched the full-TLS new-only reassembly byte for byte. S6 G fell from 15.33% to
13.37% and absolute TLS coverage fell from 91.80% to 91.45%. On S6 → S3 the cross-layout
N proxy was 9.82%, down from 10.89% with the persistence filter. Of 5,052 remaining S6 ghosts,
1,745 had a point within 0.06 m only in the old-map export, 1,612 only in the new-map export,
and 1,694 in both. One had no such neighbour. Both runs reported zero trailing frames and
56–64 ms of active wall time per scan. Post-replay processing and export took 31.6 s on
S3 and 56.9 s on S6. The map quality and commit cost remain open.

One offline plane-voting trial grouped neighbouring new points with aligned normals and
used multi-view hit and miss evidence to confirm or suppress each group. At the same poses
and with the same old-map points, it produced S6 G of 9.50% versus 9.42% for point voting.
The S3 cross-layout new-structure proxy was 9.41% for both. These maps were reconstructed
from saved vote geometry and scored with the full TLS cloud. The trial left the live
filter unchanged.

Recovering omitted points that agreed with a locally planar retained neighbourhood improved
the sampled development N proxy only from 0.121 to 0.125 at the widest tested radius. It
also increased G from 0.188 to 0.189. A separate filter that kept only well-supported planar
neighbourhoods lowered G to 0.120 at S = 0.973 with the sampled TLS. Stricter variants lost
more static structure. Both are offline probes and did not change the live filter.

### L5 evaluation tooling (2026-11-19)

- `scripts/evaluate_luh_trajectory.py` scores R and L against total-station poses and T from
  the live summary. It estimates the base-to-TLS transform once from the standalone base
  trajectory, matches the two localization paths at shared truth timestamps and records
  the transform in its report. Replay summaries include the first input stamp, active wall
  time per scan and process peak RSS. The T report lists the scan-to-pose count gap and the
  trailing-frame count. The initial LiDAR scan may initialize odometry without a pose.
  Published total-station timestamps sometimes repeat at 0.1 s resolution. Scoring uses the
  first sample for each repeated timestamp.
- `scripts/evaluate_luh_map_update.py` scores map exports with the full TLS cloud streamed
  in chunks. It takes the floor elevation, an explicit hall polygon, an update path in TLS
  coordinates and the fixed transform. N requires a separately supplied reference map and
  transform. A refusal log records an unavailable place-recognition baseline.
- `scripts/evaluate_luh_holdout.py` consumes a JSON manifest mapping each of the six
  preregistered update keys (such as `S0-S4-S7/S4`) to relative `map` and `motion` report
  paths. It applies the declared thresholds and reports the product or opt-in decision.
  A missing G comparison cannot demonstrate improvement over the baseline.

With the same approximate development polygon but the full TLS cloud, G after S6 was 0.153
with new-point filtering and 0.421 with old removal alone. Static coverage was 0.918 and
0.924, respectively. For S6 → S3, the S1 cross-layout N proxy was 0.109 versus 0.187. These
measurements exercise the evaluation path. The holdout measurement setup was subsequently
frozen below.

The full launch-area replays gave G = 0.153 after S6 and the same S6 → S3 development
N proxy of 0.109. This uses the same development polygon and full TLS cloud. No holdout
log or holdout ground truth was used in these trials.

### L5 holdout freeze (2026-11-19)

An S6 development replay with the two existing sessions profiled 55.77 s in commit,
including 39.75 s of new-voxel visibility replay and 7.23 s of export. One trial used
direct grid lookups for the same indexed voxels. It produced byte-identical stored
poses, evidence and map PCD, and took 53.47 s in the matching replay (4.1% less).
The original sparse lookup is used for holdout.

Use the existing `live` store path at sensor rate with `--start-near-first-submap`,
`--track-shared-surfaces` and `--refine-visibility` on every update. Start the base
session in an empty store. Use the known launch area within 5 m of the first stored
submap for the startup prior. Supply no pose from ground truth and do not retry
relocalization with a different prior. A refused update fails R. A successor that
requires an unaligned predecessor is recorded as blocked.

For every sequence, use `bench_report/data/long_term_2026-11/rko_lio_luh_os1.yaml`,
`lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml` and converter column step
2 with the published MD5. Use the full published empty-hall TLS cloud. Use the same
approximate hall polygon as development, with TLS XY vertices (11, 7.7), (51.5, 7.7),
(51.5, 14.4), (11, 14.4) and floor elevation −1.39 m. Apply the height band and the
10 m window around the update's total-station path from the preregistration.

Fit the store-to-TLS transform once from the base session's causal odometry. Use
standalone offline maps for the new-structure reference, with S1 for S0 and S3 for
S2 and S4. Compare each update with the existing `lidarslam-map merge --place-recognition
--engine core` on the same prefix of the chain. Record a baseline refusal as a
missing comparison. Keep the preregistered thresholds. The holdout total-station
trajectories had yet to be opened at freeze.

### L5 holdout results (2026-11-19)

| Chain | Update | G (%) | S (%) | N (%) | R (s) | L (cm, ratio) | T (ms/scan) | Offline merge |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| S0 → S4 → S7 | S4 | — | 117.80 | — | 1.30 | 10.54, 7.541 | 58.0 | Refused |
| S0 → S4 → S7 | S7 | 37.56 | 102.22 | — | 1.30 | 10.06, 6.898 | 58.4 | Refused |
| S7 → S4 → S0 | S4 | — | 100.32 | 21.80 | 1.30 | 1.52, 1.091 | 59.6 | Refused |
| S7 → S4 → S0 | S0 | — | 75.98 | — | 1.30 | 15.05, 1.334 | 49.6 | Refused |
| S2 → S5 | S5 | 11.56 | 100.44 | — | 1.20 | 1.84, 1.151 | 63.6 | Refused |
| S5 → S2 | S2 | — | 100.97 | 34.28 | 1.20 | 5.58, 1.040 | 64.0 | Refused |

All six updates passed R and T, with zero trailing frames dropped. The two
three-session stores each exported a map with three aligned sessions. Commit and
export took 23.9–80.1 s after replay.

Five updates passed S. In S7 → S4 → S0, absolute TLS coverage fell from 91.17% to
69.27% after S0. The three updates involving S0 as base or update exceeded both L
limits. S0's standalone RMSE was 11.28 cm.

G exceeded 2% in both updates to layout C. N was below 90% in both updates directly
from layout C to B. Offline place recognition refused all six comparisons. In each
case the accepted matches provided too little support for one transform across three
places spaced at least 5 m apart.

The six-report evaluator returned `opt-in`. L5 evaluation is complete. The
preregistered product-adoption rule keeps the mode experimental.

### S0 trajectory follow-up

The causal odometry used to create the S0 base store had 11.28 cm RMSE against
the total station after a rigid fit over 3,147 poses. The table compares each
update on the same matched timestamps with the frozen L5 store-to-TLS transform
and with an independent rigid fit of the saved update trajectory to TLS.

| Update | Matched poses | L5 RMSE (cm) | Independently fitted update (cm) | Independently fitted standalone (cm) |
| --- | ---: | ---: | ---: | ---: |
| S0 → S4 | 1,554 | 10.54 | 1.53 | 1.40 |
| S0 → S4 → S7 | 1,171 | 10.06 | 1.50 | 1.46 |
| S7 → S4 → S0 | 3,136 | 15.05 | 11.03 | 11.28 |

S4 and S7 retained close to standalone trajectory accuracy after a per-session
rigid fit. Most of their measured error is associated with placement in the fixed
frame established by the S0 base. For S0 as the update, about 11 cm of error was
still present after the independent fit, consistent with its standalone result.
The independent fits use ground truth for diagnosis. The L5 scores use the
original fixed transform.

### Review and roll back an experimental update

Copy the last accepted store while it is idle. Use fresh paths for the copy,
the reference export and the run output. The example assumes a known launch
area within 5 m of the first stored submap and uses the replay's sensor parameters.

```bash
accepted=/path/to/accepted-store
candidate=/path/to/candidate-store
before=/path/to/pre-update-export
bag=/path/to/update-bag
run=/path/to/update-run
odometry_params=/path/to/odometry.yaml
backend_params=/path/to/backend.yaml
python -m lidarslam store export "$accepted" --out "$before"
cp -a "$accepted" "$candidate"
python -m lidarslam live "$bag" --store "$candidate" --out "$run" \
  --odometry-params "$odometry_params" --backend-params "$backend_params" \
  --realtime --start-near-first-submap --track-shared-surfaces --refine-visibility
python -m lidarslam store info "$candidate"
```

Before using the candidate for the next session, check that its latest session
matches the replay's `store_session`, that `store_anchor_stamp_sec` is set, and
that `dropped_trailing_frames` is zero. Compare `$candidate/export/map.pcd` with
`$before/map.pcd` in a point-cloud viewer. Check alignment and inspect permanent
surfaces and expected changes in the area just surveyed. A session without an
anchor can appear in the store index while the export still shows the previous map.

When the latest update should be undone, confirm its ID with `store info` and run
the rollback once on the candidate. Run the export explicitly to cover the case
where the remaining sessions have no change evidence.

```bash
python -m lidarslam store rollback "$candidate"
python -m lidarslam store info "$candidate"
python -m lidarslam store export "$candidate"
cmp "$before/map.pcd" "$candidate/export/map.pcd"
```

Confirm that the committed session list and map match the pre-update state before
using the candidate again. Rollback removes the latest entry from the index and
keeps its session files. The three-session S7 → S4 → S0 store was tested on a copy.
Rolling back S0 and then S4 reproduced both earlier map PCDs byte for byte, with
all session file checksums intact.

## Milestones

Each milestone is one reviewable change with its own checks.

1. **L0, this document**, the data converter for LUH (Ouster + IMU, ROS 2 bag), and a first
   look at S1/S3/S6 with the existing single-run mapping (does each map alone; ATE against the
   total station).
2. **L1, ROS-free live mapping.** `lidarslam.live.Session` over `GraphSlamApplication` and the
   RKO-LIO driver, fed scan by scan; `lidarslam-map live <log> --realtime`. Check: with no store,
   the live run's trajectory equals the causal output of the offline backend on the same input,
   and real time holds on the development sequences.
3. **L2, the store.** Write a store from a live session; load it; export the Autoware bundle from
   it (byte-identical to the single-run bundle when the store has one session).
4. **L3, relocalization and the live multi-session graph.** Anchor a new session to the store,
   loops between new and stored submaps, priors on stored poses. Check: R and L on the
   development chains.
5. **L4, change detection.** Per-submap evidence, ray tracing, the four rules, tombstones,
   rollback. Check: G, S and N on the development chains; parameters frozen and recorded here.
6. **L5, holdout evaluation.** Holdout chains once, under the decision rule above; then the
   product decision.
7. **L6, product.** `lidarslam-map update` and `merge` on the store, the wizard's `--join-map`,
   docs; a ROS 2 node over the same adapter.

## Risks

- **Relocalization in a changed layout.** Layout A hides most of the hall; Scan Context may find
  no confident match against a layout C map (the join then refuses, by design). A caller-given
  start pose (the cart starts at the same spot in most sequences) is the fallback that is
  checked, not trusted.
- **Removing what is only out of view.** Thin structures (table legs, chair backs) are hit by few
  rays and crossed by many; the neighbour and grazing-angle rules exist for this, and S measures
  it.
- **One dataset.** All public data with a known changed layout and a reference scan found so far
  is this one hall. Static-structure checks on other multi-session data (no change expected) are
  added when it is downloaded again; they can only lower confidence, not replace the holdout.

## Agent review, development trial (preregistered 2026-11-19, before any measurement)

Question: does reviewing an update's candidates with the map review tools
(`docs/map-review.md`, `python -m lidarslam mcp`) improve the map over the automatic change
detection alone? Development data only (S1, S3, S6); no holdout data is used.

- **Stores.** Two chains, each a base session in an empty store and one update, both with
  `live --start-near-first-submap --detect-changes --filter-new-points` (walking backend preset,
  `rko_lio_luh_os1.yaml`, column step 2): S3 → S6 (update to layout C) and S6 → S3 (update to
  layout B).
- **Arms.** A: the store's export after the update, no edits. B: the same store after one
  agent review, committed as one `agent:claude` edit.
- **Reviewer.** Claude, through `lidarslam.map_review.Review` only (the MCP tools'
  implementation: `info`, `list_candidates`, `describe`, `view_region`, `decide`,
  `decide_box`, `commit`). It sees no TLS, total-station or other reference data and no score
  until its edit is committed. Budget: the 20 largest candidates of each kind (60 per chain),
  in list order; it may leave any undecided. It decides from the tool's numbers and pictures with
  the instructions of the MCP server (`lidarslam/mcp_server.py`, `INSTRUCTIONS`) as written at
  this commit.
- **Measures.** `scripts/evaluate_luh_map_update.py` with the full TLS cloud
  (`Reference_Map_TLS.pcd`, md5 `231eb6939447570e407ebad56029e151`), the L5 hall polygon and
  floor (−1.39 m), the update's path, and the store-to-TLS transform fitted once per chain
  from the base session's odometry against its total-station positions (Umeyama, rigid). G
  and S on both chains; N on S6 → S3 against S1 mapped alone (the development N proxy).
- **Rule.** Review helps on a chain if, against A, G falls by at least 0.02 (absolute) while S
  stays at least 0.97 and, on S6 → S3, N falls by no more than 0.02. Both chains are reported
  whatever the outcome, with the decisions and the time the review took.

### Agent review, development results (2026-11-19)

Both reviews were committed before any TLS or total-station data was read. The
store-to-TLS fits used 1,183 (S3) and 1,218 (S6) matched poses, with 3.7 cm and 1.6 cm RMSE;
the S1 reference fit 1,626 poses at 3.8 cm. The TLS md5 matched. "Before" is the base session
exported alone.

| Chain | Arm | G (%) | S (%) | N (%) | Map points in region |
| --- | --- | ---: | ---: | ---: | ---: |
| S3 → S6 | A, no edits | 14.45 | 100.71 | — | 34,463 |
| S3 → S6 | B, agent edit | 13.91 | 100.66 | — | 34,085 |
| S6 → S3 | A, no edits | 29.55 | 100.79 | 11.04 | 41,205 |
| S6 → S3 | B, agent edit | 27.52 | 100.81 | 9.55 | 39,851 |

Decisions (60 candidates per chain): S3 → S6 kept 13, removed 34, left 13 undecided
(16,801 voxels in the edit); S6 → S3 kept 19, removed 31, left 10 undecided (13,701 voxels).
Removing a `removed_old` or `suppressed_new` candidate only confirms the evidence; the map
changes come from keeps on those kinds and removes on `added`. The review time was not
recorded.

Under the rule:

- **S3 → S6: does not help.** G fell by 0.0054, below 0.02. S held.
- **S6 → S3: helps, at the margin.** G fell by 0.0203. S held at 100.81%. N fell by 0.0149,
  within 0.02.

Both arms leave G far above the 2% L5 target, and N on S6 → S3 is about 10% in both: the
automatic update misses most new structure, and review, which only sees what the update
produced, cannot add it back. The review removed some points that the S1 reference counts as
new structure. One chain at the margin on development data is not enough to change the
product. A further check would need a fresh public holdout, since the LUH holdout ground
truth is open.

### New structure after an update: what the development N proxy measures (2026-11-21)

The development N proxy (S6 → S3 against S1 mapped alone) is about 10% for every variant, so
it looked as if the update loses most new structure. It does not measure that. S1 is layout A
(the poster corridor) and S3 layout B (L-shaped walls, tables); S1's non-TLS points are mostly
structure that layout B does not have. Two references built from S3 itself, in the store
frame of the S6 → S3 store, show what the update keeps:

- **S3 alone:** S3's points farther than 0.20 m from the TLS, people included.
- **S3, both halves:** of those, the points within 0.10 m of a point from the other half of
  S3 (submaps split at the middle), thinned to 0.10 m, so most passing people drop out.

| S6 → S3 export | G (%) | N, S1 proxy (%) | N, S3 alone (%) | N, S3 both halves (%) |
| --- | ---: | ---: | ---: | ---: |
| no change masks | 55.91 | — | 99.86 | 99.86 |
| old-map removal only | 41.47 | — | 99.86 | 99.86 |
| `--filter-new-points` (arm A above) | 29.55 | 11.04 | 55.79 | 91.28 |
| `--refine-visibility` | 26.96 | — | — | 85.61 |
| arm A + agent review (arm B above) | 27.52 | 9.55 | 49.62 | 83.14 |

On S3 → S6, `--refine-visibility` gave G = 11.92% against 14.45% for `--filter-new-points`,
with S = 100.57%.

- Old-map removal never touches the update's own points. The persistence filter keeps 91%
  of the new structure S3 saw in both halves, and drops much of what it saw only once.
- The agent review removed new structure: 91% → 83% on the both-halves reference. Its
  development result above was scored on the S1 proxy and should be read with this.
- The L5 N measure (another session of the same layout mapped alone, everything farther than
  0.20 m from the TLS) counts that session's people and its briefly seen points as new
  structure. Against S3 itself, a perfect update could reach only what S3's own points
  support, about 56% with this filter; N ≥ 0.90 against such a reference cannot be met by
  any update that removes people. A holdout N needs a reference that keeps only structure
  confirmed across time.
- In S6 most cells farther than 0.20 m from the TLS form a diffuse cloud over the open hall
  (10–15 pedestrians), and their per-cell hit, miss and hit-span counts overlap those of
  S3's furniture: no threshold on per-cell votes separated the two in an offline table over
  the recorded votes (`NewPointEvidence.cells()`). The remaining gap is G in layout C, not N.

Replays used the same pre-update stores (`store rollback` of the scored stores) and
reproduced the scored exports byte for byte.

### Ghosts after an update to layout C: what G counts (2026-11-21)

G counts every map point farther than 0.10 m from the empty-hall TLS. On S3 → S6 the map
points were split by the store's own points: **static, not in the TLS** when the S3 session
and both halves of S6 (submaps split at the middle) have points within 0.10 m. Layout B
structure cannot be seen by S6, and a person rarely stands at one spot in both halves. The
remainder is **removable**: what a correct update could drop.

| S3 → S6 export | G (%) | static, not in the TLS (%) | removable (%) |
| --- | ---: | ---: | ---: |
| no change masks | 55.56 | 5.97 | 49.59 |
| old-map removal only | 44.10 | 7.16 | 36.94 |
| `--filter-new-points` (arm A) | 14.45 | 8.11 | 6.33 |
| `--refine-visibility` | 11.92 | 7.45 | 4.47 |
| arm A + agent review (arm B) | 13.91 | 8.40 | 5.51 |

With `--refine-visibility`, the removable part was about one third old (S3) points and two
thirds S6 points, per store point thinned to 0.10 m per session. Half of those S6 points lie
in one area beside the start of the S6 path. Their filter cells were hit over a median span
of 114 s of the 122 s session (median 64 hits): objects present for all of S6, missing from S3
and from the TLS, below 1.1 m for 90% of them. An update has to keep them. Only about a tenth of
the removable S6 points had a hit span under 10 s, the signature of a passer-by.

What an update could still remove is therefore about 2% of the map: the old S3 points that the
S6 scans never saw through (about 2.1% of thinned store points) and the few passers-by
(about 0.4%). G ≤ 2%, the L5 target, can only be checked if the reference counts structure
that is static but absent from the TLS as correct. The next
evaluation should therefore not use the empty-hall scan alone.

- **G:** a ghost is a point farther than 0.10 m from the TLS and from every other session of
  the same layout mapped alone; LUH has three sessions per layout C (S5, S6, S7).
- **N:** the reference is the new structure that two other sessions of the same layout show
  (both within 0.10 m), not one session's points farther than 0.20 m from the TLS.

Both need data that is not yet open for a blind run; the LUH holdout ground truth is open since
L5, so the next formal measurement needs another public dataset with repeated layouts.

## NCLT map-update evaluation (preregistered 2026-11-22, before the holdout run)

LUH's holdout ground truth is open since L5, and its empty-hall TLS cannot tell a ghost from
static structure missing from the scan (above). The next measurement uses the University of
Michigan North Campus Long-Term dataset ([NCLT](https://robots.engin.umich.edu/nclt/), ODbL 1.0,
Carlevaris-Bianco et al., IJRR 2016): 27 sessions of one campus over 15 months, a Velodyne
HDL-32E and a Microstrain IMU, and SLAM ground-truth poses. References are other sessions, not a
scan of the true scene.

### Data and preparation

- `scripts/convert_nclt_session.py` streams each session's Velodyne archive and writes a ROS 2
  bag with every 6th return (disk); `bench_report/data/long_term_2026-11/rko_lio_nclt.yaml` holds
  the extrinsics. Odometry parameters were not tuned.
- Area: the 150 m square at x −180…−30 m, y −585…−435 m (NCLT local frame), the window with the
  most 10 m cells visited by all eight chain sessions. Each session keeps its longest pass
  through it, ±10 s (`bench_report/data/long_term_2026-11/nclt/windows.json`). Only 1 Hz
  ground-truth positions were used to pick the area and the windows.
- The update bag starts at the first ground-truth position within 1.5 m of the base pass
  (`trim.py`), because startup registration searches stored keyframes within 3 m for 10 s.
- **Known start:** the update's `--initial-pose` comes from ground truth: the base session's
  odometry and the update mapped alone are each fitted to their ground truth (rigid), and the
  update's pose at its first scan is carried into the store frame (`prior.py`). R (finding the
  start) is not measured here.

### Chains

| Chain | Base | Update | References |
| --- | --- | --- | --- |
| development | 2012-01-15 | 2012-05-26 | 2012-05-11, 2012-06-15 |
| holdout | 2012-02-18 | 2012-11-16 | 2012-11-04, 2012-11-17 |

### Arms

- **C:** `live --store --initial-pose` (no change detection: the update is appended).
- **A:** C plus `--detect-changes --filter-new-points`.
- **B:** C plus `--detect-changes --refine-visibility`.

Each arm starts from a copy of the same base store.

### Measures (`scripts/evaluate_nclt_map_update.py`, d = 0.5 m)

Each reference session is mapped alone, placed in the store frame by its ground-truth fit and
refined by point-to-plane ICP against the arm's export. Points are scored within 10 m (XY) of the
update path and of both reference paths, 0.3–3.0 m above the local ground (lowest point per 2 m
cell).

- **Ghost points:** export points with no point of either reference within d. G is their share.
- **S:** coverage of persistent structure (reference 1 within d of reference 2 and of the map
  before the update), after divided by before.
- **N:** share of new structure (reference 1 within d of reference 2, farther than 2d from the map
  before) that the export covers within d.

d = 0.5 m was chosen on the development chain, where ICP left a median residual of 0.13–0.18 m
and d = 0.3 m counted 14% of the base map as ghost.

Development (d = 0.5 m):

| Arm | Ghost points | vs C | G (%) | S | N (%) |
| --- | ---: | ---: | ---: | ---: | ---: |
| C | 20,813 | 1.00 | 3.52 | 1.000 | 94.93 |
| A | 12,513 | 0.60 | 2.88 | 0.998 | 83.62 |
| B | 12,554 | 0.60 | 2.82 | 0.999 | 89.43 |

### Decision rule (holdout)

An arm (A or B) **helps on NCLT** when, on the holdout chain, its ghost points are at most 0.75 of
arm C's, S ≥ 0.97 and N ≥ 0.85. If both help, the one with the higher N is preferred. The
result is reported whatever it is, with G for every arm. It decides between the two change
modes; it does not by itself make the long-term mode the default.

### NCLT holdout results (2026-11-22)

All three arms anchored with the known start; the update bag started 11 s into its window.
ICP moved the references by 1.3–2.3 m from their ground-truth placement (median residual
0.15–0.18 m).

| Arm | Ghost points | vs C | G (%) | S | N (%) | Rule |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| C, append | 38,556 | 1.00 | 9.96 | 1.000 | 99.12 | — |
| A, `--filter-new-points` | 17,052 | 0.44 | 5.73 | 0.994 | 71.16 | fails N |
| B, `--refine-visibility` | 24,898 | 0.65 | 8.04 | 0.999 | 93.48 | **helps** |

Under the preregistered rule, **B helps on NCLT** and A does not (N below 0.85). B is the
preferred change mode.

The preregistered measure aligns the references to each arm's export separately. Reference 1
moved 1.98–2.28 m depending on the arm, so the scored region and the new-structure reference
differ between arms: A had 2,569 new-structure points against 20,490 for B and 25,679 for C, and
its map before the update counted 2.4% ghost against 12.8% and 15.9%. A post-hoc check (not
preregistered) aligned both references once to the map before the update and scored every arm
against that same placement:

| Chain | Arm | Ghost points | vs C | S | N (%) |
| --- | --- | ---: | ---: | ---: | ---: |
| holdout | C | 80,645 | 1.00 | 1.000 | 35.53 |
| holdout | A | 37,875 | 0.47 | 0.993 | 32.32 |
| holdout | B | 41,168 | 0.51 | 0.993 | 33.14 |
| development | C | 122,107 | 1.00 | 1.000 | 86.51 |
| development | A | 56,608 | 0.46 | 0.994 | 77.43 |
| development | B | 65,029 | 0.53 | 0.994 | 80.15 |

With the shared placement the references sit on the old map rather than on the update, so even
append keeps only 36% of the holdout's new structure: N is then limited by how well the update
is placed, not by the change mode. What holds in every analysis: both change modes cut ghost
points to about half of append (0.44–0.65) with S ≥ 0.99, and B keeps more new structure than
A (by 2.7–22 points of N). The absolute N depends on the reference placement; a placement that
uses the update's own map and the old map together is the next fix to the measure.

## Start an update anywhere in the store (development, 2026-11-22)

Startup registration only searched stored keyframes within 3 m of a given pose, or within 5 m
of the first stored submap, for the first 10 s of data; on NCLT the update had to be trimmed and
given a ground-truth start. `live --relocalize-anywhere` (experimental) instead:

- ranks every stored keyframe by Scan Context distance and registers only candidates closer
  than 0.18 (NCLT development: correct top candidates had median distance 0.13 and 90% below
  0.19; wrong ones median 0.23, mostly places the store does not hold), with the existing
  corroboration by a neighbouring keyframe and refusal when two verified candidates disagree;
- tries one keyframe every 2 s of data for as long as the session runs, and anchors only when
  two such searches give session transforms within 0.5 m and 2°;
- keeps at most 600 queued scans for change detection while it searches.

Development, store = 2012-01-15 (NCLT), each session from its first scan, no initial pose:

| Session | Anchored after | Path offset from the ground-truth placement (RMS, max) | Processing |
| --- | ---: | --- | ---: |
| 2012-05-11 | 87.1 s | 1.48 m, 3.79 m | 196 s for 504 s of data |
| 2012-06-15 | 135.5 s | 1.33 m, 3.58 m | 132 s for 520 s |
| 2012-05-26 (from +83 s) | 3.4 s | 0.52 m, 0.74 m | 81 s for 457 s |

The ground-truth placement itself is only good to 1–2 m here (reference sessions needed
1.3–2.3 m of ICP correction above), so the offsets show the anchors are in the right place,
not how accurate they are. A holdout run with a preregistered rule (anchor rate, wrong anchors
checked by ICP against the stored map) is still to come.

### Holdout check of `--relocalize-anywhere` (preregistered 2026-11-22, before the run)

The method is frozen as committed in `75c20ee4` (Scan Context gate 0.18, one search per 2 s,
two agreeing searches within 0.5 m and 2°). Its thresholds were chosen on the development store
only; the holdout sessions below have not been run with it.

- **Store:** the NCLT holdout base store (2012-02-18 pass, `h_store`).
- **Sessions:** 2012-11-04, 2012-11-16 (from its first scan, not trimmed) and 2012-11-17, each
  `live --store COPY --relocalize-anywhere` with no initial pose, odometry and backend parameters
  as in the NCLT evaluation.
- **Correct anchor:** both (a) the session path placed by the anchor lies within 3.0 m RMS of the
  same path placed by ground truth (each session's and the base's odometry fitted rigidly to their
  NCLT ground truth; that placement is itself good to 1–2 m), and (b) point-to-plane ICP of the
  session's map, placed by the anchor, against the stored base map moves it by at most 1.0 m with
  a median residual of at most 0.3 m.
- **Rule:** the mode works on the holdout if every session anchors correctly within 180 s of
  data and no session anchors wrongly. A session that never anchors is a miss, not a wrong anchor;
  a wrong anchor fails the rule outright. Anchor time and processing time are reported.

#### Holdout results (2026-11-22)

| Session | Anchored after | Offset from the ground-truth placement (RMS) | ICP correction (median residual) | Preregistered verdict |
| --- | ---: | ---: | ---: | --- |
| 2012-11-04 | 6.1 s | 0.80 m | 1.29 m (0.16 m) | wrong (ICP > 1.0 m) |
| 2012-11-16 | 15.1 s | 1.07 m | 1.48 m (0.18 m) | wrong (ICP > 1.0 m) |
| 2012-11-17 | 25.0 s | 0.83 m | 0.28 m (0.15 m) | correct |

All three sessions anchored from their first scan without a start pose, in 6–25 s of data;
processing took 74–92 s for 440–570 s of data. **Under the preregistered rule the mode fails**:
two sessions exceed the 1.0 m ICP bound, which the rule counts as wrong anchors.

A post-hoc check (not preregistered) ran the same ICP from the ground-truth placement as well.
In all three sessions the two ICP runs converged to the same pose (0.00 m apart), and ICP moved
the ground-truth placement by 2.12, 1.35 and 1.28 m. So the 1.0 m bound fails the ground-truth
placement too: ICP of a whole 400–570 s session map against the base map finds a best fit about
1–2 m away from any single anchor, and the bound did not separate right from wrong anchors. No
anchor was shown to be wrong. The bound was not checked on the development store before it was
written, which is the mistake.

The mode stays experimental. A next check would preregister a criterion calibrated on the
development store, for example that ICP from the anchor and ICP from the ground-truth placement
converge to the same pose, on sessions not used here, and would add sessions that never enter
the stored area (no anchor expected).

### Calibrating the anchor check on the development store (2026-11-22)

`scripts/evaluate_relocalize_anywhere.py` compares an anchor (`store_from_session`) with two
things: the ground-truth placement `T_g` (base and session odometry each fitted rigidly to NCLT
ground truth) and point-to-plane ICP of the session's map against the base map, run once from
the anchor (`M_a`) and once from `T_g` (`M_g`). `converged_apart` is the RMS distance between the
session's odometry positions placed by `M_a` and by `M_g`, and the angle of `M_a·M_g⁻¹`.

Development store 2012-01-15, the three development sessions of the table above re-run with the
frozen method (anchored after 87.1 s, 135.5 s and 3.4 s; offsets 1.45, 1.27 and 0.36 m RMS from
the ground-truth placement). Each anchor was also replaced by perturbed copies
(`anywhere_perturb_grid.txt`: XY shifts of 0.5–5 m in four directions, yaw turns of 2–10° about
the path's centroid, and shift-and-turn combinations; 38 per session,
`bench_report/data/long_term_2026-11/nclt/anywhere_calibration_dev.jsonl`):

| Start (3 sessions) | n | `converged_apart` (m) | Offset from `T_g` (m RMS) | ICP median residual (m) |
| --- | ---: | ---: | ---: | ---: |
| the anchor | 3 | 0.000 | 0.36–1.45 | 0.136–0.192 |
| shift 0.5–2 m | 48 | 0.000 | 0.31–2.84 | 0.136–0.192 |
| turn 2° | 6 | 0.000 | 1.83–3.03 | 0.136–0.192 |
| shift 3 m | 12 | 0.000–1.155 | 2.72–3.75 | 0.136–0.230 |
| shift 5 m | 12 | 3.34–4.70 | 4.71–5.66 | 0.220–0.267 |
| turn 5° (with 0–2 m shift) | 30 | 1.64–5.06 | 4.77–6.68 | 0.196–0.268 |
| turn 10° | 6 | 8.58–10.70 | 9.68–11.98 | 0.228–0.261 |

ICP from any start within 2 m and 2° of the anchor reaches exactly the pose ICP reaches from the
ground-truth placement; from 5 m or 5° it never does (at least 1.6 m apart, with a higher
residual); 3 m is the edge of the basin. So "ICP from the anchor and from the ground truth
converge to the same pose" is a check that the anchor lies within about 2–3 m and 2° of the best
fit to the stored map. It cannot tell anchors closer than that apart, which the 1–2 m ground
truth cannot either. The old 1.0 m bound on the ICP correction would have called 39 of the 57
starts within 2 m and 2° wrong.

### Second holdout check of `--relocalize-anywhere` (preregistered 2026-11-22, before the run)

Method frozen as in `75c20ee4` (unchanged since). Store: the NCLT holdout base store (2012-02-18
pass, `h_store`), odometry and backend parameters as in the NCLT evaluation. No session below
has been run with the method; their ground truth has been used only at 1 Hz, as positions, to
choose the windows.

- **Sessions** (`select_anywhere_holdout.py`, `anywhere_holdout2_windows.json`): for each of four
  periods (2012-01–02, 03–05, 06–09, 2012-10–2013-04), the NCLT session not used so far with the
  longest pass through the evaluation square: 2012-02-02, 2012-03-17, 2012-09-28, 2012-10-28.
  - **In-area run:** that pass ±10 s (510–1,109 s), from its first scan, no start pose.
  - **Out-of-area run:** in the same session, the first 480 s of its longest stretch that stays at
    least 100 m (XY) from every ground-truth position of the stored base pass (closest approach
    100.0–238.1 m). The store holds nothing of these places, so no anchor is expected.
- Each run: `live --store COPY --relocalize-anywhere` (`run_anywhere.sh`), then
  `evaluate_relocalize_anywhere.py` against the base map (`holdout_anywhere2.sh`).
- **Correct anchor:** `converged_apart` ≤ 0.25 m and ≤ 0.5°, and the anchor's offset from the
  ground-truth placement ≤ 4.0 m RMS (on the development store every start within 2 m and 2°
  was ≤ 3.03 m and every start 5 m or 5° off was ≥ 4.71 m). Any other anchor is **wrong**, in an
  in-area or an out-of-area run.
- **Rule:** the mode works on this holdout if every in-area run anchors correctly within 180 s of
  data and no run (in-area or out-of-area) anchors wrongly. A run that never anchors is a miss,
  not a wrong anchor; one wrong anchor fails the rule outright. Anchor time, processing time and
  every measure above are reported for all eight runs, whatever the outcome.

Not part of the rule: under this check the three sessions of the first holdout would all count
as correct (ICP from the anchor and from the ground truth converged 0.00 m apart; offsets
0.80–1.07 m). That observation shaped the check, which is why it is tested on new sessions.

#### Second holdout results (2026-11-22)

All eight runs as preregistered (`holdout_anywhere2.jsonl`):

| Run | Anchored after | Offset from `T_g` (m RMS) | ICP correction (median residual) | `converged_apart` | Verdict | Processing / data |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| 2012-02-02 in-area | 4.0 s | 0.62 | 0.24 m (0.16 m) | 0.000 m, 0.00° | correct | 129 s / 1,109 s |
| 2012-03-17 in-area | 3.4 s | 0.76 | 0.81 m (0.17 m) | 0.000 m, 0.00° | correct | 75 s / 510 s |
| 2012-09-28 in-area | 5.7 s | 1.01 | 0.54 m (0.17 m) | 0.000 m, 0.00° | correct | 78 s / 531 s |
| 2012-10-28 in-area | 6.6 s | 1.04 | 0.73 m (0.19 m) | 0.010 m, 0.01° | correct | 77 s / 616 s |
| 2012-02-02 out-of-area | — | — | — | — | no anchor | 53 s / 480 s |
| 2012-03-17 out-of-area | — | — | — | — | no anchor | 345 s / 480 s |
| 2012-09-28 out-of-area | — | — | — | — | no anchor | 660 s / 480 s |
| 2012-10-28 out-of-area | — | — | — | — | no anchor | 498 s / 480 s |

**Under the preregistered rule the mode works on this holdout:** every in-area run anchored
correctly within 7 s of data, and no run anchored wrongly; none of the four runs outside the
stored area anchored.

Outside the stored area the search never stops, and two of the four runs took longer than their
data (660 s and 498 s for 480 s): a session that drives away from the store would fall behind
real time. Bounding the search cost (fewer candidates per search, or a pause after repeated
failures) is the next change before the mode can leave experimental; it needs its own check that
the in-area anchors above still hold.

### Bounding the search outside the stored area (development, 2026-11-22)

A profile of the 2012-09-28 run outside the stored area: 157 of 176 searches had no candidate
under the Scan Context gate and took 0.06 s; the other 19 registered 1–8 candidates at 6–10 s
each and found nothing, 700 s in all. Since `b837593`, a search that registers candidates and
finds nothing doubles the wait before the next search (data time, from 2 s), and a found
candidate resets it. The cap was chosen on the development store only, with two new runs outside
it (`anywhere_dev_out_windows.json`: the first 480 s of the longest stretch of 2012-05-11 and of
2012-06-15 at least 100 m from the 2012-01-15 pass):

| Cap | Anchored after (05-11 / 06-15 / 05-26) | Processing outside the store (05-11 / 06-15, 480 s each) |
| --- | --- | --- |
| 2 s (no back-off) | 87.1 / 135.5 / 3.4 s | 83 / 283 s |
| **8 s** (`b4d097e`) | 92.8 / 141.0 / 3.4 s | 82 / 133 s |
| 16 s | 149.2 / 141.0 / 3.4 s | 81 / 69 s |
| 32 s | 233.5 / 141.0 / 3.4 s | 82 / 72 s |

All development anchors stayed correct under the calibrated check. With the 8 s cap the eight
runs of the second holdout were repeated (`anywhere_backoff_holdout2_recheck.jsonl`; a regression
check on data already used, not a new holdout): the four in-area runs gave the same anchors as
before (3.4–6.6 s, `converged_apart` ≤ 0.01 m), and the four runs outside the store did not
anchor and took 62, 146, 306 and 182 s for 480 s of data (before: 53, 345, 660 and 498 s). The
mode stays experimental: it now keeps up with real time on these runs, on this machine, but a
search that registers candidates still blocks the session for up to a minute.

### Searching off the tracking path (development, 2026-11-22)

With the back-off a search that registers candidates still blocked tracking for up to a minute.
Since this change, `live --relocalize-anywhere --realtime` runs each search on a worker thread
(the matcher releases the GIL) and uses its result at the first keyframe after it finishes.
Without `--realtime` searches stay in line, so replays, and every number above, stay
deterministic; with it the anchor depends on wall-clock timing, as real-time playback already
does. Two runs played in real time on this machine, in line (as before) and on the worker
(`anywhere_realtime_background.jsonl`; 2012-09-28 outside the stored area is a run of the second
holdout, used here only for timing):

| Run | Search | Anchored after | Pose lag p50 / p95 / max |
| --- | --- | ---: | --- |
| 2012-09-28 outside the store (480 s) | in line | — | 56.9 / 185.5 / 196.6 s |
| | worker | — | 0.017 / 0.042 / 0.174 s |
| 2012-05-11 on the development store (504 s) | in line | 92.8 s | 0.017 / 5.4 / 12.9 s |
| | worker | 97.4 s | 0.017 / 0.25 / 1.2 s |

The worker run's anchor on the development store is correct under the calibrated check (ICP
from the anchor and from the ground truth 0.00 m apart; 1.43 m RMS from the ground-truth
placement). Searches no longer hold up tracking; the mode stays experimental until a live run
on a store not used for its development has been checked.

### NCLT map update with one reference placement for every arm (2026-11-22)

The NCLT measure placed the references by ICP against each arm's own export, so the scored region
and the new-structure reference differed between arms. `evaluate_nclt_map_update.py
--align-target` takes one map as the ICP target for every arm; here it is the append arm C's
export, which holds both the old and the new structure. `chain_shared.sh` re-runs one chain
(base store, update mapped alone, known start, arms A, B and C, both references) and scores it
both ways (`rerun_{d,h}_{own,shared}_{A,B,C}.json`). Both chains were re-run from scratch in this
environment.

| Chain | Arm | Ghost points (own / shared) | vs C (shared) | S (shared) | N (own / shared) |
| --- | --- | ---: | ---: | ---: | ---: |
| development | A, `--filter-new-points` | 13,074 / 11,753 | 0.58 | 0.998 | 87.7 / 86.3 % |
| development | B, `--refine-visibility` | 12,950 / 11,893 | 0.59 | 0.999 | 91.3 / 91.6 % |
| development | C, append | 20,104 / 20,104 | 1.00 | 1.000 | 95.2 / 95.2 % |
| holdout (re-run, post hoc) | A | 45,474 / 43,834 | 0.72 | 0.994 | 91.0 / 54.4 % |
| holdout (re-run, post hoc) | B | 42,151 / 42,542 | 0.70 | 0.994 | 57.1 / 58.5 % |
| holdout (re-run, post hoc) | C | 60,613 / 60,613 | 1.00 | 1.000 | 67.2 / 67.2 % |

The development chain reproduces the committed one closely (arm C: 20,104 ghost points and N
95.2 % against 20,813 and 94.9 %; the update was trimmed 81 s instead of 83 s). **The holdout
chain does not reproduce the committed holdout result**: the base map already differs (195,113
scored points against 194,009, and 9.3 % of it counted as ghost against 15.9 %), and arm C now
has G 14.7 % and N 67.2 % against 9.96 % and 99.1 %. The committed holdout was run in another
environment with an earlier build, and its runs are gone, so the cause is not known. With its own
placement, arm A's N on the holdout re-run jumps to 91.0 % because its references are placed
differently (2.07 m vs 1.86 m correction), the artefact the shared placement removes.

What holds under the shared placement on both chains: both change modes cut ghost points to
0.58–0.72 of append with S ≥ 0.99, and B keeps at least as much new structure as A (91.6 vs
86.3 %, 58.5 vs 54.4 %). The absolute N of the holdout chain is not stable across environments
(99 % then, 67 % now, for append), so the holdout decision recorded above (B helps, N ≥ 0.85)
rests on a number that this re-run does not reproduce; that decision stands as recorded, with
this caveat. Any further change to the change modes needs a new holdout chain, preregistered,
and run and scored in one environment.

### Live check of `--relocalize-anywhere` on a new store (preregistered 2026-11-23, before the run)

Method as merged in #635 (`bceac32`): the search of #633 with the back-off of #634, run on a worker
thread under `--realtime`. Nothing below has been run with it. Ground truth has been used only at
1 Hz, as positions, to choose the windows (`select_anywhere_live.py`,
`anywhere_live_windows.json`); every session used so far in this document is excluded.

- **Store:** the unused NCLT session with the longest pass through the evaluation square,
  2012-02-05 (that pass ±10 s, 709 s), mapped offline into a new store with the NCLT odometry
  and backend parameters.
- **Sessions:** for each later period (2012-03–05, 06–09, 2012-10–2013-04) the unused session
  with the longest pass: 2012-03-25, 2012-08-20, 2012-12-01. Each gives an **in-area run** (its
  pass ±10 s, 421–495 s) and an **out-of-area run** (the first 480 s of its longest stretch at
  least 100 m from every position of the stored pass; closest approach 197.7–326.9 m).
- Each run: `live --store COPY --relocalize-anywhere --realtime`, the in-area and out-of-area
  runs of one session played at the same time (two at a time on this 4-core machine, as for the
  development timing above), once each, then `evaluate_relocalize_anywhere.py` against the base
  map (`holdout_anywhere_live.sh`).
- **Correct anchor:** as calibrated above: `converged_apart` ≤ 0.25 m and ≤ 0.5°, and offset
  from the ground-truth placement ≤ 4.0 m RMS. Any other anchor is wrong.
- **Rule:** the mode works live on this store if (a) every in-area run anchors correctly within
  180 s of data, (b) no run anchors wrongly, and (c) every run keeps the pose lag at p95 ≤ 1.0 s
  and max ≤ 5.0 s (development, on the worker: p95 0.04–0.25 s, max 0.17–1.2 s). A run that never
  anchors is a miss for (a), not a wrong anchor. All six runs are reported whatever the outcome.
- If the rule holds, `--relocalize-anywhere` is proposed to leave experimental status (a separate
  change, for the maintainer to decide).

#### Live check results (2026-11-23)

All six runs as preregistered (`holdout_anywhere_live.jsonl`):

| Run | Anchored after | Offset from `T_g` (m RMS) | `converged_apart` | Verdict | Pose lag p95 / max |
| --- | ---: | ---: | ---: | --- | --- |
| 2012-03-25 in-area (495 s) | 48.0 s | 0.55 | 0.000 m | correct | 0.52 / 1.57 s |
| 2012-08-20 in-area (464 s) | 40.2 s | 1.02 | 0.000 m | correct | **18.9 / 31.8 s** |
| 2012-12-01 in-area (421 s) | 37.8 s | 0.67 | 0.002 m | correct | **11.3 / 23.2 s** |
| 2012-03-25 out-of-area (480 s) | — | — | — | no anchor | 0.05 / 1.54 s |
| 2012-08-20 out-of-area (480 s) | — | — | — | no anchor | 0.05 / 0.15 s |
| 2012-12-01 out-of-area (480 s) | — | — | — | no anchor | 0.04 / 0.07 s |

(a) holds (every in-area run anchored correctly, in 38–48 s of data) and (b) holds (no wrong
anchor, none of the out-of-area runs anchored), but **(c) fails**: two in-area runs fell behind
real time by up to 32 s. **Under the preregistered rule the mode does not work live on this
store**, and it stays experimental.

Post-hoc diagnosis (not preregistered): 2012-08-20 in-area was played in real time again, alone,
once with `--relocalize-anywhere` and once with no store at all (`anywhere_live_lag_probe.jsonl`).
Both fell behind the same way (p95 19.9 s and 18.0 s, max 30.9 s and 26.0 s), and the per-pose lag
was below 0.13 s for the first 300 s in both, so the search is not the cause. The lag builds up
from about 420 s of data, where the path returns near its start: the backend registers a
distance-based loop candidate on every new submap (most are then skipped as redundant), and each
of those keyframes held tracking for 3.1–3.6 s. The failure of (c) is therefore a property of the
live backend's loop closure, present without a store; `--relocalize-anywhere` itself met (a) and
(b). Bounding the backend's loop-candidate work in live sessions (fewer candidates per submap, or
loop closure off the tracking path like the search) is the next change, and it needs a new
preregistered live check, on sessions not used here, before the mode can leave experimental.

Where the time goes (post hoc, `py-spy --native` on 2012-08-20 in-area played offline with no
store): 66 % of all samples are in `BackendCore::searchLoopForSubmap`, almost all of it small_gicp
GICP verification of loop candidates (`knn_search` alone 49 %); the RKO-LIO frontend is 19 %. The
existing `loop_search_query_stride` (search every N-th submap; 1 in the walking preset) bounds it
without a code change. On the same run in real time, with the store (`anywhere_live_stride_probe.jsonl`;
a session of the live check, so this is not a selection of the stride):

| `loop_search_query_stride` | Anchored after | Pose lag p95 / max | Loop edges |
| ---: | ---: | --- | ---: |
| 1 (live check) | 40.2 s | 18.9 / 31.8 s | 2 |
| 4 | 40.2 s | 0.38 / 3.56 s | 2 |
| 8 | 39.0 s | 0.06 / 3.48 s | 2 |

Its cost on the development sessions, each mapped offline alone (ATE against NCLT ground truth,
rigid fit; optimized / causal trajectory):

| Session | Stride 1 | Stride 4 | Loop edges (1 → 4) |
| --- | --- | --- | --- |
| 2012-01-15 | 0.376 / 0.391 m | 0.376 / 0.391 m | 0 → 0 |
| 2012-05-11 | 0.605 / 0.675 m | 0.652 / 0.750 m | 6 → 5 |
| 2012-06-15 | 0.447 / 0.500 m | 0.449 / 0.506 m | 4 → 4 |
| 2012-05-26 | 0.478 / 0.499 m | 0.471 / 0.497 m | 5 → 5 |

A stride of 4 keeps three sessions within 1.5 % and loses one loop on 2012-05-11 (+8 % ATE). The
development sessions do not fall behind real time even with stride 1 (max lag 1.2 s above), so
they cannot choose the stride by lag. Two ways on: a live setting with a coarser loop-search
stride (simple; costs some loops), or loop verification off the tracking path as for the search
(keeps every loop; a larger change to the shared offline pipeline, which must stay deterministic
when not in real time). Either needs a new preregistered live check on unused sessions.

### Backend loop search off the tracking path (development, 2026-11-23)

Since this change, `background_loop_search` (set by `live.Session` for `background_search`, that
is `replay`/`live --realtime`) runs the backend's loop search and its causal graph optimizations
on a worker thread over a snapshot of the submaps. Every query still runs, in order, and the
remaining ones run at `finish()`, so the loop edges and every output after the pass are the
in-line ones; only the causal (live) corrections depend on timing. Without `--realtime` nothing
changes.

- 2012-08-20 in-area mapped offline with the worker and in line: `loop_edges.csv`,
  `trajectory_optimized.tum`, `trajectory_raw.tum`, `odometry_tum.txt` and `map.pcd`
  byte-identical; `trajectory_causal.tum` differs, as designed.
- The same run in real time with the store (`anywhere_live_background_loop_probe.jsonl`; a session
  of the live check, so post hoc): anchored after 40.2 s as before, both loop edges kept, pose lag
  p95 0.045 s and max 0.14 s (18.9 s and 31.8 s with the search in line). The worker ran at most
  19.8 s behind a submap's stamp, so live corrections arrive late instead of tracking stopping.

### Second live check of `--relocalize-anywhere` (preregistered 2026-11-23, before the run)

As the live check above, with the backend loop search on the worker (the change above) and new
sessions. Windows from 1 Hz ground-truth positions only (`select_anywhere_live.py` with the four
sessions of the first live check also excluded; `anywhere_live2_windows.json`):

- **Store:** 2012-01-22 (its pass ±10 s, 518 s).
- **Sessions:** 2012-03-31, 2012-08-04, 2013-02-23; in-area runs 466, 425 and 369 s; out-of-area
  runs 480 s each, closest approach 186.8, 249.7 and 101.0 m.
- Runs, correct-anchor check and **rule (a), (b), (c)** exactly as in the first live check
  (`holdout_anywhere_live2.sh`): every in-area run anchors correctly within 180 s of data; no run
  anchors wrongly; every run keeps pose lag at p95 ≤ 1.0 s and max ≤ 5.0 s. Reported whatever
  the outcome.

#### Second live check results (2026-11-23)

All six runs as preregistered (`holdout_anywhere_live2.jsonl`):

| Run | Anchored after | Offset from `T_g` (m RMS) | `converged_apart` | Verdict | Pose lag p95 / max |
| --- | ---: | ---: | ---: | --- | --- |
| 2012-03-31 in-area (466 s) | 175.4 s | 2.20 | 3.169 m, 3.00° | **wrong** | 0.026 / 0.072 s |
| 2012-08-04 in-area (425 s) | **222.7 s** | 1.11 | 0.000 m | correct, late | 0.038 / 0.109 s |
| 2013-02-23 in-area (369 s) | 7.0 s | 0.55 | 0.000 m | correct | 0.044 / 0.101 s |
| 2012-03-31 out-of-area (480 s) | — | — | — | no anchor | 0.030 / 0.079 s |
| 2012-08-04 out-of-area (480 s) | — | — | — | no anchor | 0.032 / 0.074 s |
| 2013-02-23 out-of-area (480 s) | — | — | — | no anchor | 0.041 / 0.112 s |

(c) now holds on every run (max pose lag 0.11 s), so the backend worker does what it was for. But
(a) fails (2012-08-04 anchored after 222.7 s, beyond 180 s; 2012-03-31 anchored wrongly) and (b)
fails (the 2012-03-31 anchor). **Under the preregistered rule the mode does not work live on this
store**, and it stays experimental.

Post hoc, about the 2012-03-31 anchor (not a change of verdict): ICP from the anchor moved it by
0.13 m to a median residual of 0.131 m, while ICP from the ground-truth placement moved that by
1.67 m to a higher residual, 0.169 m, 3.2 m and 3.0° away. The anchor is the better fit to the
stored map by this measure, so the calibrated check, which takes the ground-truth start as the
reference basin, may have called a good anchor wrong; it may equally be a wrong place that fits
well. The check cannot tell these apart, which is its limit: the ground-truth placement is only
good to 1–2 m and a few degrees. The late anchors (175 s and 223 s, against 7–48 s on the other
five in-area runs of both live checks) are the other open point.

Post-hoc diagnosis of the second live check (not preregistered). The two in-area runs that
failed (a) were run again offline (in line, deterministic) with every search logged:

- **Late anchors are the gate, not real time.** Offline, 2012-08-04 anchored after 126.4 s and
  2012-03-31 after 252.0 s, also late. On the 2012-01-22 store the best Scan Context distance of
  a search stayed above the 0.18 gate for most searches: median 0.201 (2012-08-04) and 0.262
  (2012-03-31), with 8 of 35 and 3 of 43 searches under the gate, against median 0.225 and 9 of 60
  for a development session (2012-06-15 on the 2012-01-15 store). The gate was chosen on one
  development store; this winter store and spring/summer sessions sit further apart.
- **The 2012-03-31 anchor is reproducible.** The offline run anchored at a different time from a
  different keyframe and landed on the same placement: ICP from the anchor moved it 0.12 m to a
  median residual of 0.131 m, again 3.17 m and 3.0° from where ICP from the ground-truth placement
  converges (residual 0.169 m). Two independent anchors agreeing, with the better fit, point to the
  check's reference basin (ICP from the 1–2 m ground-truth placement) rather than to the anchor.
  This does not change the verdict.

Next: a check that does not take the ground-truth start as the reference basin (for example ICP
from several starts, the anchor's solution counting as correct when it is the best-residual one
within the ground-truth tolerance), calibrated on development data and preregistered, and a
look at the Scan Context gate across seasons on development stores.

### Anchor check v2: the best fit near the ground truth (development, 2026-11-23)

`evaluate_relocalize_anywhere.py --gt-starts` also runs ICP from eight perturbed copies of the
ground-truth placement (`anywhere_gt_starts.txt`: 2 m in four directions, ±2° yaw, two
shift-and-turn combinations). Among the solutions from the ground truth, those starts and the
anchor that lie within 4.0 m RMS of the ground-truth placement, the one with the lowest median
residual is the best fit; **v2 counts an anchor correct when ICP from it reaches that best fit
(within 0.25 m and 0.5°) and the anchor lies within 4.0 m RMS of the ground-truth placement.** It
no longer takes the ground-truth start, good only to 1–2 m and a few degrees, as the reference.

On the development calibration (`anywhere_calibration_dev_v2.jsonl`: the same 3 sessions and
38 perturbations per anchor) v2 gives the same verdict as v1 on all 117 perturbed starts and the
3 anchors: every start within 2 m and 2° correct, every start 5 m or 5° off wrong, 7 of 12 at 3 m.

Post hoc (not a check of anything: these runs were all scored before, and v2 was written after
seeing the 2012-03-31 run), v2 on every in-area anchor of the second holdout and both live checks
(`anywhere_rescore_v2_posthoc.jsonl`): all ten correct. The only change is 2012-03-31 (live
check 2), whose anchor is itself the best fit (median residual 0.131 m against 0.145–0.169 m from
the ground-truth starts). v2 must be preregistered before it decides anything; with it, the open
point of `--relocalize-anywhere` is the late anchors on the 2012-01-22 store (the Scan Context
gate), not wrong anchors.

### The Scan Context gate across seasons (development, 2026-11-23)

Post hoc on sessions already used (the development store and every run of the second holdout
and both live checks are development data now), not a check of anything. Each of 25 sessions
(`anywhere_gate_jobs.txt`) was run once with every keyframe's search input saved and no search
(`record_anywhere_queries.py`); until a session anchors these inputs do not depend on the search.
`gate_study.py` then replays `--relocalize-anywhere` in line at gates 0.18–0.26: the same
candidates, registration, consensus, back-off and two-search agreement, each candidate registered
once for all gates. At 0.18 the replay gives the current code's anchor time exactly (2012-05-11:
92.79 s replayed and run; 2012-06-15: 140.98 s against 141.0 s). `gate_analysis.py` places every
verified candidate and anchor against the ground-truth placement and marks a search in the store
when the ground truth puts its keyframe within 5 m of the base session's path
(`anywhere_gate_study.jsonl`).

**No gate up to 0.26 gave a wrong candidate.** Of the 271 verified candidates of the replayed
searches (counted once per gate), none was more than 4 m RMS from the ground-truth placement and
none was found outside the store; none of the 12 out-of-area runs anchored at any gate. Registration, not the gate, keeps
wrong places out on these data.

**Anchor time counts time outside the store.** Several "in-area" windows start outside the
stored area: 2012-08-04 (second live check) enters it after 75.2 s, 2012-05-11 and 2012-06-15
after about 80 s. Seconds from entering the store to the anchor, 13 in-area runs:

| Run | Enters store | 0.18 | 0.20 | 0.22 | 0.24 | 0.26 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2012-03-31 (live 2, winter store) | 9.7 s | **242.3** | 96.1 | 161.8 | 88.7 | 10.2 |
| 2012-08-04 (live 2, winter store) | 75.2 s | 51.2 | 136.5 | 24.0 | 24.0 | 49.0 |
| 2012-06-15 (development) | 79.6 s | 61.4 | 61.4 | 58.1 | 58.1 | 55.9 |
| 2012-05-26 (development) | 0.1 s | 53.0 | 10.3 | 10.3 | 15.9 | 15.9 |
| 9 other runs | 0.1–21.1 s | 3.3–12.6 | 3.3–11.0 | 3.3–13.8 | 3.3–11.6 | 3.3–13.1 |
| median / max | | 7.1 / 242.3 | 7.1 / 136.5 | 7.4 / 161.8 | 8.6 / 88.7 | 8.0 / 55.9 |

All anchors are within 2.32 m RMS of the ground-truth placement (the 2.3 m one is the 2012-03-31
placement that v2 found to be the best fit). The time is not monotonic in the gate: a higher
gate also lets more searches that find nothing double the wait (up to 8 s), so the searches
that fall in the store change. Only 0.26 keeps every run within 60 s of entering the store, and
that rests mostly on one run (2012-03-31).

**Cost.** Candidates under the gate that a search registers (an upper bound: a disagreement
stops early), all runs:

| Gate | In-area searches / candidates | Out-of-area searches / candidates |
| --- | --- | --- |
| 0.18 | 193 / 113 | 1311 / 100 |
| 0.22 | 137 / 266 | 621 / 1127 |
| 0.26 | 94 / 522 | 604 / 2567 |

A registration takes seconds, so out of the store 0.26 costs about 25 times the registration
work of 0.18 (with the background worker this delays searches, not tracking).

Next: a preregistered check of gate 0.26 with v2 as the anchor check and the anchor time
counted from entering the store (ground truth, decided before the run), on sessions not used
so far; the remaining unused NCLT sessions are few, so a new part of the campus is likely needed.

### Check of the Scan Context gate at 0.26 in a new part of the campus (preregistered 2026-11-23, before the run)

The development replay above raised the gate of `matchAnywhere` from 0.18 to 0.26
(`startup_relocalization.hpp`, `ANYWHERE_DESCRIPTOR_GATE`). This check decides whether that
holds on data not used for it. Everything below is fixed before any of these runs.

- **Data.** Only NCLT sessions never used so far, in a new 150 m square that does not overlap the
  old one, chosen from 1 Hz ground-truth positions only (`select_anywhere_gate.py`,
  `anywhere_gate_windows.json`): x −60…90 m, y −260…−110 m.
  - Store: 2012-01-08 (its pass ±10 s, 468 s).
  - Sessions: 2012-02-04, 2012-02-12, 2012-02-19, 2012-04-29, 2013-01-10, 2013-04-05.
  - Each gives an in-area run (its pass ±10 s; 412, 370, 431, 228, 227 and 250 s) and an
    out-of-area run (480 s, 2013-01-10 255 s; at least 100 m from the store's path).
- **Runs** (`holdout_anywhere_gate.sh`), as in the live checks: `live --relocalize-anywhere
  --realtime` on a copy of the store, in-area and out-of-area runs of a session in parallel.
- **Entering the store.** The first ground-truth position of the run within 5 m (XY) of a
  ground-truth position of the store's window. A run that never comes that close is reported and
  left out of (a).
- **Anchor check v2**, as defined above (`--gt-starts` with `anywhere_gt_starts.txt`): an anchor
  is correct when ICP from it reaches the best fit near the ground truth (within 0.25 m and 0.5°)
  and it lies within 4.0 m RMS of the ground-truth placement.
- **Rule.**
  - **(a)** Every in-area run that enters the store anchors, correctly by v2, within 60 s of data
    after entering it.
  - **(b)** No run anchors wrongly: no anchor that v2 finds wrong, and no anchor in an
    out-of-area run.
  - **(c)** Every run keeps pose lag at p95 ≤ 1.0 s and max ≤ 5.0 s.

  The gate stays at 0.26 only if (a), (b) and (c) all hold; otherwise it goes back to 0.18. The
  result is reported whatever the outcome, with the anchor time from the start of each run as well.

#### Results of the gate-0.26 check (2026-11-23)

All twelve runs as preregistered (`holdout_anywhere_gate.jsonl`):

| Session | In-area run | Enters store | Anchored after | After entering | v2 | Offset from `T_g` | Out-of-area run |
| --- | ---: | ---: | ---: | ---: | --- | ---: | --- |
| 2012-02-04 | 412 s | 0.1 s | 83.0 s | **82.9 s** | correct | 0.25 m | no anchor |
| 2012-02-12 | 370 s | 70.4 s | **—** | — | — | — | no anchor |
| 2012-02-19 | 431 s | 98.3 s | 141.0 s | 42.7 s | correct | 1.17 m | no anchor |
| 2012-04-29 | 228 s | 70.4 s | **—** | — | — | — | no anchor |
| 2013-01-10 | 227 s | 0.1 s | 6.8 s | 6.7 s | correct | 0.34 m | no anchor |
| 2013-04-05 | 250 s | 0.1 s | 13.8 s | 13.7 s | correct | 1.16 m | no anchor (480 s) |

(b) holds: every anchor is correct by v2 and no out-of-area run anchored. (c) holds: pose lag
p95 ≤ 0.061 s and max ≤ 0.304 s in every run. **(a) fails**: 2012-02-04 anchored 82.9 s after
entering the store, and 2012-02-12 and 2012-04-29 (299 s and 157 s of data after entering) did
not anchor at all. **Under the preregistered rule the gate goes back to 0.18**, and
`--relocalize-anywhere` stays experimental.

What this says: on a new store and new sessions, a wider gate did not bring wrong anchors, but
it did not make anchoring prompt either. The development replay credited 0.26 mostly to one run
(2012-03-31); here three of six in-area runs were slow or never anchored with it. Why they did
not anchor (no candidate under the gate, or candidates that registration rejected) is not known
from this run and would be a post-hoc question on data that is now used.

#### Why the gate-0.26 check failed (post hoc, 2026-11-23)

Not preregistered; these sessions are used now. Three in-area runs were recorded again with every
keyframe's search input (`record_anywhere_queries.py`) and diagnosed
(`registration_diagnosis.py`: the session placed in the store by ICP from the ground truth;
`anywhere_gate_posthoc.jsonl`):

- **Registration is not what fails.** From its true pose, `match_with_reference` verifies 33 of
  36, 9 of 12 and 118 of 120 sampled in-store keyframes (2012-02-12, 2012-04-29, 2012-02-04), and
  most in-store keyframes with a candidate under 0.18 have a correct one (within 3 m and 20°).
- **The real-time search with the wide gate is.** The same inputs replayed in line anchor early,
  and the shipped gate in real time does too:

  | Session | Check (0.26, real time) | In line, 0.26 | In line, 0.18 | Real time, 0.18 |
  | --- | ---: | ---: | ---: | ---: |
  | 2012-02-04 | 83.0 s | 3.4 s | 3.4 s | 6.8 s |
  | 2012-02-12 | none | 77.1 s | 86.1 s | 89.4 s |
  | 2012-04-29 | none | 219.2 s | 226.0 s | — |

  At 0.26 a search registers several times as many candidates (development: 4–25×), each taking
  seconds; with the search on the worker the next search waits for it, so in real time far
  fewer keyframes are searched. 2012-02-12 is in the store only briefly (about 73–90 s, then from
  about 298 s), and two agreeing searches did not fit. The wider gate made the real-time mode
  worse, not better; the development replay, in line, could not show this.
- **"Entering the store" overstated the time in it.** 2012-04-29 first comes within 5 m of the
  store's path at 70.4 s but is in the store for only a few seconds then and again from about
  207 s to its end (228 s); its in-line anchor at 226 s is late by the rule, but there was little
  to anchor on. A future rule should count seconds of data in the store, not seconds since first
  entering it.

The gate stays at 0.18. What limits `--relocalize-anywhere` live is search cost per keyframe in
real time, which a later change could bound (for example, register only the closest few
candidates per search), checked in real time.

#### Replaying the real-time search (post hoc, 2026-11-23)

`gate_study.py --realtime` replays the background search of `live --realtime`: a search started
at a keyframe ends its measured seconds later in data time (`--slowdown` scales that), keyframes
before then are not searched, and its result is taken at the first keyframe after it ends.
`--caps` registers at most that many candidates per search. On the three recorded runs it gives
the real-time runs' anchor times (`anywhere_realtime_cap_replay.jsonl`, slowdown 1):

| Session | Real time, 0.18 (run / replay) | Real time, 0.26 (check / replay) | 0.26, at most 3 / 2 candidates |
| --- | ---: | ---: | ---: |
| 2012-02-04 | 6.8 s / 5.8 s | 83.0 s / 83.0 s | 12.5 s / 11.2 s |
| 2012-02-12 | 89.4 s / 89.4 s | none / none | 313.0 s / 307.5 s |
| 2012-04-29 | — / none | none / none | none / none |

- At 0.18 the cap changes nothing: few candidates pass the gate, a search takes 0.05 s (median),
  and the real-time search behaves like the in-line one.
- At 0.26 a search took 8–28 s (median) and up to 90 s with eight candidates. Capping at two or
  three shortens it but does not bring back what the in-line search found (2012-02-12: 77 s in
  line, 307 s or more capped).
- **A single registration takes 4–7 s** here, on repeated calls too (it is not a one-time
  preparation of the stored keyframe; preparing the query takes 0.04 s). This, not the number of
  candidates, is what makes a wider gate unaffordable in real time.

So capping candidates is not worth a product change. The lever is the cost of one registration
(GICP against the stored keyframe windows, up to three neighbours, three stages, then
confirmation), which can be measured directly and compared with identical verification
outcomes on development data before any gate is widened again.

### Cheaper registration for startup relocalization (development, 2026-11-24)

A profile of `match_with_reference` put about 94% of the time in GICP. `registration_bench.py`
writes `matchPrepared` again with the small_gicp Python bindings (same thinning, preparation,
neighbours, stages, overlap and confirmation; on the first keyframes it gives the C++ verdicts
and poses exactly) and runs variants from the same starts: every sixth recorded keyframe of nine
sessions on five stores, its two or three closest Scan Context candidates
(`anywhere_registration_iterations.jsonl`). A verified registration took 0.46 s, a rejected one
6.5 s: candidates from wrong places do not converge and run all 30 iterations of each stage.

| Variant | Faster | Verified ones lost | Rejected ones verified | Pose > 0.1 m from C++ |
| --- | ---: | ---: | ---: | ---: |
| 10 iterations per stage | 2.5× | 0 of 374 | 0 of 1042 | 0 |
| 5 iterations per stage | 4.3× | 1 of 364 | 0 | 0 |
| 10 iterations, 2 neighbours | 3.7× | 9 of 364 | 0 | 0 |
| coarse first stage (1.0 m voxels) | 1.4× | 1 of 95 | 0 | 0 |
| 4 threads | 1.4–1.6× | 0 of 95 | 0 | 0 |

`StartupRelocalizer` now runs 10 iterations per stage (the offline multi-session join keeps 30):
the same verdicts on all 1416 registrations, 2.5 times faster.

Real-time replays with the new core (`gate_study.py --realtime`, slowdown 1): at the shipped gate
(0.18) searches are short already (median 0.05–2.3 s) and the anchor times of the three runs with
earlier real-time numbers did not change (2012-02-04 5.8 s, 2012-02-12 89.4 s, 2012-04-29 none).
At 0.26 searches got shorter (2012-02-04 anchors after 28.9 s instead of 83.0 s; 2012-02-12 after
305.2 s instead of never), but 0.26 is still slower than 0.18 in real time on seven of the nine
sessions, so the gate stays at 0.18. The change cuts the cost of every relocalization
registration without changing its outcome; it does not by itself make anchoring earlier.
