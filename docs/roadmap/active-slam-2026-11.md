# Active SLAM: choose where to go next from the SLAM's own uncertainty (2026-11)

Purpose (maintainer, 2026-11-26): explore an unknown place autonomously and build an accurate
map of it. Success is, for the same distance (or time) budget, more of the place covered and an
accurate map (ATE, map quality) at once, against exploring by the nearest frontier alone.

Goal: a robot that maps a place decides where to drive next from what the SLAM knows about
itself: where the map is still unobserved, where the trajectory is weakly constrained, and where a
revisit would close a loop. The SLAM proposes goals and scores them; driving there is left to an
existing navigation stack (Nav2). This repository gains no motion planner or controller.

Status: P1 done except the live topic; P2/P3 first version done (`active_exploration.hpp`,
below, P3 "Done"); P4/P5 not started.

Decisions (maintainer, 2026-11-26):

- The uncertainty and candidate logic, its messages, and the goal-selection node live in this
  repository. The goal-selection node is an opt-in package, not part of the default launch path.
- Closed-loop evaluation runs in Robot Native Engine (RNE, `rsasaki0109/RobotNativeEngine`), not
  Gazebo. RNE depends on this repository (as an external process over ROS 2); this repository
  never depends on RNE.
- The project still has no robot of its own: open-loop parts are checked on public bags,
  closed-loop parts only in simulation.

## Where the project is today

| Need | What exists |
| --- | --- |
| Weakly constrained directions | `localizability_analysis.hpp`, `persistent_weak_direction.hpp`, `odometry_covariance_localizability.hpp`, `degeneracy_aware_solve.hpp` |
| Revisit and loop candidates | `trajectory_revisit_segmentation.hpp`, `loop_search_schedule.hpp`, `scan_context.hpp` |
| Observed / unobserved space | `probabilistic_surfel_map.hpp`, `map_change_evidence.hpp` |
| Map quality | `map_quality_metrics.hpp`, `scripts/run_map_quality_check.sh` |
| ROS-free engine boundary | `GraphSlamApplication` (`docs/roadmap/ros-free-core-2026-11.md`) |

Missing: (1) a pose-graph uncertainty measure exposed outside the backend, (2) goal candidates,
(3) a utility that compares them, (4) a closed-loop evaluation.

## Where the code goes

| Part | Location |
| --- | --- |
| Uncertainty, coverage, candidate generation, utility (P1-P3) | `graph_based_slam/include/graph_based_slam/` as ROS-free headers (same shape as `localizability_analysis.hpp`), movable to `lidarslam_core` later |
| Messages | `lidarslam_msgs`: `SlamUncertainty`, `ActiveGoalCandidates` |
| Goal selection and Nav2 client (P4) | new package `lidarslam_active/` (C++), opt-in, excluded from the default launch files |
| Worlds, scenarios, closed-loop runner, metrics (P5) | RNE: an example/scenario using `adapters/ros2/rne_ros2_bridge`, not an RNE engine crate |

```
RNE scenario  --ROS 2-->  rko_lio + graph_based_slam (+ lidarslam_active)  --NavigateToPose-->  Nav2  --cmd_vel-->  RNE robot
```

`graph_based_slam` only publishes the two messages, off by default; it does not know whether
anything consumes them. RNE's own `rne_slam` is not used to drive the robot in these runs; it may
appear as a comparison only.

## Phases

### P0. Preregistration

This file is completed with the evaluation thresholds (below) before any closed-loop run.

### P1. Uncertainty output (open loop, no robot motion)

- Per keyframe: a pose uncertainty measure. Candidates: marginal covariance from the pose graph,
  or the D-optimality of the graph (log-determinant of the reduced weighted Laplacian). Pick one
  by cost on the largest existing bags.
- Per submap: localizability category and weak directions (existing analysis, exposed).
- Coverage: observed / free / unknown voxels and frontier clusters from the surfel evidence.
- Outputs: `SlamUncertainty` topic and CSV from the offline runner.
- Gates: byte-identical N-run outputs on the existing MID360 and HILTI bags (as
  `run_offline_determinism_check.sh`); existing outputs unchanged with the feature off.

This phase is useful on its own: it shows where a finished map is weak.

Done (2026-11-26): `pose_graph_uncertainty_report` (`pose_graph_uncertainty.hpp`). It reads the
map bundle's `pose_graph.g2o`, linearizes each `EDGE_SE3:QUAT` at the optimized poses as g2o's
`EdgeSE3` does, and writes per-vertex marginal covariance summaries (CSV) and the graph's log
det / D-optimality (YAML). Choice of measure: full 6-DoF marginals, not a translation-only
Laplacian, because heading error is what makes the far end of a corridor uncertain sideways
(test `HeadingErrorMakesTheWeakDirectionLateral`). Checked against g2o `computeMarginals` on a
40-vertex graph with rotations and three loops: 6x6 blocks within 3e-5 relative. Limits: the
sigmas are in the units of the configured edge information weights, not calibrated metres;
robust kernels are not in the g2o text; the backend's gravity-prior and plane-revisit edges are
not registered with g2o, so g2o does not write them and the report cannot see or count them. With
the default parameters (both off) the saved graph is complete; with either on (e.g. the RTK-GNSS
profile's `gravity_prior_info: 1000`), the report overstates the uncertainty they remove. On a
60-vertex graph from `optimizePoseGraph` with 12 gravity priors, the file held only the 286
SE3 edges and every vertex's weakest direction came out vertical. The in-backend computation
(next step) sees every edge and removes this limit. Cost: one sparse Cholesky plus 6 solves per vertex.

Done (2026-11-26): the backend computes the uncertainty itself.
`pose_graph::optimizePoseGraph(..., compute_marginals)` re-linearizes the graph at the optimized
poses and returns each submap's marginal covariance from g2o's `computeMarginals`, so every edge
the optimization used counts (GNSS anchors, gravity priors, plane revisit, robust-kernel
weights), which removes the limit of the file report. Off by default; with it on the poses and
the saved `pose_graph.g2o` are byte-identical (test `OffByDefaultAndPosesUnchanged`). On an
SE3-only graph its marginals match the file report within 1e-3 relative; with gravity priors
the far end's vertical variance drops by more than half. The offline pipeline
(`graph_slam_offline_runner`, `lidarslam` core) gains three report-only parameters:

- `pose_uncertainty_output`: `pose_uncertainty.csv` / `pose_uncertainty_summary.yaml` of the
  final graph.
- `pose_uncertainty_timeline` (+ `pose_uncertainty_timeline_every_n_submaps`, default 1):
  `pose_uncertainty_timeline.csv`, the uncertainty of the graph as it stood during the pass
  (the submaps so far and the loop edges found so far, optimized after each new submap and each
  change of the loop edges), and `pose_uncertainty_timeline_loops.csv`, the loop edges of each
  of those states. In-line loop search only, so deterministic. The timeline graph has the
  odometry, loop and gravity-prior edges, not GNSS anchors or plane revisit (only the final
  graph has those).

On the synthetic two-lap room (`lidarslam_core/test/test_python_backend.py`) every other output
is byte-identical with the parameters on and off, two runs give the same bytes, and when the
second lap first closes a loop (to the fixed start) the newest submap's sigma drops from 0.23 to
0.003. Later loop edges there are to submaps several metres away, so the newest submap keeps
that submap's heading uncertainty times the distance (0.17 at 4 m): the 6-DoF marginals show the
lever arm. The loop edges also change during the pass (the dedup window keeps the newest edge
of a revisit: 0-13, 1-14, 3-16, 5-20, then 12-25), which the final `loop_edges.csv` cannot show;
hence the per-state loop file.

`tools/readme_media/render_pose_uncertainty.py` renders a timeline as the interim GIF (Demo GIF
below).

Done (2026-11-26): the interim GIF, `lidarslam/images/pose_uncertainty_mid360_drive.gif`, on
the public MID-360 drive of the Docker first-map demo (1.1 km, one block and back to the start;
`docs/readme-media.md` has the commands and numbers). The newest submap's sigma climbs to 74
and drops to 1.4 when the loop closes. Two full runs (RKO-LIO and backend) gave byte-identical
timelines and the same GIF bytes; with the parameters off, the run's other 379 files (map
tiles, trajectories, `pose_graph.g2o`) were byte-identical. Data tried and not used: the README's `outdoor_hard_02a` and
the other Koide outdoor sequences close no loop; the Koide indoor sequences are an Azure Kinect
depth camera, not a LiDAR; NTU VIRAL (9 loop edges on `tnp_01`) was down for maintenance;
the HILTI 2022 bucket refused the download here.

Next in P1: the same timeline in the live node with the `SlamUncertainty` message; then
coverage/frontiers from the surfel evidence.

### P2. Goal candidates

- Explore: frontier clusters, reachable side by the 2D projection used for navigation.
- Revisit: for a trajectory segment whose uncertainty grew, poses near earlier keyframes where a
  loop is likely (revisit segmentation + Scan Context distance).
- De-degenerate: poses from which structure constraining the current weak direction is visible
  (e.g. the end wall of a long corridor).

### P3. Utility and mode switching

- `utility = expected map entropy reduction + expected D-opt gain of the predicted loop edge
  - lambda * path cost` (path cost from the Nav2 planner).
- Mode switch explore -> relocalize when the uncertainty exceeds a threshold, back with
  hysteresis.
- Pure functions, unit-tested with synthetic graphs (`synthetic_degeneracy_fixtures.hpp`).

Done (2026-11-26), first version of P2 and P3: `active_exploration.hpp` (ROS-free, Eigen only).

- Coverage grid: each submap scan's points cast as rays from the sensor onto the ground plane
  (ground points free the whole ray; points in an obstacle height band mark a hit; higher
  points are ignored), hit / pass counts per cell, growing as scans arrive.
- Path lengths: Dijkstra on free cells, obstacles inflated (not around the robot itself).
- Frontiers: free cells next to unknown ones, 8-connected clusters; goal = the reachable
  frontier cell nearest the centroid; gain = unknown area within `gain_radius`.
- Revisit targets (in revisit mode only): submaps at least `min_index_gap` older than the newest,
  with sigma at most `anchor_sigma_ratio` x the newest's, at least `min_revisit_distance` away
  and reachable; gain = newest sigma minus theirs.
- Selection: `info_weight * unknown area + uncertainty_weight * sigma drop - distance_weight *
  path length`; revisit mode from `revisit_trigger_sigma`, back below `revisit_release_sigma`.
- Tests (`test_active_exploration.cpp`, synthetic 2D LiDAR rooms): walls / free / unknown, a
  closed room is explored, a doorway is the frontier, paths go around walls, the closer of two
  equal frontiers wins, revisit with hysteresis, the grid grows without losing cells,
  deterministic decisions.
- Replay on recorded runs: `active_exploration_timeline` (with `pose_uncertainty_timeline`)
  writes `active_exploration_timeline.csv`, the goal it would choose at every graph state
  (`exploration_*` parameters). On the 1.1 km MID-360 drive (trigger 30, release 10, the other
  settings default): it explores by frontiers about 20 m away while sigma < 30 (274 states),
  then asks to revisit (362 states); 40 m before the recorded car reaches its start it asks to
  go back to submaps 12-13 at the start, and when the recorded loop closes (sigma 74 -> 1.4) it
  returns to exploring. Two replays give the same bytes, and with the parameter off the
run's other outputs are byte-identical. (These are the first version's decisions. With the second round below,
revisits compete by utility and, at the default revisit weight 1, the same replay chooses a
frontier in all 636 states, 362 of them in revisit mode with 87-334 revisit candidates; it takes
501 s instead of 126 s, as each state now computes the cross covariances with the newest, O(N^2)
over a run.) The recorded car did not follow the
  goals, so the revisit paths grow to 500 m: the replay checks the decisions on real scans, it
  does not evaluate exploration (P5 does).

Known limits of this version, next in P3:

- The revisit gain is the sigma difference to the target, a stand-in. It prefers recent targets
  with half the newest sigma over the well-known start far away. Replace it by the plan's
  measure: add the predicted loop edge to the graph and compute the newest sigma (or log det)
  it would give.
- The sigma thresholds are in the units of the edge information weights, so they are set per
  parameter set (30 / 10 on the MID-360 preset; 0.15 / 0.05 on the synthetic room).
- The grid uses each submap's odometry pose as created; after a loop closure it is not moved
  to the optimized poses.

### P4. `lidarslam_active` node

- Subscribes to `SlamUncertainty` and `ActiveGoalCandidates`, asks Nav2 for path costs, sends
  `NavigateToPose`, re-plans on goal reached / failed / timeout.
- Opt-in launch file only. Humble and Jazzy builds in CI; no Nav2 dependency for any other
  package.

### P5. Closed-loop evaluation in RNE

- Robot and sensor: Go2 with RNE's Livox Mid-360 model (fitted to real Go2 recordings,
  `docs/LIVOX_MID360.md` in RNE). Matches the sensor this project is tuned on and
  `docs/research/go2-rko-integration-2026-09.md`.
- First task: **lockstep determinism over ROS 2.** RNE publishes `/clock` and advances only when
  the SLAM has consumed the frame; two runs with the same seed must give the same trajectory
  bytes. If this cannot be reached, the evaluation falls back to N seeds and reports spread, and
  this file says so.
- Worlds: an office with rooms, a long featureless corridor (degenerate), a warehouse with loops.
- Baselines: nearest frontier, random frontier, the proposed utility.

Done (2026-11-26), first closed loop: `tools/rne_active_slam_env` (prototype kept in this
repository while it settles; RNE write access was not available to this work).

- Changes from the plan above: a kinematic differential-drive robot instead of the Go2 (no
  locomotion error mixed into the SLAM), and no ROS 2 / Nav2: the env is a Rust program
  (RNE's `sample_livox_mid360` against an analytic box world, an IMU with noise and bias) driven
  over stdin / stdout in lockstep, and the loop is one Python process with
  `lidarslam.live.Session` (new: `decide_exploration`, `exploration_grid`) and a pure-pursuit
  controller on the SLAM's own odometry pose. Determinism: two runs of each policy gave
  byte-identical `frames.csv` and decisions.
- Checks of the simulated data: points moved to the world with the true pose at each point's
  own time all land on the world's surfaces (80-88 % with one pose per frame while turning at
  0.8 rad/s). The messages follow `livox_ros_driver2` (the header carries the first point's time:
  RKO-LIO's timestamp check rejects a header 2 ms earlier).
- What the simulation showed before any comparison:
  - RKO-LIO presets: in place turns at the stop of a corridor drive slide the car preset 2.7 m
    and the handheld-outdoor preset 1.0 m along the corridor; the indoor (walking) preset holds
    (0.24 m at most, 0.03 m after). The walking preset is the default, with
    `imu_acceleration_unit: auto` (the env sends g like the real driver; the handheld preset read
    1 g as 1 m/s^2 and estimated an 8.8 m/s^2 bias).
  - Flat walls alone leave a corridor's axis unobservable; the `ring` world has pilasters every
    3 m and crates.
  - Explorer fixes found in the loop: the floor under a LiDAR is never seen, so the cells around
    the sensor are marked free (`clear_radius`) and frontiers closer than `min_goal_distance` are
    not goals; a frontier's gain counts only unknown cells in line of sight (the unknown behind a
    wall made a shadow behind a crate the best goal); a frontier still there when the robot has
    come within `visited_radius` is marked visited.
- First comparison (`ring`: a 5 m corridor around a 30 x 14 m block, ~100 m loop; one seed;
  400 s budget; neither policy finished):

  | | nearest frontier | active (trigger 1.5, release 0.5) |
  | --- | --- | --- |
  | floor coverage | 72.7 % | 76.5 % |
  | distance | 144 m | 216 m |
  | live position error RMSE / max | 0.52 / 0.63 m | 0.16 / 0.54 m |
  | loop edges | 0 | 2 (back at the start) |
  | frames in collision | 138 | 0 |

  One world and one seed: an observation, not the preregistered evaluation (below).
- Seen in the active run: about 130 s in revisit mode chasing revisit targets 3-4 m away where
  no loop edge was accepted; sigma stayed between release and trigger, so the mode did not end.
  This is the stand-in revisit gain (P3 limits): it rates a close target with half the sigma above
  the well-known start. Next: the predicted-loop-edge gain, revisit targets the robot reached
  without a loop marked visited (as frontiers), then the preregistered evaluation (several worlds
  and seeds).

Done (2026-11-26), second round: the predicted revisit gain, and what the loop showed next.

- Predicted gain: `optimizePoseGraph` also returns Cov(x_i, x_newest) for every submap; from the
  joint covariance of k and the newest, `pose_graph_uncertainty::predictedCovarianceAfterLoop`
  gives the newest submap's covariance after a loop edge k -> newest (one Gaussian update,
  Woodbury). Test: equal to re-solving the graph with that edge within 2e-3 relative. A revisit's
  gain is now the predicted fraction of the newest sigma removed (unitless; the weight carries
  over between parameter sets), with `exploration_min_predicted_drop` (0.2) and
  `exploration_predicted_loop_fitness` (0.2, the information of the predicted edge).
- Revisit targets the robot reached in revisit mode without a loop edge are tried and dropped
  until the mode ends (`exploration_revisit_reached_radius` 2 m, `_tried_radius` 3 m).
- Revisit targets must be farther back in travel than the loop search's `distance_loop_closure`
  (set from it): the car preset searches loops 100 m of travel apart, the walking preset 60 m;
  a revisit 30 m back cannot close a loop, and the first runs chased exactly those.
- In revisit mode frontiers and revisits compete by utility (before, any revisit won).
- The simulation now uses the indoor graph preset too (`lidarslam_mid360_rko_graph_walking`),
  with `exploration_inflation_radius` 0.8 m (0.6 m let the SLAM's 0.5 m side error put the robot
  into a block corner, repeatedly) and a goal rejection after 3 collisions on one goal
  (`Session.reject_exploration_goal`, as a navigation stack aborts a goal). Sigma on this preset
  is ~25 times smaller than on the car preset (0.09 after 80 m vs 2.3): the triggers are 0.05 /
  0.02 here.
- Runs (one seed each, 400 s; `grid`: 3 m corridors around a 3 x 2 grid of blocks, 57 x 35 m):

  | world | policy | coverage | distance | live error RMSE / max | loop edges |
  | --- | --- | --- | --- | --- | --- |
  | ring | active | 99.3 % | 175 m | 0.16 / 0.34 m | 2 |
  | ring | nearest frontier | 99.95 % (done at 377 s) | 201 m | 0.25 / 0.44 m | 2 |
  | grid | active | 84.7 % | 246 m | 0.22 / 0.54 m | 1 |
  | grid | nearest frontier | 74.3 % | 225 m | 0.12 / 0.33 m | 1 |

  **The active runs never chose a revisit** (0 frames in revisit goals): at these weights a
  frontier ahead always had the higher utility. The differences above come from the frontier
  choice alone (the active policy counts the unknown area, the baseline only the distance), not
  from uncertainty-driven revisits, and one seed per cell does not separate them from chance.
- Next: (1) preregister the evaluation (P0): worlds, seeds (start pose and noise), budget,
  metrics and the success rule, fixed before the runs; (2) an ablation that isolates the
  revisits (frontier utility with and without revisits), and worlds where exploring forward does
  not close loops by itself (long out-and-back branches, a large loop with drift); (3) calibrate
  the revisit weight against the frontier utility on development seeds only.

## Demo GIF (the deliverable that shows the result)

Recorded once P5 has a result, from the same runs the evaluation counts (not a separate
hand-picked run), and documented in `docs/readme-media.md` like the other README media (data,
commands, commit, metrics, render command).

- **Final GIF (closed loop, RNE):** one RNE world, the same seed, side by side: nearest-frontier
  baseline vs the proposed method, Go2 with the Mid-360 model. Each panel shows the map as it
  is built, the trajectory coloured by the per-vertex uncertainty (relative scale, labelled as
  such), the goal candidates by kind (explore / revisit / de-degenerate) with the chosen one
  highlighted, and the moment a revisit closes a loop and the colour drops. Underneath: ATE and
  coverage against distance driven for both. Rendered by RNE's wgpu renderer with its metadata
  JSON, and labelled "simulation".
- **Interim GIF (open loop, after P1 is in the backend):** a public bag (e.g. the Koide
  MID-360 sequence already used for the README) replayed, trajectory coloured by uncertainty
  after each optimization: it grows along the path and drops at loop closures. Needs the
  per-optimization uncertainty output (P1 next step); the current report reads only the final
  graph, so it cannot animate.

## Evaluation (P0, approved by the maintainer 2026-11-26)

Fixed before any holdout run; changed only by a dated note in this file, before those runs.

### Question

Does choosing revisits from the SLAM's own uncertainty make the map more accurate than
exploring alone, without costing coverage? The revisits are isolated by an ablation: the
`active` policy is the `frontier_info` policy plus revisits, everything else equal.

### Policies

| Policy | Frontier utility | Revisits |
| --- | --- | --- |
| `nearest_frontier` (baseline) | `- 1.0 x path length` | no |
| `frontier_info` (ablation) | `1.0 x visible unknown area - 0.5 x path length` | no |
| `active` (proposed) | as `frontier_info` | yes: `w_u x predicted sigma fraction removed - 0.5 x path length`, in revisit mode only |

### Worlds

- Development (already run, may be looked at and tuned on): `ring` (100 m loop around a block),
  `grid` (3 m corridors around a 3 x 2 grid of blocks).
- Holdout (defined here, never run before the freeze):
  - `branches`: a 60 m main corridor with six 15-25 m dead-end side corridors, some ending in a
    room: exploring means going out and back; the only loop is the main corridor's end joined
    back to its start through one long return corridor.
  - `big_loop`: a 3 m corridor around a 70 m x 40 m block (~220 m loop) with sparse features
    (pilasters every 6 m), so drift grows before the loop closes by itself.
- All worlds: wall height 2.5 m, floor box, crates and pilasters as in the development worlds;
  the robot, sensor, IMU noise model and controller as in `tools/rne_active_slam_env`.

### Seeds

A seed sets the start pose (one of 10 fixed free-space poses per world, with heading) and the
noise seeds of the LiDAR model and the IMU. Development: seeds 1-3. Holdout: seeds 101-110. A
run is deterministic given (world, seed, policy, commit).

### Budget and stop

600 s of simulated time, or earlier when the explorer reports `exploration_done` three times in a
row. A run also stops as failed when the live position error exceeds 2 m (SLAM diverged).

### Metrics (per run)

- Primary accuracy: **final-map ATE**: RMSE of the optimized submap positions
  (`trajectory_optimized.tum`) against the true positions at the same stamps, after an SE(2)
  alignment (the odometry frame starts at the start pose; the alignment only removes the start
  error).
- Primary coverage: **floor coverage at the stop** (cells with a floor return / free floor cells,
  0.5 m cells, from the true poses).
- Secondary (reported, not judged): live position error RMSE and max; time and distance to 90 %
  coverage; distance driven; loop edges; revisit decisions taken; collision frames, rejected
  goals; map MME and plane thickness of `map.pcd` (`run_map_quality_check.sh`).
- Failure: diverged (above), or coverage below 50 % at 600 s.

### Success rule (holdout only, `active` vs `frontier_info`, paired by seed)

Per holdout world, all of:

1. final-map ATE: median paired reduction of at least 15 %, and `active` lower in at least 7 of
   the 10 seeds;
2. coverage at the stop: median paired difference no worse than -5 percentage points;
3. failures: no more failed runs than `frontier_info`.

The method passes if both holdout worlds pass. `active` vs `nearest_frontier` is reported with
the same statistics but does not decide. Results are reported whatever they are, failures
included.

### What may be tuned, and where

On development seeds and worlds only: the revisit weight `w_u`, the revisit trigger and release
(per parameter preset, as sigma's scale depends on it), `min_predicted_drop`, the inflation
radius. Frozen in `tools/rne_active_slam_env/eval_config.yaml` and its commit recorded here
before the first holdout run; the holdout worlds are run once at that commit.

### Development and freeze (2026-11-26)

Development set (ring, grid; seeds 1-3), `active` vs `frontier_info`, by revisit weight
`w_u` (trigger 0.05, release 0.02, `min_predicted_drop` 0.2, inflation 0.8 m):

| `w_u` | ring: median ATE reduction, wins | grid: median ATE reduction, wins, coverage |
| --- | --- | --- |
| 20 | -2 %, 0/3 (revisits chosen in 2 runs; one run diverged, below) | 0 %, 0/3 (no revisit chosen) |
| 40 | -2 %, 0/3 | 0 %, 0/3 |
| 80 | +1.4 %, 2/3 | +16 %, 2/3, -1.3 points |

Development findings and changes before the freeze: (1) a revisit's utility at weight 20 never
beat a frontier's (unknown area ~20-25 m^2); (2) in revisit mode with no reachable target the
robot stood still until the budget ran out, and on ring seed 3 its odometry drifted 2.2 m while
standing: `no_reachable_goal` now ends the mission like `exploration_done`; (3) the
development maps are already accurate (final-map ATE 2-8 cm on ring), so they say little about
the revisits' benefit.

**Frozen:** `w_u` = 80, the rest as above; `eval_config.yaml` at commit `1127155`
(SHA-256 prefix `8c1ee164b521f6c8`). The holdout set is run once at that configuration.

### Holdout result (2026-11-27): fail, and the holdout was invalid

Run once at commit `aa85121` (configuration as frozen, SHA-256 `8c1ee164...`), 2 worlds x 10
seeds x 3 policies:

| world | policy | failed | median final-map ATE | median coverage |
| --- | --- | --- | --- | --- |
| branches | active | 9/10 | 0.026 m | 16.8 % |
| branches | frontier_info | 9/10 | 0.026 m | 16.8 % |
| branches | nearest_frontier | 9/10 | 0.023 m | 18.5 % |
| big_loop | active | 10/10 | 0.149 m | 17.7 % |
| big_loop | frontier_info | 10/10 | 0.149 m | 17.7 % |
| big_loop | nearest_frontier | 10/10 | 0.141 m | 23.3 % |

By the rule: **fail** in both worlds (`active` lower in 0/9 and 1/10 seeds; 9 vs 9 and 10 vs 10
failures).

Why, found after the run: 57 of the 60 runs ended with `exploration_done` after 5-120 s (3
diverged in big_loop). The holdout worlds put crates (and 0.2 m pilasters) inside 3 m
corridors; with the planner's 0.8 m inflation they cut the worlds into pieces: from the ten
starts only 8-34 % (branches) and 19-42 % (big_loop) of the floor the robot can stand on is
reachable (`worlds.reachable_fractions`; the development worlds give 100 % from every start). The
explorer correctly reported nothing reachable left. The checks before the run looked only at the
start poses' clearance. So this holdout says nothing about the method: all three policies
explored the same small piece, and `active` chose a revisit in 3 of 20 runs.

Kept as the record of this holdout; it is not re-run, and these two worlds are now seen.
`reachable_fractions` must give 1.0 from every start for a world to be used
(`test_active_slam_eval.py` checks the development worlds).

Proposed amendment (needs the maintainer's approval before anything runs): two new holdout
worlds of the same two kinds (dead-end branches; a long loop with sparse features), built and
checked with `reachable_fractions` and nothing else, then run once at the frozen configuration
(`w_u` 80; the stop rule and everything else unchanged).

### To build before the runs

`--seed` in the harness (start pose and noise seeds), the two holdout worlds, the final-map ATE
(truth by stamp), the `frontier_info` policy, an `eval` driver that runs a world x seed x policy
table and writes the summary, and `eval_config.yaml`.

## Risks

- Simulation-to-real gap: the RNE Mid-360 model reduces it but does not remove it; no real
  closed-loop result is claimed.
- Cost of marginal covariance on large graphs: approximate (local subgraph or sparsified) if P1
  measurements require it.
- Determinism over ROS 2 may not hold (see P5); the byte-identical gates apply to P1 only until it
  does.
