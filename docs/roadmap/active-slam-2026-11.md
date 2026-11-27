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

Amendment (approved by the maintainer 2026-11-27): two new holdout worlds of the same two
kinds, built and checked with `reachable_fractions` and the start clearance only (no policy was
run on them), then run once at the frozen configuration (`w_u` 80; the stop rule and everything
else unchanged):

- `branches2`: 4 m corridors (70 x 50 m): a main corridor with four dead ends up (two into
  rooms) and two down, and one loop through a return corridor; crates in the rooms only.
- `long_loop2`: a 4 m corridor around a 66 x 36 m block (~220 m loop), pilasters every 6 m, no
  crates.
- Checks: every start reaches 100 % of the floor the robot can stand on under the 0.8 m
  inflation; the start clearance is 1.8 m or more.
- `eval_config.yaml` changes only its holdout world list (SHA-256 prefix `2b471555c417f025`); the first
  holdout worlds are kept as `RETIRED` in `worlds.py`.

### Second holdout result (2026-11-27): fail

Run once at commit `c6a50ee`, config SHA-256 prefix `2b471555c417f025`, clean tree; 60 runs,
nothing rerun or excluded. Judged by the success rule above:

| world | median paired ATE reduction | active lower | median coverage diff | failed (active / frontier_info) | rule |
| --- | --- | --- | --- | --- | --- |
| `branches2` | 7.9 % (need >= 15 %) | 6 / 10 (need 7) | -2.8 points | 2 / 2 | fail |
| `long_loop2` | 3.7 % (need >= 15 %) | 5 / 10 (need 7) | -0.7 points | 2 / 3 | fail |

Median final-map ATE (active / frontier_info / nearest_frontier): `branches2` 0.095 / 0.099 /
0.036 m, `long_loop2` 0.132 / 0.176 / 0.148 m. The pass/fail verdict above is the result; what
follows is post-hoc reading, not a claim:

- The direction is right but the size is not: in the seeds where `active` revisited and neither
  run failed, it lowered the ATE in 6 of 8 (`branches2`, median 11 %) and 4 of 7 (`long_loop2`,
  median 7 %), at a cost of 2-4 coverage points.
- `nearest_frontier`, which ignores uncertainty, had by far the lowest median ATE in
  `branches2`. The baseline the rule compares against (`frontier_info`) drives long information-
  rich legs; `active`'s revisits recover part of what that choice costs, not more.
- Failures not caused by the policy: seed 104 in `branches2` ended after 15 s for all three
  policies with `exploration_done` while one frontier was left but unreachable on the coverage
  grid (the stop rule treats "no reachable frontier" as done); seed 101 in `branches2` ended
  with 30+ collisions at about 190 s for all three. 6 of the 7 `long_loop2` failures are the
  odometry passing the 2 m divergence limit in the ~220 m loop before a loop closed, and in
  seeds 101 and 109 `active` never revisited, so it ran identically to `frontier_info`.
- The static checks (reachability, start clearance) did not catch the two `branches2` start
  failures; a short dry run of each start would have.

This closes the holdout for this configuration. Any change (stop rule, planner, a revisit
trigger tied to the travel distance rather than sigma alone) needs new development work and a
new, untouched holdout before a claim is made.

### Sensor bug found after the second holdout (2026-11-27)

Looking into the 15-second `branches2` seed 104 runs showed that the simulated Mid-360 saw only
the robot's left: RNE's sensor frame is +X forward, +Y Livox up, +Z right, and the env rotated
it as if it were z-up, so the sensor lay on its side. Every closed-loop number above (the
development tuning, the `w_u` freeze and both holdouts) was measured with it; they are kept as
the record but say nothing about the method with a working sensor. Fixed in the env (yaw about
+Y, points converted to the Livox frame) with cargo tests that both corridor walls and the end
wall appear where they should.

What follows from it:

- `branches2` and `long_loop2` have been run, so they join the development set (`ring`, `grid`,
  `branches2`, `long_loop2`); the next holdout must be new worlds, built, checked statically and
  with a short dry run from every start, and approved before it is run once.
- Development restarts from the frozen configuration with the fixed sensor, then the two
  changes the second holdout pointed at: the stop rule (frontiers that exist but are unreachable
  are not "done") and a revisit trigger tied to the travel since the last loop.

### Development baseline with the fixed sensor (2026-11-27)

The frozen configuration on the four development worlds, seeds 1-3, 36 runs: no run failed or
diverged, and every policy's median final-map ATE is 0.022-0.044 m (`ring` 0.023 / 0.022 /
0.024 m for active / frontier_info / nearest_frontier, `grid` 0.027 / 0.025 / 0.025, `branches2`
0.041 / 0.044 / 0.037, `long_loop2` 0.031 / 0.037 / 0.032). `active` revisits, closes more loops
(e.g. `grid` seed 2: 10 against 2) and covers as much or more, but the ATE barely moves: the
largest live error of any run is 0.39 m over 100-360 m of travel, so there is little drift for a
revisit to remove.

So the simulation, with a working sensor, is easier than the real thing: perfect boxes with
pilasters every few metres give RKO-LIO constraints in every direction, and drift is about
0.01 % of the distance. A test of active SLAM needs odometry that drifts like a real one. The
next step sets the env's difficulty without looking at any exploration policy: a fixed scripted
route per world, odometry alone, and sensor/IMU noise and scene features chosen so the drift
matches a stated target from real Mid-360 data; only then are the policies run again.

### Difficulty calibration: target and rule (written before the sweep's results)

Measured on fixed routes per development world (`worlds.ROUTES`, driven on the true pose; no
exploration policy runs), by `calibrate_difficulty.py`: RKO-LIO's mean translational drift over
50 m of travel (`odometry_drift_50m_pct`, odometry only, no loop closure).

- Target: median over the four development worlds of 0.3-1.0 % of the distance. Reference
  points: KITTI-class LiDAR odometry is about 1 % (the repo's KITTI 00 graph run: 1.02 %), and
  the real Mid-360 drive in this repo gives 0.19 % translational RPE with the graph backend
  (docs/research/phase7-plane-revisit-regression-2026-07.md), so odometry alone drifts more.
- Knobs: pilaster spacing (0 as built, 12, 24 m, none) x IMU noise scale (1, 5). The as-built
  ring already gives 0.18 % (two laps, one check run).
- Rule: the setting closest to as built (fewest pilasters removed first, then the lower IMU
  noise) whose median is in the band and with no world above 2 %. If none is, report that and
  do not run policies on a setting outside the band.
- The setting chosen applies to every world, development and holdout alike, and is frozen
  before any policy is run on it.

### Difficulty calibration: result (2026-11-27): no setting is in the band

`calibrate_difficulty.py`, seed 1, 32 routes of 210-256 m, no collision. Odometry drift over
50 m [%]:

| pilaster spacing [m] | IMU noise | ring | grid | branches2 | long_loop2 | median |
| --- | --- | --- | --- | --- | --- | --- |
| as built | 1 | 0.178 | 0.187 | 0.211 | 0.130 | 0.183 |
| as built | 5 | 0.178 | 0.209 | 0.206 | 0.145 | 0.192 |
| 12 | 1 | 0.181 | 0.207 | 0.179 | 0.134 | 0.180 |
| 12 | 5 | 0.178 | 0.220 | 0.188 | 0.143 | 0.183 |
| 24 | 1 | 0.176 | 0.207 | 0.204 | 0.130 | 0.190 |
| 24 | 5 | 0.181 | 0.227 | 0.192 | 0.142 | 0.187 |
| none | 1 | 0.173 | 0.248 | 0.191 | 0.145 | 0.182 |
| none | 5 | 0.180 | 0.279 | 0.180 | 0.160 | 0.180 |

By the rule, no setting reaches 0.3 %, so no policy is run on any of them. The drift barely
depends on either knob: without pilasters the corners, end walls and crates still constrain
RKO-LIO, and IMU noise five times larger hardly matters to a LiDAR-dominated estimate. Shifting
the truth by one frame either way raises the drift (ring: 0.44 / 0.19 / 0.64 % at -1 / 0 / +1),
so it is not a timing artifact of the metric. The simulated odometry drifts about as much as
the repo's real Mid-360 drive with its graph backend (0.19 %).

### Degenerate worlds: calibration rule (written before their results)

Real LIO drifts where the scene does not constrain it, so two development worlds at the scale
where that happens with a Mid-360 (70 m range): `tunnel_loop` (4 m tunnel, 164 m long sides,
~400 m loop) and `long_tunnels` (two 200 m tunnels joined at both ends and in the middle). In
the middle of a long side both end walls are out of range; only pilasters (built every 6 m)
fix the position along the tunnel.

- Measured as before: fixed routes (~400 m), odometry drift over 50 m, no policy.
- Knob: pilaster spacing 6 (as built), 12, 24, 48 m, none; IMU noise scale 1 (it did not
  matter).
- Rule: per tunnel world, the setting closest to as built whose drift is 0.3-1.0 % and whose
  route does not lose the odometry (live error under 5 m). If both worlds land in the band at
  one common spacing, that spacing applies to both; otherwise the per-world settings are
  recorded and used. The four earlier worlds keep their as-built scene (reported as the easy
  set).
- Frozen before any policy runs on the tunnels.

### Degenerate worlds: calibration result (2026-11-27)

Seed 1, routes of ~400 m, no collision. Drift over 50 m [%] (largest live error [m]):

| pilaster spacing [m] | tunnel_loop | long_tunnels |
| --- | --- | --- |
| 6 (as built) | 0.12 (0.37) | 0.13 (0.36) |
| 12 | 0.12 (0.73) | 0.13 (0.33) |
| 24 | 0.13 (0.55) | 0.14 (0.39) |
| 48 | 4.37 (6.04) | 0.53 (1.32) |
| none | 32.3 (58.6) | 8.02 (14.3) |

By the rule: `long_tunnels` at 48 m (0.53 %, in the band); `tunnel_loop` has no setting in the
band (24 m is below, 48 m above both limits). The drift jumps between 24 and 48 m.

Amendment, made after seeing this table and before any policy run on the tunnels: a finer
sweep of `tunnel_loop` at 30, 36 and 42 m under the same rule, and seeds 2 and 3 of
`long_tunnels` at 48 m to see whether one seed's 0.53 % holds. No policy result is involved.

### Degenerate worlds: frozen difficulty (2026-11-27)

Finer sweep: `tunnel_loop` 30 m 0.81 % (live error up to 1.45 m), 36 m 2.26 % (4.75 m), 42 m
4.95 % (10.8 m); `long_tunnels` at 48 m, seeds 2 and 3: 0.78 % and 0.49 % (seed 1: 0.53 %).
By the rule: `tunnel_loop` pilasters every 30 m, `long_tunnels` every 48 m (no common spacing
is in the band for both). Frozen in `eval_config.yaml` (`world_overrides`) before any policy
runs on the tunnels, with a 1500 s budget (a ~400 m tour takes ~580 s) and a 10 m divergence
limit (2 m is below what the odometry reaches on the routes). The tunnels are run as the set
`tunnels` (development, seeds 1-3); the policy configuration is still the earlier frozen one
(`w_u` 80, trigger 0.05), tuned with the buggy sensor, so this run is a baseline, not a test.

### Tunnels baseline (2026-11-27, development, not a test)

18 runs (2 worlds x seeds 1-3 x 3 policies), no failure. Final-map ATE [m] per seed:

| world / seed | active | frontier_info | nearest_frontier |
| --- | --- | --- | --- |
| tunnel_loop 1 | 0.297 | 0.297 | 0.355 |
| tunnel_loop 2 | 0.789 | 0.801 | 0.588 |
| tunnel_loop 3 | 0.070 | 0.585 | 0.179 |
| long_tunnels 1 | 0.496 | 0.589 | 0.615 |
| long_tunnels 2 | 0.419 | 0.901 | 1.471 |
| long_tunnels 3 | 0.724 | 0.724 | 0.953 |

Medians: `tunnel_loop` 0.297 / 0.585 / 0.355, `long_tunnels` 0.496 / 0.724 / 0.953; coverage
0.99 for all but `nearest_frontier` on `long_tunnels` (0.93). Where drift is real, `active` is
never worse than `frontier_info` and is better in 3 of 6 seeds, but two pairs are identical
(`tunnel_loop` 1, `long_tunnels` 3) although revisit mode was on: the decision logs show why.

- Revisit mode starts within the first ~60 m (sigma 0.05) and stays on, but a revisit is
  rarely chosen: its utility is `w_u x gain - 0.5 x path`, with `gain` a fraction (<= 1), so
  at most 80 against a path of 100-200 m back along a dead-end tunnel (-50 to -100), while a
  frontier scores ~+30 (unknown area) minus its path. On `long_tunnels` 3 the newest sigma
  reached 1.07 m with 75 candidates and no revisit was chosen until the last frontier was gone.
- On `tunnel_loop` 1 the first revisit (frame 4750) pointed along the same corridor as the
  only remaining frontier; the loop closed before the paths diverged, so the run is identical.

Reading: the revisit's value does not grow with the error it removes, so in large worlds the
travel cost always wins. Next development change: score a revisit by the absolute sigma it is
predicted to remove (metres, `newest_sigma - after`), so that its value grows with the drift,
and retune `w_u` on these two worlds only (the holdout stays untouched).

### Absolute revisit gain: tuning rule (written before its runs)

`exploration_revisit_absolute_gain` (`SelectionConfig::absolute_gain`) scores a revisit by
`w_u x (newest sigma - predicted sigma)` [m]; `min_predicted_drop` stays a fraction (0.2). With
the distance weight 0.5 per m, `w_u` is twice the path [m] one metre of sigma removed is worth.
Sweep on the tunnels set (seeds 1-3, `active` only; `frontier_info` and `nearest_frontier` are
the baseline's runs, which the change does not touch), `w_u` in {50, 150, 400}, trigger and
release unchanged. Choice: the lowest median final-map ATE over the 6 runs among the settings
whose median coverage is within 5 points of `frontier_info`'s and with no failed run; ties
(within 0.02 m) go to the smaller `w_u`. If no setting beats the fraction gain's median
(0.30 m `tunnel_loop`, 0.50 m `long_tunnels`, pooled 0.46 m), the change is not adopted.

### Absolute revisit gain: result (2026-11-27): not adopted

`active` over the 6 tunnel runs (final-map ATE median, coverage median, failed):

| gain | `w_u` | ATE [m] | coverage | failed |
| --- | --- | --- | --- | --- |
| fraction (baseline) | 80 | 0.458 | 0.993 | 0 |
| absolute | 50 | 0.654 | 0.993 | 0 |
| absolute | 150 | 0.565 | 0.992 | 0 |
| absolute | 400 | 0.309 | 0.333 | 5 |

By the rule the change is not adopted (50 and 150 are worse than the baseline; 400 fails the
coverage and failure conditions); the default stays the fraction gain. What the runs show:

- 50 and 150 mostly reproduce `frontier_info` (`tunnel_loop` 1 and 2, `long_tunnels` 3 are
  identical to it): with sigma 0.3-1 m the absolute gain is smaller than the fraction, so
  revisits are chosen even less.
- 400 does go back, and the parts it maps are accurate (ATE 0.03-0.37 m where it did not
  diverge, 35-91 loop edges), but revisit mode never ends (trigger 0.05 m is passed again within
  tens of metres), so it shuttles back and forth and runs out of the 1500 s budget at 23-57 %
  coverage. On `tunnel_loop` 1 and 2 the live error passed 10 m (divergence): with pilasters
  every 30 m the corridor repeats, and many revisit loop searches add the chance of a false loop.

Reading: the missing piece is not the revisit's value but when revisit mode is on. The trigger
(0.05 m) and release (0.02 m) were set with the half-blind sensor; at the tunnels' drift they
keep revisit mode on all the time.

### Revisit trigger: tuning rule (written before its runs)

Absolute gain with `w_u` 400 (the only setting that does go back), revisit mode on above a
trigger in {0.3, 0.5, 0.8} m and off below 0.4 x the trigger (0.12, 0.2, 0.32 m). Tunnels set,
seeds 1-3, `active` only (the other policies' runs are the baseline's). Choice and adoption: the
same rule as for the absolute gain (no failed run, median coverage within 5 points of
`frontier_info`, lowest median final-map ATE, ties within 0.02 m to the higher trigger, and
adopted only if below the fraction gain's 0.46 m). In parallel, the two diverged `w_u` 400 runs
are checked for false loop edges against the truth.

### Revisit trigger: result (2026-11-27): not adopted

`active`, absolute gain, `w_u` 400, over the 6 tunnel runs (before the degeneracy gate):

| trigger / release [m] | ATE median [m] | coverage median | failed (diverged) | false loops > 2 m |
| --- | --- | --- | --- | --- |
| 0.05 / 0.02 | 0.309 | 0.33 | 5 (2) | 15 |
| 0.3 / 0.12 | 0.392 | 0.57 | 4 (2) | 4 |
| 0.5 / 0.2 | 0.486 | 0.88 | 1 (0) | 5 |
| 0.8 / 0.32 | 0.623 | 0.99 | 1 (1) | 0 |

No setting meets the rule (every one has a failed run). 0.8 is not reached on 4 of 6 runs and
then equals `frontier_info`. The decision log of `tunnel_loop` 2 at 0.5 shows the failure mode
of the lower triggers: at the far end of the first tunnel (sigma 0.5) the robot drives 164 m back
to the start, the loop closes (sigma 0.2), it drives back to the frontier, and by then the
newest sigma is 0.5 again, so it turns back again: six round trips until the budget ends.

Reading: the newest submap's sigma is the wrong objective for a revisit. It measures the robot's
current pose, which every metre of travel down an unconstrained tunnel degrades again, while what
a revisit buys and keeps is the correction of the map already built. The next design step is to
trigger and score revisits on the map's uncertainty (e.g. the largest or summed submap sigma
over the graph, and the predicted reduction of it), so that a loop that has corrected a part of
the map is not paid for again.

### Map objective: tuning rule (written before its runs)

`exploration_revisit_map_objective` (`SelectionConfig::map_objective`): revisit mode follows the
map's mean sigma (each submap's sigma weighted by the coverage area it first observed;
`mapSigma`), and a revisit's gain is the predicted drop of that mean [m] (`predictedMapSigmaDrop`:
the trajectory from the target on is taken as a chain, scaled to the predicted newest drop).
A smoke run (`tunnel_loop` 2, trigger 0.3, `w_u` 1600) never entered revisit mode: the map's
mean sigma stayed within 0.07-0.26 m, and the run equals `frontier_info`. The grid is set from
that range: trigger in {0.1, 0.2} m (release half of it) x `w_u` in {400, 1600}, with the loop
degeneracy gate at 0.002. Tunnels set, seeds 1-3, `active` only; the comparison is with the
re-run baseline with the same gate (all three policies). Choice and adoption: as before (no
failed run, median coverage within 5 points of `frontier_info`'s, lowest median final-map ATE,
ties within 0.02 m to the smaller `w_u`, then the higher trigger), adopted only if its median ATE
is below that of the gated baseline's `active` (fraction gain). Whatever the outcome, this is
the last tuning round on these six runs: the next step freezes a configuration and builds new
holdout worlds.

### False loop edges in the tunnels (2026-11-27)

Every loop edge of the tunnel runs was checked against the truth (relative truth pose of the two
submaps vs the edge's measurement). The baseline's 18 runs have none off by more than 2 m. The
`w_u` 400 runs have 15 (10 of the 11 edges on `tunnel_loop` 2, 4 of 30 on `tunnel_loop` 1, 1 on
`long_tunnels` 1): all slide along the corridor (measured along-axis offset ~0, truth 2-8 m),
some with a good fitness (0.05). Revisiting more means more loop searches in a corridor that
repeats every 30 m, and the two diverged runs are these.

The registration's information separates them: the ratio of the smallest to the largest
eigenvalue of its translation block is <= 0.0016 for all 15, and of the 540 edges of the
baseline and sweep runs, the 450 with ratio >= 0.002 are all within 0.52 m of the truth (90
rejected). New gate `loop_min_translation_information_ratio`
(`GateConfig::min_translation_information_ratio`, default 0: off, so the release gate's runs do
not change): a loop registration whose ratio is below it is rejected. 0.002 is chosen from these
development runs; it changes the backend for every policy, so the tunnels are run again for all
three policies with it before any policy setting is compared.

### Tunnels baseline with the loop gate (2026-11-27, development)

`eval_config.yaml` plus the gate at 0.002, 18 runs (commit 5f77f54, clean tree). No loop edge
of any run is off by more than 2 m. Final-map ATE [m], median of the six runs:

| policy | median | runs |
| --- | --- | --- |
| `active` (fraction gain) | 0.427 | 0.07, 0.295, 0.414, 0.44, 0.724, 0.768 |
| `nearest_frontier` | 0.595 | 0.179, 0.355, 0.588, 0.603, 0.952, 1.464 |
| `frontier_info` | 0.658 | 0.295, 0.52, 0.593, 0.724, 0.801, 0.901 |

Per world: `active` vs `frontier_info` passes on `long_tunnels` (median reduction 30 %, 2 of 3
seeds lower) and fails on `tunnel_loop` (4 %), with 3 seeds per world.

### Map objective: result (2026-11-27): not adopted

`active` only, the other two policies are the gated baseline's runs. `frontier_info`'s median
coverage is 0.9917, so the floor is 0.9417.

| trigger / `w_u` | median ATE [m] | median coverage | failed | eligible |
| --- | --- | --- | --- | --- |
| 0.1 / 400 | 0.804 | 0.9984 | 0 | yes |
| 0.2 / 400 | 0.804 | 0.9984 | 0 | yes |
| 0.1 / 1600 | 0.433 | 0.7123 | 1 | no |
| 0.2 / 1600 | 0.550 | 0.9303 | 0 | no (coverage) |

The rule picks 0.2 / 400 (tie with 0.1 / 400: same runs, frame for frame); its 0.804 m is above
the gated baseline's 0.427 m, so it is not adopted. On `tunnel_loop` 1 and 2 the `w_u` 400 runs
equal the baseline frame for frame: their revisit goals differ, but both lie back down the same
corridor and the mode ends before either is reached. On `long_tunnels` they are worse (0.84-0.98
vs 0.41-0.72 m). `w_u` 1600 pulls back so often that coverage falls short. One loop edge of the
24 runs is off by more than 2 m (`0.1 / 1600`, `long_tunnels` 3).

### Frozen configuration (2026-11-27)

Tuning on the tunnels ends here. `eval_config.yaml` is frozen as the gated baseline: `active` with
the fraction gain (trigger 0.05, release 0.02, `w_u` 80, minimum predicted drop 0.2) and the loop
gate at 0.002 for every policy. The map objective and the absolute gain stay in the code, off by
default. Next: new holdout worlds, built and checked (static checks and a short dry run from
every start, no policy comparison) before the user approves them, then one run of the holdout
set with `--frozen` from a clean tree.

### Third holdout: worlds and calibration rule (written before any run on them)

Built after the configuration was frozen (approved by the maintainer 2026-11-27); no policy has
run on them. Both are 4 m tunnels with pilasters every 6 m as built, of the degenerate kind,
not copies of the development tunnels:

- `dogleg_loop` (150 x 70 m): a ~440 m loop with a dogleg (straights of 150, 40, 76, 30, 70 and
  66 m) and a 20 m dead-end spur.
- `mine_branches` (190 x 60 m): a 190 m main tunnel, a ~230 m loop above it through a top
  gallery, and three dead ends (28, 24, 20 m).

Static checks: `reachable_fractions` gives 1.0 from all ten starts (also with pilasters every
24, 36, 48 m); start clearance and the fixed routes are covered by `test_active_slam_eval.py`.

Difficulty, measured as for the tunnels: fixed routes (`worlds.ROUTES`, ~430 m and ~410 m,
on the true pose, no policy), odometry drift over 50 m, IMU noise scale 1.

- Sweep: pilaster spacing 6 (as built), 12, 24, 30, 36, 42, 48 m; seeds 1, 2, 3.
- Rule: per world, the setting closest to as built whose median drift over the three seeds is
  in 0.3-1.0 % and none of whose routes has a live error of 5 m or more. If a world has none,
  report that, and that world is not run.
- Budget 1500 s and divergence limit 10 m, as for the tunnels (both worlds are of their size).
- Then one 60 s check run from each of the ten starts with `nearest_frontier` only (no
  collision, no divergence, the robot moves); its ATE is not looked at.
- The maintainer approves the worlds and settings; then the set `holdout` (these two worlds,
  seeds 101-110, the three policies) is run once with `--frozen` from a clean tree and reported
  as it comes out, under the preregistered pass rule.

### Third holdout: calibration result (2026-11-27)

42 routes (~419 m and ~412 m), no collision. Median drift over 50 m [%] over seeds 1-3 (largest
live error over the three [m]):

| pilaster spacing [m] | dogleg_loop | mine_branches |
| --- | --- | --- |
| 6 (as built) | 0.144 (0.39) | 0.133 (0.16) |
| 12 | 0.143 (0.48) | 0.138 (0.27) |
| 24 | 0.148 (0.63) | 0.178 (0.38) |
| 30 | 0.153 (0.65) | 0.211 (0.93) |
| 36 | 0.326 (1.43) | 0.205 (0.29) |
| 42 | 0.159 (1.03) | 0.185 (0.66) |
| 48 | 0.173 (0.85) | 0.309 (2.08) |

By the rule: `dogleg_loop` every 36 m, `mine_branches` every 48 m; both just inside the band,
and no route reaches 5 m. The drift is not monotonic in the spacing (`dogleg_loop` 42 m is below
36 m): which pilasters remain near the long straights' middles matters more than how many.
Frozen in `eval_config.yaml` (`world_overrides`, set `holdout`) before any policy runs on them.

### Third holdout: check runs (2026-11-27)

`nearest_frontier`, 60 s, seeds 101-110 (all ten starts) per world, frozen difficulty and loop
gate. All 20 move 31-40 m and reach 10-15 % coverage, with no collision, no divergence and no
rejected goal; the largest live error is 0.52 m (`mine_branches` 101), the others are under
0.19 m. (`failed` is set in all of them only because coverage is below 50 % after 60 s.) ATE was
not looked at. Approved by the maintainer (2026-11-27); the holdout set is run once next.

### Third holdout: result (2026-11-27): fail

`run_eval.py --set holdout --frozen`, commit 736f1dd, clean tree, 60 runs, none failed; no loop
edge of the 499 is off by more than 2 m.

| world | `active` | `frontier_info` | `nearest_frontier` | reduction vs `frontier_info` | `active` lower | verdict |
| --- | --- | --- | --- | --- | --- | --- |
| `dogleg_loop` | 0.193 | 0.169 | 0.106 | 0.2 % | 5 / 10 | fail |
| `mine_branches` | 0.050 | 0.051 | 0.095 | -4.7 % | 5 / 10 | fail |

(median final-map ATE [m]; median coverage 0.99-1.00 for every policy and world.)

`active` revisits in every run (720-6984 revisit goal frames) and closes more loops than
`frontier_info` (e.g. 28 vs 11 on `mine_branches` 107), and it travels further (median 612 vs
504 m, 691 vs 619 m) and hits the 1500 s budget in 4 of 20 runs; but its final map is no more
accurate. In 7 of the 20 seed pairs the two ATEs are within 0.002 m of each other although
`active` closed different loops: the error that remains is not one a revisit's loop removes.
`nearest_frontier`, which never revisits, has the lowest median on `dogleg_loop`. The spread
between seeds (0.03-0.9 m) is far larger than any difference between the policies.

This holdout is now seen. By the preregistered rule the method has not shown that it makes the
map more accurate on unseen worlds.

### Why revisiting does not help: diagnosis (2026-11-27, development runs only)

`diagnose_map_error.py` on the gated tunnels baseline (18 runs) and the easy development set
(`eval_dev_fixed`, 36 runs); no holdout run is used. Medians (range):

| | easy worlds | tunnels |
| --- | --- | --- |
| ATE, odometry only (`trajectory_raw`) [m] | 0.028 | 0.607 (0.08-1.48) |
| ATE, after the pose graph [m] | 0.027 | 0.590 (0.07-1.46) |
| ATE after a similarity (scale) alignment [m] | 0.025 | 0.374 (0.05-0.77) |
| scale that best fits the map to the truth | 1.000 | 1.006 (1.001-1.018) |
| share of the squared error along the heading | 0.18 | 0.74 |
| drift a loop edge can see (truth vs odometry, between its two submaps) [m] | 0.036 | 0.057 |
| loop edge measurement error vs truth [m] | 0.010 | 0.009 |
| rank correlation of a submap's sigma with its error | -0.05 | 0.07 (-0.75..0.77) |

1. The loop edges are accurate (about 1 cm), but there is almost nothing for them to correct:
   between the two submaps of a loop the odometry has drifted only 4-6 cm, while the tunnel
   maps are 0.6 m off. The pose graph changes the map by less than 5 % in 43 of the 54 runs.
2. The tunnel error is mostly along the direction of travel, and a single scale factor removes
   a third to a half of it: the odometry under-measures the distance travelled through the
   feature-poor stretches (the maps are 0.1-1.8 % short). An error that depends on the place
   rather than on the time (the same stretch comes out short on every pass, in either direction)
   keeps every loop closed, so no loop and no revisit can see or remove it.
3. The graph's sigma does not say where the map is wrong (rank correlation about 0, either
   sign); it models time-accumulated drift, which is not where the error is.

So in these simulated worlds the error that the method targets (drift that a revisit's loop
removes) is small, and the error that dominates (place-dependent under-registration in
degenerate stretches) is invisible to loops and to the covariance. That explains the holdout
better than the policy's tuning does: `active` and `frontier_info` end within 2 mm of each other
in 7 of 20 holdout pairs despite closing different loops.

Correction: `check_loops.py` (scratch script used for "False loop edges in the tunnels") read
the truth 0.1 s early (`frames.csv`'s `t` is the frame's start; the truth is the pose at its
end), which adds up to ~0.14 m to each edge's error. The 2 m threshold for a false loop and the
15 false loops found are unaffected; the "within 0.52 m" figure is an upper bound. The harness's
own metrics (`final_map_ate`, `odometry_drift`) use the frame end and are correct.

### Does the way of driving change the error? Rule (written before its runs)

If the place-dependent under-registration depends on how a stretch is driven, the active part
of active SLAM is how to drive there, not where to go back to. First lever: speed (more scans
per metre). Harness: `--speed` (path follower, default 0.7 m/s as before).

- Fixed routes on the true pose (no policy) on `tunnel_loop` (pilasters 30 m) and
  `long_tunnels` (48 m), seeds 1-3, at 0.35, 0.7 and 1.0 m/s; budget long enough to finish.
- Measured on the odometry only (`trajectory_raw`, no loops on a route that does not repeat
  itself much): `|scale - 1|` of the similarity fit, the SE(2) ATE, and the drift over 50 m.
- Speed is worth building a policy on if, at 0.35 m/s, the median `|scale - 1|` over the six
  runs is at least 30 % lower than at 0.7 m/s and the ATE is lower in at least 5 of 6.
  Otherwise speed is recorded as not a lever here, and the next lever (keeping pilasters in
  view) is tried under its own rule.

### Speed: result (2026-11-27): slowing down is not a lever; the error depends on speed

18 routes (~400 m each, odometry only, no collision). Medians over the six runs per speed:

| speed [m/s] | `\|scale - 1\|` | SE(2) ATE [m] | drift over 50 m [%] | largest live error [m] |
| --- | --- | --- | --- | --- |
| 0.35 | 0.0136 | 0.940 | 1.24 | 1.95-2.65 |
| 0.7 | 0.0070 | 0.621 | 0.80 | 1.01-2.11 |
| 1.0 | 0.0005 | 0.167 | 0.20 | 0.49-0.76 |

By the rule speed is not a lever in the direction tested: at 0.35 m/s the scale error doubles
and the ATE is lower in 1 of 6 runs. The opposite holds strongly: at 1.0 m/s the shrink all but
disappears (ATE lower in 6 of 6). The distance lost per scan is about the same at 0.35 and 0.7
m/s (so the fraction doubles when the per-scan motion halves) and collapses at 0.1 m per scan:
it looks like a property of the frontend's registration in a degenerate corridor at small
per-scan motion against its 0.25 m voxel map (`voxel_size`, `max_correspondence_distance`), not
of the corridor's geometry alone. Two consequences:

- The tunnels' difficulty (calibrated at 0.7 m/s) is speed-dependent: a policy that drives
  faster changes the difficulty it is measured at. Every policy drives at 0.7 m/s.
- "Drive faster through degenerate stretches" would be adopted only under its own rule and only
  after the frontend's part is understood; it may be a parameter artifact rather than physics.

### Frontend's part in the shrink: rule (written before its runs)

Diagnostic only (the frozen evaluation keeps its frontend). The same routes as the speed test,
at 0.7 m/s, seeds 1-3, `tunnel_loop` (30 m) and `long_tunnels` (48 m), odometry only, one
RKO-LIO parameter group changed at a time from `rko_lio_mid360_walking.yaml`:

- `vox125`: `voxel_size` and `max_correspondence_distance` 0.125 m (from 0.25 m)
- `corr50`: `max_correspondence_distance` 0.5 m (the upstream default)
- `noloc`: `localizability_weighting` off

Reference: the speed test's 0.7 m/s runs (median `|scale - 1|` 0.0070, ATE 0.621 m). A variant
explains the shrink if its median `|scale - 1|` is at most half the reference's and its ATE is
lower in at least 5 of 6 runs. If one does, the shrink is a frontend setting, fixed (and
checked against the release gate) outside the active SLAM work, and the tunnels are
re-measured before revisiting is evaluated again. If none does, driving (speed, keeping
pilasters in view) is the lever, and a policy for it is built under its own rule.

### Frontend's part in the shrink: result (2026-11-27): no single setting explains it

Medians over six routes at 0.7 m/s (reference: `|scale - 1|` 0.0070, ATE 0.621 m):

| variant | `\|scale - 1\|` | ATE [m] | ATE lower than reference | note |
| --- | --- | --- | --- | --- |
| `vox125` | 0.0056 | 0.470 | 4 / 6 | `tunnel_loop` 0.24-0.68 m (better); `long_tunnels` 2 lost (30 m), 1 worse (3.5 m) |
| `corr50` | 0.0217 | 1.675 | 0 / 6 | shrink three times larger |
| `noloc` | 0.0222 | 1.530 | 0 / 6 | shrink three times larger |

None meets the rule. The current settings (one-voxel correspondence distance, localizability
weighting) already cut the shrink to a third of the upstream defaults'; a finer map helps on one
tunnel and loses the odometry on the other. The shrink is the frontend's under-registration
along a degenerate corridor, reduced but not removed by its settings. By the rule, how the robot
drives through degenerate stretches is the lever left; a policy for it is built only under its
own rule, and a sim-only gain from driving faster would need checking on real data before it
is claimed.

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
