# Active SLAM: choose where to go next from the SLAM's own uncertainty (2026-11)

Goal: a robot that maps a place decides where to drive next from what the SLAM knows about
itself: where the map is still unobserved, where the trajectory is weakly constrained, and where a
revisit would close a loop. The SLAM proposes goals and scores them; driving there is left to an
existing navigation stack (Nav2). This repository gains no motion planner or controller.

Status: P1 started. Done: the offline pose-graph uncertainty report (below, P1 "Done").

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
robust kernels and non-SE3 edges (gravity prior, plane revisit) are not in the g2o text and are
ignored (counted in the summary). Cost: one sparse Cholesky plus 6 solves per vertex.

Next in P1: time the report on the largest existing bundles; expose the same computation in the
backend (after each optimization, off by default) and the `SlamUncertainty` message; then
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

## Evaluation (to be fixed in P0, before any closed-loop run)

- Metrics per run: ATE against RNE ground truth vs distance driven; coverage vs distance driven;
  map MME and plane thickness (`run_map_quality_check.sh`); loops closed; failure (SLAM divergence
  or stuck) rate.
- Runs: N seeds per world per method (N to be fixed).
- Success rule: thresholds on ATE and coverage improvement over nearest frontier (to be fixed).

## Risks

- Simulation-to-real gap: the RNE Mid-360 model reduces it but does not remove it; no real
  closed-loop result is claimed.
- Cost of marginal covariance on large graphs: approximate (local subgraph or sparsified) if P1
  measurements require it.
- Determinism over ROS 2 may not hold (see P5); the byte-identical gates apply to P1 only until it
  does.
