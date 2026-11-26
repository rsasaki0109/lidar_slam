# Closed-loop active SLAM environment (RobotNativeEngine)

The simulation side of the active SLAM evaluation (`docs/roadmap/active-slam-2026-11.md`, P5):
a robot explores a world while lidar_slam chooses where it goes next.

- `src/main.rs` (`rne_active_slam_env`): a kinematic differential-drive robot with
  [RobotNativeEngine](https://github.com/rsasaki0109/RobotNativeEngine)'s Livox Mid-360 model
  (its scan pattern, range noise and per-return emission times; MIT OR Apache-2.0, pinned by
  commit in `Cargo.toml`) and an IMU, in a static world of boxes. One request on stdin advances
  one 0.1 s LiDAR frame; the reply on stdout is the frame's points and IMU samples in the
  `livox_ros_driver2` layout (sensor frame, per-point absolute timestamps, acceleration in g) and
  the ground-truth pose. Lockstep: the simulation never runs ahead of the SLAM, so a run is
  deterministic.
- `run_closed_loop.py`: the loop. It feeds the frames to `lidarslam.live.Session` (RKO-LIO and
  graph_based_slam, ROS-free), asks the backend for the next goal and its path
  (`Session.decide_exploration`, `active_exploration.hpp`), and drives there with a pure-pursuit
  controller on the SLAM's own pose. The ground truth is used for the metrics only: floor
  coverage, distance, live position error.

This is a prototype kept here while it settles; the simulator part belongs in
RobotNativeEngine.

```bash
cargo build --release --manifest-path tools/rne_active_slam_env/Cargo.toml
python3 tools/rne_active_slam_env/run_closed_loop.py --world ring --policy nearest_frontier \
  --out /tmp/ring_nearest
python3 tools/rne_active_slam_env/run_closed_loop.py --world ring --policy active \
  --revisit-trigger-sigma 1.5 --revisit-release-sigma 0.5 --out /tmp/ring_active
```

Needs the `lidarslam` package built from this tree (`pip install ./lidarslam_core`).

Checks of the simulated data (see the roadmap): a frame's points, moved to the world with the
true pose at each point's own time, all land on the world's surfaces; with one pose for the
whole frame (no deskew) 80-88 % do while turning at 0.8 rad/s. On these scans RKO-LIO's car and
handheld-outdoor presets slide 1-2 m along a corridor when the robot stops and turns in place;
the indoor (walking) preset does not, so it is the default here.
