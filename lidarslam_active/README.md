# lidarslam_active

Opt-in frontier exploration for lidarslam: the node keeps a coverage grid from the SLAM's point
clouds and sends the next exploration goal to a navigation stack as a `geometry_msgs/PoseStamped`
on `goal_pose`, the topic Nav2's `bt_navigator` listens to. It has no Nav2 dependency and is not
part of any default launch file.

```bash
ros2 launch lidarslam_active exploration.launch.py cloud_topic:=/rko_lio/frame
```

| Topic | Type | |
| --- | --- | --- |
| `cloud` (in) | `sensor_msgs/PointCloud2` | scans, e.g. RKO-LIO's deskewed scan |
| TF (in) | | `global_frame` <- cloud frame, `global_frame` <- `robot_frame` |
| `goal_pose` (out) | `geometry_msgs/PoseStamped` | sent when the chosen goal changes |
| `exploration_path` (out) | `nav_msgs/Path` | planned path on the coverage grid |
| `exploration_grid` (out) | `nav_msgs/OccupancyGrid` | the coverage grid (latched) |
| `exploration_status` (out) | `std_msgs/String` | `waiting_for_data`, `exploring`, `exploration_done`, `no_reachable_goal` |

Goals are chosen by frontier information per path length (`policy: frontier_info`), the
decision of graph_based_slam's offline explorer (`active_exploration.hpp`). A goal not reached
within `goal_timeout`, or one the robot stops making progress towards (`stall_timeout`,
`stall_distance`), is dropped and not chosen again. After `done_confirmations` decisions without
a goal the node reports `exploration_done` and sends nothing more; saving the map stays a
separate call (`ros2 service call /map_save std_srvs/srv/Empty`). Parameters and their defaults
are in `config/exploration.yaml`; the height bands of the grid are relative to the sensor.

The uncertainty-driven `active` policy is not available here: in the simulated evaluation it did
not beat frontier exploration (`docs/roadmap/active-slam-2026-11.md`), and it would need the
graph's uncertainty published live. This node has been tested with unit tests on synthetic scans
only, not yet with Nav2 on a robot.
