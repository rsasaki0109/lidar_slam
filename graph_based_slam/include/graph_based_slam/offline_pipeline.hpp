// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// The offline backend (graph_slam_offline_runner) as a library without ROS.
//
//   graphslam::offline::OfflinePipeline pipeline(parameters, log);
//   if (!pipeline.configure()) {...}           // parameters, loop registration, receipt
//   pipeline.begin();
//   for each recorded message on pipeline.odometryTopic() / pipeline.cloudTopic(), in bag order:
//     pipeline.addOdometry(odometry) / pipeline.addCloud(std::move(cloud));
//   if (!pipeline.finish()) {...}             // pose graph, refinement, outputs in outputDir()
//
// The parameters are those of the ROS 2 runner (the node graph_based_slam, e.g.
// lidarslam/param/lidarslam_mid360_rko_graph.yaml); odometry and clouds are paired by equal
// stamps. Given the same parameters and messages, the outputs are byte-identical to the ROS 2
// runner's, which is a thin shell around this class.

#ifndef GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_
#define GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/gnss_anchor_store.hpp"
#include "graph_based_slam/map_export.hpp"
#include "graph_based_slam/map_store.hpp"
#include "graph_based_slam/offline_runtime.hpp"

namespace graphslam
{
namespace offline
{

// What a live system could output for one odometry / cloud pair, as the causal trajectory
// (causal_trajectory_output) defines it: the pair's odometry pose and that pose corrected by the
// latest pose-graph optimization the background worker had finished by the pair's stamp.
struct LivePose
{
  double stamp_sec {0.0};
  Eigen::Isometry3d odometry {Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map {Eigen::Isometry3d::Identity()};
  // The pair started a submap.
  bool new_submap {false};
  // Submaps and corrections (optimizations whose result applies) so far.
  std::size_t submaps {0U};
  std::size_t corrections_applied {0U};
};

using LivePoseListener = std::function<void (const LivePose &)>;

// The explorer's next goal now (active_exploration.hpp; docs/roadmap/active-slam-2026-11.md),
// for a robot at a position in the odometry frame: OfflinePipeline::decideExploration.
struct ExplorationDecision
{
  // "goal", "exploration_done", "no_reachable_goal", or "disabled" (active_exploration_timeline
  // is off) / "no_submaps" (nothing mapped yet).
  std::string status {"disabled"};
  std::string kind;  // "frontier" or "revisit" with a goal
  bool revisit_mode {false};
  double newest_sigma {0.0};  // the newest submap's sigma in the latest uncertainty state
  Eigen::Vector2d goal {Eigen::Vector2d::Zero()};  // odometry frame
  double path_length {0.0};
  std::vector<Eigen::Vector2d> path;  // from the robot's cell to the goal's (odometry frame)
  double unknown_area {0.0};
  double uncertainty_drop {0.0};
  std::size_t submaps {0U};
  std::size_t frontiers {0U};
  std::size_t reachable_frontiers {0U};
  std::size_t revisit_candidates {0U};
};

// The map-saving parameters (offline_map_bundle's map_leaf_size, map_grid_size_x, ...), with the
// live node's defaults. Throws ParameterTypeError for a parameter of the wrong type.
map_export::MapExportConfig mapExportConfig(const Parameters & parameters);

class OfflinePipeline
{
public:
  // Both are borrowed and must outlive the pipeline.
  OfflinePipeline(const Parameters & parameters, const Log & log);
  ~OfflinePipeline();
  OfflinePipeline(const OfflinePipeline &) = delete;
  OfflinePipeline & operator=(const OfflinePipeline &) = delete;

  // Reads the parameters and activates the loop registration (writes
  // registration_plugin_receipt.yaml). Returns false, after logging why, when that fails.
  // Throws ParameterTypeError for a parameter of the wrong type.
  bool configure();

  // After configure(): bag_path, odom_topic, cloud_topic and output_dir.
  const std::string & inputPath() const;
  const std::string & odometryTopic() const;
  const std::string & cloudTopic() const;
  const std::string & outputDir() const;

  // Called for every odometry / cloud pair, as it is processed, with its live pose. Only with
  // causal_trajectory_output (without it the listener is not called). Set before begin(). The
  // outputs are the same with or without a listener.
  void setLivePoseListener(LivePoseListener listener);

  // With active_exploration_timeline (and pose_uncertainty_timeline): the goal the explorer
  // would choose now for a robot at robot_xy (odometry frame), from the coverage grid of the
  // submaps so far and the latest uncertainty state. For a closed loop in which a controller
  // drives the robot (tools/rne_active_slam_env). In-line loop search only. Does not change
  // any output file.
  ExplorationDecision decideExploration(const Eigen::Vector2d & robot_xy);

  // The robot could not reach a goal decideExploration gave (blocked, collided, timed out): it
  // is not chosen again (frontier goals within exploration_visited_radius of it, revisit targets
  // within exploration_revisit_tried_radius until revisit mode ends).
  void rejectExplorationGoal(const Eigen::Vector2d & goal);

  // The explorer's coverage grid now: cell states (0 unknown, 1 free, 2 occupied) row by row
  // (y outer), cell (x0, y0) first, cells of `resolution` metres with integer coordinates
  // floor(world / resolution). Empty without active_exploration_timeline.
  struct ExplorationGrid
  {
    int x0 {0};
    int y0 {0};
    int width {0};
    int height {0};
    double resolution {0.0};
    std::vector<std::uint8_t> cells;
  };
  ExplorationGrid explorationGrid() const;

  // Keep the session's submaps (stamps, odometry and optimized poses, compressed scans) for
  // sessionMap(), the session of a long-term map store (map_store.hpp). Set before begin(). The
  // outputs are the same either way.
  void keepSessionMap(bool keep);

  // The input, in recorded order, between begin() and finish().
  void begin();
  void addOdometry(const msg::Odometry & odometry);
  void addCloud(msg::PointCloud2 && cloud);
  // use_gnss: the NavSatFix messages of gnssTopic(), in recorded order with the odometry and the
  // clouds (ignored when use_gnss is false).
  bool usesGnss() const;
  const std::string & gnssTopic() const;
  void addGnssFix(const gnss::Fix & fix);

  // The remaining stages and all outputs. Returns false, after logging why, when the fixed loop
  // edges (fixed_loop_edges_path) cannot be loaded.
  bool finish();

  // After finish() with keepSessionMap(true): the session's submaps at their optimized poses
  // (the poses of the map bundle), its loop edge count, GNSS origin and pose graph.
  const map_store::SessionMap & sessionMap() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace offline
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_
