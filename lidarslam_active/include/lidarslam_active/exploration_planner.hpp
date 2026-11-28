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
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
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

#ifndef LIDARSLAM_ACTIVE__EXPLORATION_PLANNER_HPP_
#define LIDARSLAM_ACTIVE__EXPLORATION_PLANNER_HPP_

// The frontier explorer of the lidarslam_active node, without ROS: keyframe scans go into a
// coverage grid, and plan() returns the goal to drive to (docs/roadmap/active-slam-2026-11.md,
// P4). The decision is OfflinePipeline::decideExploration's (graph_based_slam): frontiers and
// paths through known free cells, then through unknown cells when only unreachable frontiers
// are left, scored by active_exploration::selectGoal. On top it tracks the goal it handed out:
// a goal not reached in time, or one the robot stopped making progress towards, is rejected
// (not chosen again), as a navigation stack aborting a goal.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "graph_based_slam/active_exploration.hpp"

namespace lidarslam_active
{

struct PlannerParams
{
  graphslam::active_exploration::CoverageGridConfig grid;
  graphslam::active_exploration::PlannerConfig planner;
  graphslam::active_exploration::FrontierConfig frontier;
  // Defaults: the harness's frontier_info policy (frontier information per path length, no
  // revisits: the trigger sigma is never reached without an uncertainty input).
  graphslam::active_exploration::SelectionConfig selection = []() {
      graphslam::active_exploration::SelectionConfig s;
      s.info_weight = 1.0;
      s.distance_weight = 0.5;
      return s;
    }();
  double keyframe_distance {1.0};  // [m] a scan becomes a keyframe this far from the last one
  double keyframe_angle {0.5};  // [rad] or turned this much
  double goal_timeout {120.0};  // [s] a goal not reached in this time is rejected
  double stall_timeout {20.0};  // [s] a goal is rejected when the robot moved less than
  double stall_distance {0.5};  // [m] this in stall_timeout
  double goal_change_distance {0.5};  // [m] a chosen goal closer than this keeps the old one
  // Consecutive decisions without a goal before the exploration is declared over.
  int done_confirmations {3};
};

enum class PlanStatus
{
  WAITING_FOR_DATA,  // no keyframe yet
  EXPLORING,  // a goal is active (goal, path)
  EXPLORATION_DONE,  // no reachable frontier left
  NO_REACHABLE_GOAL,  // frontiers left, none reachable even through unknown cells
};

inline const char * toString(PlanStatus status)
{
  switch (status) {
    case PlanStatus::WAITING_FOR_DATA: return "waiting_for_data";
    case PlanStatus::EXPLORING: return "exploring";
    case PlanStatus::EXPLORATION_DONE: return "exploration_done";
    case PlanStatus::NO_REACHABLE_GOAL: return "no_reachable_goal";
  }
  return "unknown";
}

struct PlanResult
{
  PlanStatus status {PlanStatus::WAITING_FOR_DATA};
  bool new_goal {false};  // goal differs from the one handed out before: send it
  bool has_goal {false};
  Eigen::Vector2d goal {Eigen::Vector2d::Zero()};
  std::vector<Eigen::Vector2d> path;  // planned path to the goal (cell centres)
  bool through_unknown {false};  // the path crosses unknown cells
  std::size_t frontiers {0U};
  std::string rejected;  // "", "timeout" or "stalled": the previous goal was rejected now
};

class ExplorationPlanner
{
public:
  explicit ExplorationPlanner(const PlannerParams & params = PlannerParams())
  : params_(params), grid_(params.grid.resolution) {}

  const PlannerParams & params() const {return params_;}
  const graphslam::active_exploration::CoverageGrid & grid() const {return grid_;}
  std::size_t keyframes() const {return keyframes_;}

  // A scan (points in the sensor frame) and the sensor pose in the planning frame. It goes into
  // the grid when it is the first one or the sensor moved keyframe_distance or turned
  // keyframe_angle since the last keyframe; returns whether it did.
  bool addScan(const Eigen::Isometry3d & sensor_pose, std::vector<Eigen::Vector3d> points)
  {
    if (keyframes_ > 0U) {
      const double moved = (sensor_pose.translation() - last_keyframe_.translation()).norm();
      const double turned =
        Eigen::AngleAxisd(last_keyframe_.linear().transpose() * sensor_pose.linear()).angle();
      if (moved < params_.keyframe_distance && turned < params_.keyframe_angle) {
        return false;
      }
    }
    graphslam::active_exploration::SubmapScan scan;
    scan.pose = sensor_pose;
    scan.points = std::move(points);
    grid_.addScan(scan, params_.grid);
    last_keyframe_ = sensor_pose;
    ++keyframes_;
    return true;
  }

  // The navigation stack reports the goal unreachable: it is not chosen again.
  void rejectGoal(const Eigen::Vector2d & goal)
  {
    state_.visited_frontiers.push_back(goal);
    if (has_goal_ && (goal - goal_).norm() < 1e-9) {
      has_goal_ = false;
    }
  }

  // The decision for a robot at robot_xy (planning frame) at time now [s].
  PlanResult plan(const Eigen::Vector2d & robot_xy, double now)
  {
    namespace ae = graphslam::active_exploration;
    PlanResult out;
    if (grid_.empty()) {
      return out;
    }
    if (!has_progress_ref_ || (robot_xy - progress_xy_).norm() >= params_.stall_distance) {
      progress_xy_ = robot_xy;
      progress_time_ = now;
      has_progress_ref_ = true;
    }
    if (has_goal_) {
      if (now - goal_time_ > params_.goal_timeout) {
        out.rejected = "timeout";
      } else if (now - progress_time_ > params_.stall_timeout) {
        out.rejected = "stalled";
      }
      if (!out.rejected.empty()) {
        rejectGoal(goal_);
        progress_time_ = now;
      }
    }

    const ae::GraphUncertainty no_uncertainty;
    const ae::PathField known_paths(grid_, params_.grid, params_.planner, robot_xy);
    std::unique_ptr<ae::PathField> optimistic_paths;
    const ae::PathField * paths = &known_paths;
    std::vector<ae::Frontier> frontiers =
      ae::findFrontiers(grid_, params_.grid, known_paths, params_.frontier);
    ae::Decision decision =
      ae::selectGoal(known_paths, frontiers, no_uncertainty, params_.selection, state_);
    const bool unreachable_left = std::any_of(
      frontiers.begin(), frontiers.end(), [](const ae::Frontier & f) {return !f.reachable;});
    if (decision.status != ae::Decision::Status::GOAL && unreachable_left &&
      !params_.planner.allow_unknown)
    {
      ae::PlannerConfig optimistic = params_.planner;
      optimistic.allow_unknown = true;
      optimistic_paths =
        std::make_unique<ae::PathField>(grid_, params_.grid, optimistic, robot_xy);
      paths = optimistic_paths.get();
      frontiers = ae::findFrontiers(grid_, params_.grid, *paths, params_.frontier);
      decision = ae::selectGoal(*paths, frontiers, no_uncertainty, params_.selection, state_);
      out.through_unknown = decision.status == ae::Decision::Status::GOAL;
    }
    out.frontiers = frontiers.size();

    if (decision.status == ae::Decision::Status::GOAL && decision.chosen >= 0) {
      done_count_ = 0;
      const ae::GoalCandidate & chosen =
        decision.candidates[static_cast<std::size_t>(decision.chosen)];
      if (!has_goal_ || (chosen.goal - goal_).norm() > params_.goal_change_distance) {
        goal_ = chosen.goal;
        goal_time_ = now;
        has_goal_ = true;
        out.new_goal = true;
        progress_xy_ = robot_xy;
        progress_time_ = now;
      }
      out.status = PlanStatus::EXPLORING;
      out.has_goal = true;
      out.goal = goal_;
      out.path = paths->pathTo(goal_);
      return out;
    }

    ++done_count_;
    if (done_count_ < params_.done_confirmations && has_goal_) {
      // Not confirmed yet: keep driving to the current goal.
      out.status = PlanStatus::EXPLORING;
      out.has_goal = true;
      out.goal = goal_;
      out.path = paths->pathTo(goal_);
      return out;
    }
    if (done_count_ < params_.done_confirmations) {
      out.status = PlanStatus::EXPLORING;
      return out;
    }
    has_goal_ = false;
    out.status = decision.status == ae::Decision::Status::EXPLORATION_DONE ?
      PlanStatus::EXPLORATION_DONE : PlanStatus::NO_REACHABLE_GOAL;
    return out;
  }

private:
  PlannerParams params_;
  graphslam::active_exploration::CoverageGrid grid_;
  graphslam::active_exploration::ExplorerState state_;
  Eigen::Isometry3d last_keyframe_ {Eigen::Isometry3d::Identity()};
  std::size_t keyframes_ {0U};
  bool has_goal_ {false};
  Eigen::Vector2d goal_ {Eigen::Vector2d::Zero()};
  double goal_time_ {0.0};
  bool has_progress_ref_ {false};
  Eigen::Vector2d progress_xy_ {Eigen::Vector2d::Zero()};
  double progress_time_ {0.0};
  int done_count_ {0};
};

}  // namespace lidarslam_active

#endif  // LIDARSLAM_ACTIVE__EXPLORATION_PLANNER_HPP_
