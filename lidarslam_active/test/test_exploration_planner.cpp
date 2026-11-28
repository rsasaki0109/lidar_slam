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

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "lidarslam_active/exploration_planner.hpp"

namespace
{

using lidarslam_active::ExplorationPlanner;
using lidarslam_active::PlannerParams;
using lidarslam_active::PlanStatus;

// A planar LiDAR in an axis-aligned box room [0, length] x [0, width]: per ray an obstacle point
// at the wall (sensor height) and ground points along the free part of the ray.
std::vector<Eigen::Vector3d> scanInBox(
  const Eigen::Vector2d & sensor, double length, double width, double max_range)
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 720; ++i) {
    const double a = 2.0 * M_PI * i / 720.0;
    const Eigen::Vector2d d(std::cos(a), std::sin(a));
    double t = std::numeric_limits<double>::infinity();
    if (d.x() > 1e-9) {t = std::min(t, (length - sensor.x()) / d.x());}
    if (d.x() < -1e-9) {t = std::min(t, -sensor.x() / d.x());}
    if (d.y() > 1e-9) {t = std::min(t, (width - sensor.y()) / d.y());}
    if (d.y() < -1e-9) {t = std::min(t, -sensor.y() / d.y());}
    const double reach = std::min(t, max_range);
    for (double r = 0.3; r < reach; r += 0.25) {
      points.emplace_back(r * d.x(), r * d.y(), -0.6);
    }
    if (t <= max_range) {
      points.emplace_back(t * d.x(), t * d.y(), 0.0);
    }
  }
  return points;
}

Eigen::Isometry3d poseAt(const Eigen::Vector2d & xy)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(xy.x(), xy.y(), 0.0);
  return pose;
}

PlannerParams shortRange()
{
  PlannerParams params;
  params.grid.max_range = 6.0;
  return params;
}

TEST(ExplorationPlanner, WaitsForData)
{
  ExplorationPlanner planner;
  const auto result = planner.plan(Eigen::Vector2d::Zero(), 0.0);
  EXPECT_EQ(result.status, PlanStatus::WAITING_FOR_DATA);
  EXPECT_FALSE(result.has_goal);
}

TEST(ExplorationPlanner, TakesKeyframesByDistanceAndAngle)
{
  ExplorationPlanner planner(shortRange());
  const Eigen::Vector2d at(2.0, 3.0);
  EXPECT_TRUE(planner.addScan(poseAt(at), scanInBox(at, 30.0, 6.0, 6.0)));
  EXPECT_FALSE(planner.addScan(poseAt(at + Eigen::Vector2d(0.5, 0.0)), {}));
  Eigen::Isometry3d turned = poseAt(at);
  turned.linear() = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(planner.addScan(turned, {}));
  EXPECT_TRUE(planner.addScan(poseAt(at + Eigen::Vector2d(1.1, 0.0)), {}));
  EXPECT_EQ(planner.keyframes(), 3U);
}

TEST(ExplorationPlanner, SendsTheGoalTowardsTheUnseenEndOnce)
{
  ExplorationPlanner planner(shortRange());
  const Eigen::Vector2d at(2.0, 3.0);
  planner.addScan(poseAt(at), scanInBox(at, 30.0, 6.0, 6.0));
  const auto first = planner.plan(at, 0.0);
  ASSERT_EQ(first.status, PlanStatus::EXPLORING);
  ASSERT_TRUE(first.has_goal);
  EXPECT_TRUE(first.new_goal);
  EXPECT_GT(first.goal.x(), at.x() + 2.0);  // the corridor continues in +x beyond the range
  EXPECT_GE(first.path.size(), 2U);
  const auto second = planner.plan(at, 1.0);
  EXPECT_TRUE(second.has_goal);
  EXPECT_FALSE(second.new_goal);
  EXPECT_LT((second.goal - first.goal).norm(), 1e-9);
}

TEST(ExplorationPlanner, RejectsAGoalTheRobotStopsMovingTowards)
{
  PlannerParams params = shortRange();
  params.stall_timeout = 10.0;
  ExplorationPlanner planner(params);
  const Eigen::Vector2d at(2.0, 3.0);
  planner.addScan(poseAt(at), scanInBox(at, 30.0, 6.0, 6.0));
  const auto first = planner.plan(at, 0.0);
  ASSERT_TRUE(first.has_goal);
  EXPECT_TRUE(planner.plan(at, 5.0).rejected.empty());
  const auto stalled = planner.plan(at, 11.0);
  EXPECT_EQ(stalled.rejected, "stalled");
  if (stalled.has_goal) {
    EXPECT_GT((stalled.goal - first.goal).norm(), params.goal_change_distance);
  }
}

TEST(ExplorationPlanner, RejectsAGoalNotReachedInTime)
{
  PlannerParams params = shortRange();
  params.goal_timeout = 30.0;
  params.stall_timeout = 1000.0;
  ExplorationPlanner planner(params);
  const Eigen::Vector2d at(2.0, 3.0);
  planner.addScan(poseAt(at), scanInBox(at, 30.0, 6.0, 6.0));
  ASSERT_TRUE(planner.plan(at, 0.0).has_goal);
  // The robot moves (no stall) but does not get there in time.
  const auto late = planner.plan(at + Eigen::Vector2d(0.0, 0.6), 31.0);
  EXPECT_EQ(late.rejected, "timeout");
}

TEST(ExplorationPlanner, DeclaresTheExplorationDoneInASeenRoom)
{
  PlannerParams params;
  params.grid.max_range = 30.0;
  params.done_confirmations = 3;
  ExplorationPlanner planner(params);
  // A small closed room seen whole from two places.
  for (const Eigen::Vector2d & at : {Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(4.0, 2.0)}) {
    planner.addScan(poseAt(at), scanInBox(at, 6.0, 4.0, 30.0));
  }
  const Eigen::Vector2d robot(3.0, 2.0);
  PlanStatus status = PlanStatus::WAITING_FOR_DATA;
  for (int i = 0; i < 3; ++i) {
    const auto result = planner.plan(robot, static_cast<double>(i));
    EXPECT_FALSE(result.new_goal);
    status = result.status;
  }
  EXPECT_EQ(status, PlanStatus::EXPLORATION_DONE);
}

}  // namespace
