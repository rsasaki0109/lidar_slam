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

#include <cmath>
#include <limits>
#include <vector>

#include "graph_based_slam/active_exploration.hpp"

namespace
{

namespace ae = graphslam::active_exploration;

struct Segment
{
  Eigen::Vector2d a;
  Eigen::Vector2d b;
};

// Walls of the 10 m x 6 m room [0, 10] x [0, 6]; with a doorway, the right wall has a 2 m gap
// at y in [2, 4] (the space behind it is never seen: rays through it return nothing).
std::vector<Segment> room(bool doorway)
{
  std::vector<Segment> walls{
    {{0, 0}, {10, 0}}, {{0, 6}, {10, 6}}, {{0, 0}, {0, 6}}};
  if (doorway) {
    walls.push_back({{10, 0}, {10, 2}});
    walls.push_back({{10, 4}, {10, 6}});
  } else {
    walls.push_back({{10, 0}, {10, 6}});
  }
  return walls;
}

// A 2D LiDAR-like scan from `sensor`: 1440 rays, each ending at the nearest wall within 15 m
// (wall height 0 m relative to the sensor) or, missing every wall, producing no point; plus
// floor points (0.8 m below the sensor) every 0.25 m out to 3 m along every 8th ray.
ae::SubmapScan scanFrom(const Eigen::Vector2d & sensor, const std::vector<Segment> & walls)
{
  ae::SubmapScan scan;
  scan.pose.translation() = Eigen::Vector3d(sensor.x(), sensor.y(), 0.0);
  for (int k = 0; k < 1440; ++k) {
    const double angle = 2.0 * M_PI * k / 1440.0;
    const Eigen::Vector2d dir(std::cos(angle), std::sin(angle));
    double best = 15.0;
    for (const Segment & wall : walls) {
      const Eigen::Vector2d e = wall.b - wall.a;
      Eigen::Matrix2d m;
      m << dir, -e;
      if (std::abs(m.determinant()) < 1e-12) {
        continue;
      }
      const Eigen::Vector2d tu = m.inverse() * (wall.a - sensor);
      if (tu.x() > 1e-6 && tu.y() >= 0.0 && tu.y() <= 1.0) {
        best = std::min(best, tu.x());
      }
    }
    if (best < 15.0) {
      scan.points.emplace_back(best * dir.x(), best * dir.y(), 0.0);
    }
    if (k % 8 == 0) {
      for (double r = 0.25; r <= std::min(3.0, best - 0.1); r += 0.25) {
        scan.points.emplace_back(r * dir.x(), r * dir.y(), -0.8);
      }
    }
  }
  return scan;
}

ae::CoverageGridConfig gridConfig()
{
  ae::CoverageGridConfig config;
  config.resolution = 0.25;
  return config;
}

struct World
{
  ae::CoverageGrid grid;
  std::vector<ae::Frontier> frontiers;
};

TEST(ActiveExploration, GridMarksWallsFreeSpaceAndUnknown)
{
  const auto walls = room(false);
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({3.0, 3.0}, walls), scanFrom({7.0, 3.0}, walls)}, config);
  auto state = [&](double x, double y) {return grid.stateAt({x, y}, config.occupied_ratio);};
  EXPECT_EQ(state(5.0, 3.0), ae::Cell::FREE);
  EXPECT_EQ(state(1.0, 1.0), ae::Cell::FREE);
  EXPECT_EQ(state(5.0, 6.05), ae::Cell::OCCUPIED);
  EXPECT_EQ(state(10.05, 3.0), ae::Cell::OCCUPIED);
  EXPECT_EQ(state(12.0, 3.0), ae::Cell::UNKNOWN);  // behind the wall
}

TEST(ActiveExploration, GridGrowsAndKeepsWhatItSaw)
{
  const auto walls = room(false);
  const ae::CoverageGridConfig config = gridConfig();
  ae::CoverageGrid grid(config.resolution);
  grid.addScan(scanFrom({3.0, 3.0}, walls), config);
  const int width = grid.width();
  // A second room 200 m away: the grid grows and the first room is unchanged.
  std::vector<Segment> far_walls;
  for (const Segment & w : walls) {
    far_walls.push_back({w.a + Eigen::Vector2d(200, -50), w.b + Eigen::Vector2d(200, -50)});
  }
  ae::CoverageGrid both = grid;
  both.addScan(scanFrom({203.0, -47.0}, far_walls), config);
  EXPECT_GT(both.width(), width + 700);
  for (double y = -0.5; y <= 6.5; y += 0.25) {
    for (double x = -0.5; x <= 10.5; x += 0.25) {
      EXPECT_EQ(both.stateAt({x, y}, 0.25), grid.stateAt({x, y}, 0.25)) << x << ", " << y;
    }
  }
  EXPECT_EQ(both.stateAt({205.0, -47.0}, 0.25), ae::Cell::FREE);
}

TEST(ActiveExploration, ClosedRoomIsExploredAndDoorwayIsAFrontier)
{
  const ae::CoverageGridConfig config = gridConfig();
  const ae::PlannerConfig planner;
  const ae::FrontierConfig frontier_config;
  ae::SelectionConfig selection;
  ae::GraphUncertainty graph;
  graph.positions = {{3.0, 3.0}, {7.0, 3.0}};
  graph.sigma = {0.0, 0.1};

  for (const bool doorway : {false, true}) {
    const auto walls = room(doorway);
    const ae::CoverageGrid grid = ae::buildCoverageGrid(
      {scanFrom({3.0, 3.0}, walls), scanFrom({7.0, 3.0}, walls)}, config);
    const ae::PathField paths(grid, config, planner, {7.0, 3.0});
    const auto frontiers = ae::findFrontiers(grid, config, paths, frontier_config);
    ae::ExplorerState state;
    const ae::Decision decision = ae::selectGoal(paths, frontiers, graph, selection, state);
    if (!doorway) {
      EXPECT_EQ(decision.status, ae::Decision::Status::EXPLORATION_DONE) << decision.reason;
      continue;
    }
    ASSERT_EQ(decision.status, ae::Decision::Status::GOAL) << decision.reason;
    const ae::GoalCandidate & goal = decision.candidates[static_cast<std::size_t>(
        decision.chosen)];
    EXPECT_EQ(goal.kind, ae::GoalKind::FRONTIER);
    // The doorway's frontier: at the gap in the right wall.
    EXPECT_NEAR(goal.goal.x(), 10.0, 0.6);
    EXPECT_NEAR(goal.goal.y(), 3.0, 0.6);
    EXPECT_GT(goal.unknown_area, 10.0);
    EXPECT_NEAR(goal.path_length, 3.0, 1.0);
  }
}

TEST(ActiveExploration, PathGoesAroundAWall)
{
  // An internal wall x = 5, y in [0, 4.5] splits the room; from (2, 1) to (8, 1) the path goes
  // over its end (about 2 x hypot(3, 3.5+) > 9 m), not straight (6 m).
  auto walls = room(false);
  walls.push_back({{5.0, 0.0}, {5.0, 4.5}});
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({2.0, 1.0}, walls), scanFrom({2.5, 5.2}, walls), scanFrom({7.5, 5.2}, walls),
      scanFrom({8.0, 1.0}, walls)}, config);
  ae::PlannerConfig planner;
  planner.inflation_radius = 0.3;
  const ae::PathField paths(grid, config, planner, {2.0, 1.0});
  const double length = paths.distanceTo({8.0, 1.0});
  ASSERT_TRUE(std::isfinite(length));
  EXPECT_GT(length, 9.0);
  EXPECT_LT(length, 14.0);
  EXPECT_FALSE(std::isfinite(paths.distanceTo({12.0, 1.0})));  // outside the room
}

TEST(ActiveExploration, RevisitsWhenUncertainAndReturnsToExploringWithHysteresis)
{
  const auto walls = room(true);
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({1.5, 1.5}, walls), scanFrom({5.0, 3.0}, walls), scanFrom({8.5, 4.5}, walls)},
    config);
  const ae::PathField paths(grid, config, ae::PlannerConfig(), {8.5, 4.5});
  const auto frontiers = ae::findFrontiers(grid, config, paths, ae::FrontierConfig());
  ae::SelectionConfig selection;
  selection.revisit_trigger_sigma = 1.0;
  selection.revisit_release_sigma = 0.5;
  selection.min_index_gap = 20;
  selection.uncertainty_weight = 100.0;

  // 30 submaps along the room; sigma grows from the fixed first one.
  ae::GraphUncertainty graph;
  for (int k = 0; k < 30; ++k) {
    graph.positions.emplace_back(1.5 + 7.0 * k / 29.0, 1.5 + 3.0 * k / 29.0);
    graph.sigma.push_back(0.05 * k);  // newest: 1.45
  }
  ae::ExplorerState state;
  ae::Decision decision = ae::selectGoal(paths, frontiers, graph, selection, state);
  ASSERT_EQ(decision.status, ae::Decision::Status::GOAL) << decision.reason;
  EXPECT_TRUE(decision.revisit_mode);
  const ae::GoalCandidate & revisit = decision.candidates[static_cast<std::size_t>(
      decision.chosen)];
  EXPECT_EQ(revisit.kind, ae::GoalKind::REVISIT);
  EXPECT_LE(revisit.source, 9);  // older than min_index_gap, sigma <= 0.5 x newest
  EXPECT_GT(revisit.uncertainty_drop, 1.0);

  // Between release and trigger: stays in revisit mode.
  graph.sigma.back() = 0.8;
  decision = ae::selectGoal(paths, frontiers, graph, selection, state);
  EXPECT_TRUE(decision.revisit_mode);
  // Below release (a loop closed): explores the doorway again.
  graph.sigma.back() = 0.3;
  decision = ae::selectGoal(paths, frontiers, graph, selection, state);
  EXPECT_FALSE(decision.revisit_mode);
  ASSERT_EQ(decision.status, ae::Decision::Status::GOAL);
  EXPECT_EQ(decision.candidates[static_cast<std::size_t>(decision.chosen)].kind,
    ae::GoalKind::FRONTIER);
  // And below the trigger from exploring: does not enter revisit mode.
  graph.sigma.back() = 0.8;
  decision = ae::selectGoal(paths, frontiers, graph, selection, state);
  EXPECT_FALSE(decision.revisit_mode);
}

TEST(ActiveExploration, CloserFrontierWinsWhenGainsAreEqual)
{
  // Doorways in the left and the right wall; the robot stands near the right one.
  std::vector<Segment> walls{
    {{0, 0}, {10, 0}}, {{0, 6}, {10, 6}}, {{0, 0}, {0, 2}}, {{0, 4}, {0, 6}},
    {{10, 0}, {10, 2}}, {{10, 4}, {10, 6}}};
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({3.0, 3.0}, walls), scanFrom({7.0, 3.0}, walls)}, config);
  const ae::PathField paths(grid, config, ae::PlannerConfig(), {8.0, 3.0});
  const auto frontiers = ae::findFrontiers(grid, config, paths, ae::FrontierConfig());
  ae::GraphUncertainty graph;
  graph.positions = {{3.0, 3.0}, {8.0, 3.0}};
  graph.sigma = {0.0, 0.1};
  ae::ExplorerState state;
  const ae::Decision decision =
    ae::selectGoal(paths, frontiers, graph, ae::SelectionConfig(), state);
  ASSERT_EQ(decision.status, ae::Decision::Status::GOAL);
  int doorways = 0;
  for (const auto & c : decision.candidates) {
    doorways += c.kind == ae::GoalKind::FRONTIER ? 1 : 0;
  }
  EXPECT_EQ(doorways, 2);
  EXPECT_GT(decision.candidates[static_cast<std::size_t>(decision.chosen)].goal.x(), 9.0);
}

TEST(ActiveExploration, UnknownBeyondAWallIsNoGainAndAReachedFrontierIsDropped)
{
  // The doorway room: the unknown outside the walls is not in sight of a frontier there, so the
  // doorway's gain is the unknown seen through the opening only. A frontier the robot stands
  // next to that is still a frontier is marked visited and not chosen again.
  const auto walls = room(true);
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({3.0, 3.0}, walls), scanFrom({7.0, 3.0}, walls)}, config);
  {
    const ae::PathField paths(grid, config, ae::PlannerConfig(), {7.0, 3.0});
    const auto frontiers = ae::findFrontiers(grid, config, paths, ae::FrontierConfig());
    ASSERT_EQ(frontiers.size(), 1U);
    // A 4 m disc is 50 m^2; behind the 2 m doorway the visible unknown is a wedge of it.
    EXPECT_LT(frontiers[0].unknown_area, 20.0);
    EXPECT_GT(frontiers[0].unknown_area, 3.0);
  }
  ae::GraphUncertainty graph;
  graph.positions = {{3.0, 3.0}, {9.0, 3.0}};
  graph.sigma = {0.0, 0.1};
  ae::ExplorerState state;
  const ae::PathField near(grid, config, ae::PlannerConfig(), {8.8, 3.0});
  const auto frontiers = ae::findFrontiers(grid, config, near, ae::FrontierConfig());
  ae::Decision decision = ae::selectGoal(near, frontiers, graph, ae::SelectionConfig(), state);
  EXPECT_EQ(decision.status, ae::Decision::Status::EXPLORATION_DONE) << decision.reason;
  EXPECT_EQ(state.visited_frontiers.size(), 1U);
  // From afar it stays visited.
  const ae::PathField far(grid, config, ae::PlannerConfig(), {3.0, 3.0});
  decision = ae::selectGoal(
    far, ae::findFrontiers(grid, config, far, ae::FrontierConfig()), graph,
    ae::SelectionConfig(), state);
  EXPECT_EQ(decision.status, ae::Decision::Status::EXPLORATION_DONE);
}

TEST(ActiveExploration, PredictedLoopGainPicksTheTargetThatHelpsAndTriedTargetsAreDropped)
{
  const auto walls = room(false);
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({1.5, 1.5}, walls), scanFrom({5.0, 3.0}, walls), scanFrom({8.5, 4.5}, walls)},
    config);
  ae::SelectionConfig selection;
  selection.revisit_trigger_sigma = 1.0;
  selection.revisit_release_sigma = 0.5;
  selection.min_index_gap = 20;
  selection.uncertainty_weight = 20.0;
  selection.distance_weight = 0.5;
  ae::GraphUncertainty graph;
  for (int k = 0; k < 30; ++k) {
    graph.positions.emplace_back(1.5 + 7.0 * k / 29.0, 1.5 + 3.0 * k / 29.0);
    graph.sigma.push_back(0.05 * k);  // newest: 1.45
  }
  // A loop edge to the start (k <= 2) would remove 90 % of the newest sigma, to k = 9 only 10 %
  // (below min_predicted_drop), to the others 40 %.
  graph.predicted_sigma.assign(30, 0.6 * 1.45);
  for (int k = 0; k <= 2; ++k) {
    graph.predicted_sigma[static_cast<std::size_t>(k)] = 0.1 * 1.45;
  }
  graph.predicted_sigma[9] = 0.9 * 1.45;

  ae::ExplorerState state;
  const ae::PathField far(grid, config, ae::PlannerConfig(), {8.5, 4.5});
  ae::Decision decision = ae::selectGoal(far, {}, graph, selection, state);
  ASSERT_EQ(decision.status, ae::Decision::Status::GOAL) << decision.reason;
  const ae::GoalCandidate & best = decision.candidates[static_cast<std::size_t>(decision.chosen)];
  EXPECT_EQ(best.kind, ae::GoalKind::REVISIT);
  EXPECT_LE(best.source, 2);
  EXPECT_NEAR(best.uncertainty_drop, 0.9, 1e-9);
  for (const auto & c : decision.candidates) {
    EXPECT_NE(c.source, 9);  // predicted drop below min_predicted_drop
  }
  {
    // absolute_gain: the same targets, scored by the sigma removed in metres (0.9 x 1.45 m);
    // min_predicted_drop stays a fraction.
    ae::SelectionConfig absolute = selection;
    absolute.absolute_gain = true;
    ae::ExplorerState fresh;
    const ae::Decision d = ae::selectGoal(far, {}, graph, absolute, fresh);
    ASSERT_EQ(d.status, ae::Decision::Status::GOAL) << d.reason;
    const ae::GoalCandidate & c = d.candidates[static_cast<std::size_t>(d.chosen)];
    EXPECT_LE(c.source, 2);
    EXPECT_NEAR(c.uncertainty_drop, 0.9 * 1.45, 1e-9);
    for (const auto & other : d.candidates) {
      EXPECT_NE(other.source, 9);
    }
  }

  // The robot reached the start area and no loop edge came: those targets are tried, and the
  // next choice is elsewhere.
  const ae::PathField at_start(grid, config, ae::PlannerConfig(), {1.6, 1.6});
  decision = ae::selectGoal(at_start, {}, graph, selection, state);
  EXPECT_FALSE(state.tried_revisits.empty());
  for (const auto & c : decision.candidates) {
    EXPECT_GT(c.source, 2) << "a tried target was offered again";
  }
  // Leaving revisit mode forgets them.
  graph.sigma.back() = 0.2;
  decision = ae::selectGoal(at_start, {}, graph, selection, state);
  EXPECT_FALSE(decision.revisit_mode);
  EXPECT_TRUE(state.tried_revisits.empty());
}

TEST(ActiveExploration, RevisitsRespectTheLoopSearchTravelGapAndCompeteWithFrontiers)
{
  const auto walls = room(true);
  const ae::CoverageGridConfig config = gridConfig();
  const ae::CoverageGrid grid = ae::buildCoverageGrid(
    {scanFrom({1.5, 1.5}, walls), scanFrom({5.0, 3.0}, walls), scanFrom({8.5, 4.5}, walls)},
    config);
  const ae::PathField paths(grid, config, ae::PlannerConfig(), {8.5, 4.5});
  const auto frontiers = ae::findFrontiers(grid, config, paths, ae::FrontierConfig());
  ASSERT_FALSE(frontiers.empty());
  ae::GraphUncertainty graph;
  for (int k = 0; k < 30; ++k) {
    graph.positions.emplace_back(1.5 + 7.0 * k / 29.0, 1.5 + 3.0 * k / 29.0);
    graph.sigma.push_back(0.05 * k);
    graph.travel.push_back(2.0 * k);  // newest: 58 m
  }
  graph.predicted_sigma.assign(30, 0.1 * 1.45);  // any loop edge would remove 90 %
  ae::SelectionConfig selection;
  selection.revisit_trigger_sigma = 1.0;
  selection.revisit_release_sigma = 0.5;
  selection.min_index_gap = 1;
  selection.min_travel_gap = 40.0;  // only submaps 0..8 (travel <= 16 m) are far enough back
  selection.uncertainty_weight = 1000.0;
  ae::ExplorerState state;
  ae::Decision decision = ae::selectGoal(paths, frontiers, graph, selection, state);
  ASSERT_EQ(decision.status, ae::Decision::Status::GOAL);
  for (const auto & c : decision.candidates) {
    if (c.kind == ae::GoalKind::REVISIT) {
      EXPECT_LE(c.source, 8);
    }
  }
  EXPECT_EQ(decision.candidates[static_cast<std::size_t>(decision.chosen)].kind,
    ae::GoalKind::REVISIT);
  // With a small revisit weight the doorway's frontier has the better utility, also in revisit
  // mode.
  selection.uncertainty_weight = 1.0;
  decision = ae::selectGoal(paths, frontiers, graph, selection, state);
  EXPECT_TRUE(decision.revisit_mode);
  EXPECT_EQ(decision.candidates[static_cast<std::size_t>(decision.chosen)].kind,
    ae::GoalKind::FRONTIER);
}

TEST(ActiveExploration, DecisionIsDeterministic)
{
  const auto walls = room(true);
  const ae::CoverageGridConfig config = gridConfig();
  auto decide = [&]() {
      const ae::CoverageGrid grid = ae::buildCoverageGrid(
        {scanFrom({3.0, 3.0}, walls), scanFrom({7.0, 2.0}, walls)}, config);
      const ae::PathField paths(grid, config, ae::PlannerConfig(), {7.0, 2.0});
      const auto frontiers = ae::findFrontiers(grid, config, paths, ae::FrontierConfig());
      ae::GraphUncertainty graph;
      graph.positions = {{3.0, 3.0}, {7.0, 2.0}};
      graph.sigma = {0.0, 0.2};
      ae::ExplorerState state;
      return ae::selectGoal(paths, frontiers, graph, ae::SelectionConfig(), state);
    };
  const ae::Decision a = decide();
  const ae::Decision b = decide();
  ASSERT_EQ(a.candidates.size(), b.candidates.size());
  EXPECT_EQ(a.chosen, b.chosen);
  for (std::size_t i = 0; i < a.candidates.size(); ++i) {
    EXPECT_EQ(a.candidates[i].goal, b.candidates[i].goal);
    EXPECT_EQ(a.candidates[i].utility, b.candidates[i].utility);
  }
}

}  // namespace
