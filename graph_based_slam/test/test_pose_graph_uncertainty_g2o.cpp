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

// Cross-check of pose_graph_uncertainty.hpp against g2o itself: the marginal
// covariances it computes from g2o's saved text must match what g2o's
// SparseOptimizer::computeMarginals gives for the same optimized graph.

#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "graph_based_slam/pose_graph_optimization.hpp"
#include "graph_based_slam/pose_graph_uncertainty.hpp"

namespace
{

namespace pgu = graphslam::pose_graph_uncertainty;

TEST(PoseGraphUncertaintyG2o, MatchesG2oComputeMarginals)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);

  g2o::SparseOptimizer optimizer;
  auto linear_solver =
    std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  optimizer.setAlgorithm(
    new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linear_solver))));

  // A helix-like trajectory (yaw and roll every step), perturbed start values.
  constexpr int kVertices = 40;
  std::vector<Eigen::Isometry3d> truth;
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
  step.linear() = (Eigen::AngleAxisd(0.16, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX())).toRotationMatrix();
  step.translation() = Eigen::Vector3d(1.0, 0.1, 0.02);
  for (int i = 0; i < kVertices; ++i) {
    auto * vertex = new g2o::VertexSE3();
    vertex->setId(i);
    Eigen::Isometry3d start = current;
    start.translation() += 0.05 * Eigen::Vector3d(uniform(rng), uniform(rng), uniform(rng));
    vertex->setEstimate(start);
    vertex->setFixed(i == 0);
    optimizer.addVertex(vertex);
    truth.push_back(current);
    current = current * step;
  }
  auto add_edge = [&](int from, int to, double diagonal) {
      auto * edge = new g2o::EdgeSE3();
      Eigen::Isometry3d measurement = truth[from].inverse() * truth[to];
      measurement.translation() +=
        0.01 * Eigen::Vector3d(uniform(rng), uniform(rng), uniform(rng));
      edge->setMeasurement(measurement);
      pgu::Matrix6d a;
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          a(r, c) = uniform(rng);
        }
      }
      // Full (correlated) information, as loop registration reports it.
      edge->setInformation(0.2 * a * a.transpose() + diagonal * pgu::Matrix6d::Identity());
      edge->vertices()[0] = optimizer.vertex(from);
      edge->vertices()[1] = optimizer.vertex(to);
      optimizer.addEdge(edge);
    };
  for (int i = 1; i < kVertices; ++i) {
    add_edge(i - 1, i, 100.0);
  }
  add_edge(0, 39, 50.0);
  add_edge(5, 30, 80.0);
  add_edge(12, 25, 30.0);
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  std::vector<std::pair<int, int>> blocks;
  for (int i = 1; i < kVertices; ++i) {
    const int h = optimizer.vertex(i)->hessianIndex();
    blocks.emplace_back(h, h);
  }
  g2o::SparseBlockMatrix<Eigen::MatrixXd> marginals;
  ASSERT_TRUE(optimizer.computeMarginals(marginals, blocks));

  std::ostringstream saved;
  ASSERT_TRUE(optimizer.save(saved));
  std::istringstream text(saved.str());
  pgu::ParsedGraph graph;
  std::string error;
  ASSERT_TRUE(pgu::parseG2o(text, graph, error)) << error;
  const pgu::UncertaintyReport report = pgu::computeUncertainty(graph);
  ASSERT_TRUE(report.ok) << report.error;
  ASSERT_EQ(report.per_vertex.size(), static_cast<std::size_t>(kVertices));
  EXPECT_TRUE(report.per_vertex[0].fixed);
  EXPECT_FALSE(report.gauge_fixed_first_vertex);

  for (int i = 1; i < kVertices; ++i) {
    const int h = optimizer.vertex(i)->hessianIndex();
    const Eigen::MatrixXd reference = *marginals.block(h, h);
    const pgu::Matrix6d & ours = report.per_vertex[static_cast<std::size_t>(i)].covariance;
    // The saved text rounds poses and measurements; 1e-3 leaves room for that.
    EXPECT_LT((reference - ours).norm() / reference.norm(), 1e-3) << "vertex " << i;
  }
}

// A backend graph: 60 submaps, straight then turning, one loop edge.
struct BackendGraph
{
  std::vector<graphslam::pose_graph::SubmapNode> nodes;
  std::vector<graphslam::pose_graph::LoopConstraint> loops;
};

BackendGraph backendGraph(double gravity_prior_info)
{
  BackendGraph graph;
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 60; ++i) {
    graphslam::pose_graph::SubmapNode node;
    node.pose = current;
    if (gravity_prior_info > 0.0 && i % 5 == 0) {
      node.gravity_prior_info = gravity_prior_info;
      node.gravity_up_body = node.pose.linear().transpose() * Eigen::Vector3d::UnitZ();
    }
    graph.nodes.push_back(node);
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    step.linear() =
      Eigen::AngleAxisd(i < 30 ? 0.0 : 0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    step.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
    current = current * step;
  }
  graphslam::pose_graph::LoopConstraint loop;
  loop.from = 2;
  loop.to = 58;
  loop.relative_pose = graph.nodes[2].pose.inverse() * graph.nodes[58].pose;
  loop.fitness_score = 0.1;
  graph.loops.push_back(loop);
  return graph;
}

graphslam::pose_graph::OptimizationResult optimize(
  const BackendGraph & graph, bool compute_marginals)
{
  using namespace graphslam::pose_graph;  // NOLINT(build/namespaces)
  return optimizePoseGraph(
    graph.nodes, graph.loops, {}, {}, AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{},
    Chi2Collection::NONE, true, 10, {}, 0, compute_marginals);
}

TEST(PoseGraphOptimizationMarginals, OffByDefaultAndPosesUnchanged)
{
  const BackendGraph graph = backendGraph(0.0);
  const auto without = optimize(graph, false);
  const auto with = optimize(graph, true);
  EXPECT_FALSE(without.marginals_valid);
  EXPECT_TRUE(without.marginal_covariances.empty());
  ASSERT_TRUE(with.marginals_valid);
  ASSERT_EQ(with.poses.size(), without.poses.size());
  for (std::size_t i = 0; i < with.poses.size(); ++i) {
    EXPECT_EQ(with.poses[i].matrix(), without.poses[i].matrix()) << "pose " << i;
  }
  EXPECT_EQ(with.pose_graph_g2o, without.pose_graph_g2o);
}

TEST(PoseGraphOptimizationMarginals, MatchTheReportOfTheSavedGraph)
{
  // Without gravity priors every edge is an EDGE_SE3:QUAT in the saved text,
  // so the in-optimizer marginals and the file report must agree.
  const auto result = optimize(backendGraph(0.0), true);
  ASSERT_TRUE(result.marginals_valid);
  const pgu::UncertaintyReport live =
    pgu::reportFromMarginals(result.poses, result.marginal_covariances);
  std::istringstream text(result.pose_graph_g2o);
  pgu::ParsedGraph parsed;
  std::string error;
  ASSERT_TRUE(pgu::parseG2o(text, parsed, error)) << error;
  const pgu::UncertaintyReport saved = pgu::computeUncertainty(parsed);
  ASSERT_TRUE(live.ok && saved.ok);
  ASSERT_EQ(live.per_vertex.size(), saved.per_vertex.size());
  EXPECT_TRUE(live.per_vertex[0].fixed);
  for (std::size_t i = 1; i < live.per_vertex.size(); ++i) {
    const pgu::Matrix6d & a = live.per_vertex[i].covariance;
    const pgu::Matrix6d & b = saved.per_vertex[i].covariance;
    EXPECT_LT((a - b).norm() / b.norm(), 1e-3) << "vertex " << i;
  }
}

TEST(PoseGraphOptimizationMarginals, IncludeGravityPriorsTheSavedGraphLacks)
{
  const auto without = optimize(backendGraph(0.0), true);
  const auto with = optimize(backendGraph(1000.0), true);
  ASSERT_TRUE(without.marginals_valid && with.marginals_valid);
  const auto last_without =
    pgu::reportFromMarginals(without.poses, without.marginal_covariances).per_vertex.back();
  const auto last_with =
    pgu::reportFromMarginals(with.poses, with.marginal_covariances).per_vertex.back();
  // Gravity priors hold roll and pitch, so the far end's vertical uncertainty
  // (the weakest direction without them) shrinks.
  EXPECT_NEAR(std::abs(last_without.weakest_translation_direction.z()), 1.0, 1e-3);
  EXPECT_LT(last_with.covariance(2, 2), 0.5 * last_without.covariance(2, 2));
}

TEST(PoseGraphOptimizationMarginals, PredictedLoopEffectMatchesAddingTheLoop)
{
  // The newest submap's covariance predicted from the joint covariance before a loop edge must
  // equal the one computed after adding that edge (zero error, so the poses do not move).
  const BackendGraph graph = backendGraph(0.0);
  graphslam::pose_graph::LoopEdgeConfig loop_cfg;
  const auto before = optimize(graph, true);
  ASSERT_TRUE(before.marginals_valid);
  ASSERT_EQ(before.newest_cross_covariances.size(), before.poses.size());
  const std::size_t n = before.poses.size() - 1;
  for (const std::size_t k : {std::size_t{0}, std::size_t{5}, std::size_t{30}, std::size_t{45}}) {
    BackendGraph with_loop = graph;
    // The loop edges of the base graph, and a new one k -> n measured as the current estimate.
    for (std::size_t i = 0; i < with_loop.nodes.size(); ++i) {
      with_loop.nodes[i].pose = before.poses[i];
    }
    for (auto & loop : with_loop.loops) {
      loop.relative_pose = before.poses[static_cast<std::size_t>(loop.from)].inverse() *
        before.poses[static_cast<std::size_t>(loop.to)];
    }
    graphslam::pose_graph::LoopConstraint added;
    added.from = static_cast<int>(k);
    added.to = static_cast<int>(n);
    added.relative_pose = before.poses[k].inverse() * before.poses[n];
    added.fitness_score = 0.2;
    with_loop.loops.push_back(added);
    const auto after = optimize(with_loop, true);
    ASSERT_TRUE(after.marginals_valid);
    const pgu::Matrix6d predicted = pgu::predictedCovarianceAfterLoop(
      before.poses[k], before.poses[n], before.marginal_covariances[k],
      before.marginal_covariances[n], before.newest_cross_covariances[k],
      graphslam::pose_graph::loopEdgeInformation(loop_cfg, 0.2));
    const pgu::Matrix6d & actual = after.marginal_covariances[n];
    EXPECT_LT((predicted - actual).norm() / actual.norm(), 2e-3) << "k = " << k;
    EXPECT_LT(pgu::sigmaTranslationMax(predicted),
      pgu::sigmaTranslationMax(before.marginal_covariances[n])) << "k = " << k;
  }
}

}  // namespace
