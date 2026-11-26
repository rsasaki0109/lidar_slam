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

}  // namespace
