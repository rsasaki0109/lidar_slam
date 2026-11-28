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

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "graph_based_slam/pose_graph_uncertainty.hpp"

namespace
{

using graphslam::pose_graph_uncertainty::Edge;
using graphslam::pose_graph_uncertainty::Matrix6d;
using graphslam::pose_graph_uncertainty::ParsedGraph;
using graphslam::pose_graph_uncertainty::UncertaintyReport;
using graphslam::pose_graph_uncertainty::Vertex;
using graphslam::pose_graph_uncertainty::computeUncertainty;
using graphslam::pose_graph_uncertainty::formatCsv;
using graphslam::pose_graph_uncertainty::parseG2o;

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

// A straight chain along x with one edge per consecutive pair, the first vertex fixed.
ParsedGraph chain(int length, double step, const Matrix6d & information)
{
  ParsedGraph graph;
  for (int i = 0; i < length; ++i) {
    Vertex vertex;
    vertex.id = i;
    vertex.pose = translation(step * i, 0.0, 0.0);
    vertex.fixed = (i == 0);
    graph.vertices.push_back(vertex);
  }
  for (int i = 1; i < length; ++i) {
    Edge edge;
    edge.from = i - 1;
    edge.to = i;
    edge.measurement = translation(step, 0.0, 0.0);
    edge.information = information;
    graph.edges.push_back(edge);
  }
  return graph;
}

TEST(PoseGraphUncertainty, SingleEdgeMarginalIsTheEdgeCovariance)
{
  Matrix6d information = Matrix6d::Zero();
  information.diagonal() << 100.0, 400.0, 25.0, 1000.0, 2000.0, 500.0;
  const UncertaintyReport report = computeUncertainty(chain(2, 0.0, information));
  ASSERT_TRUE(report.ok) << report.error;
  ASSERT_EQ(report.per_vertex.size(), 2U);
  EXPECT_TRUE(report.per_vertex[0].fixed);
  const Matrix6d expected = information.inverse();
  EXPECT_TRUE(report.per_vertex[1].covariance.isApprox(expected, 1e-6))
    << report.per_vertex[1].covariance;
  EXPECT_NEAR(report.per_vertex[1].sigma_translation_max, 0.2, 1e-6);  // 1/sqrt(25)
  EXPECT_NEAR(report.per_vertex[1].sigma_translation_min, 0.05, 1e-6);  // 1/sqrt(400)
  EXPECT_NEAR(report.per_vertex[1].weakest_translation_direction.z(), 1.0, 1e-9);
  EXPECT_NEAR(report.per_vertex[1].sigma_rotation_max, 2.0 / std::sqrt(500.0), 1e-6);
  EXPECT_NEAR(report.log_det_information, std::log(information.determinant()), 1e-6);
}

TEST(PoseGraphUncertainty, UncertaintyGrowsAlongAnOpenChain)
{
  const UncertaintyReport report =
    computeUncertainty(chain(10, 1.0, Matrix6d::Identity() * 100.0));
  ASSERT_TRUE(report.ok) << report.error;
  for (std::size_t i = 2; i < report.per_vertex.size(); ++i) {
    EXPECT_GT(
      report.per_vertex[i].sigma_translation_max,
      report.per_vertex[i - 1].sigma_translation_max);
  }
}

TEST(PoseGraphUncertainty, HeadingErrorMakesTheWeakDirectionLateral)
{
  // Rotation is the weak part of every edge; over a long straight chain the
  // heading error dominates, so the far end is least certain across the path
  // (y or z), not along it (x). This is the coupling a translation-only
  // measure misses.
  Matrix6d information = Matrix6d::Identity() * 1e4;
  information.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1e3;
  const UncertaintyReport report = computeUncertainty(chain(30, 1.0, information));
  ASSERT_TRUE(report.ok) << report.error;
  const auto & last = report.per_vertex.back();
  EXPECT_LT(std::abs(last.weakest_translation_direction.x()), 1e-3);
  EXPECT_GT(last.sigma_translation_max, 10.0 * last.sigma_translation_min);
}

TEST(PoseGraphUncertainty, LoopClosureReducesUncertaintyAndRaisesDOptimality)
{
  ParsedGraph open = chain(20, 1.0, Matrix6d::Identity() * 100.0);
  const UncertaintyReport before = computeUncertainty(open);
  ParsedGraph closed = open;
  Edge loop;
  loop.from = 0;
  loop.to = 19;
  loop.measurement = translation(19.0, 0.0, 0.0);
  loop.information = Matrix6d::Identity() * 100.0;
  closed.edges.push_back(loop);
  const UncertaintyReport after = computeUncertainty(closed);
  ASSERT_TRUE(before.ok && after.ok);
  EXPECT_LT(
    after.per_vertex.back().sigma_translation_max,
    0.5 * before.per_vertex.back().sigma_translation_max);
  EXPECT_GT(after.log_det_information, before.log_det_information);
  EXPECT_GT(after.d_optimality, before.d_optimality);
}

TEST(PoseGraphUncertainty, FixesTheFirstVertexWhenTheFileFixesNone)
{
  ParsedGraph graph = chain(3, 1.0, Matrix6d::Identity());
  graph.vertices[0].fixed = false;
  const UncertaintyReport report = computeUncertainty(graph);
  ASSERT_TRUE(report.ok) << report.error;
  EXPECT_TRUE(report.gauge_fixed_first_vertex);
  EXPECT_TRUE(report.per_vertex[0].fixed);
  EXPECT_EQ(report.free_vertices, 2U);
}

TEST(PoseGraphUncertainty, UnconstrainedVertexIsAnError)
{
  ParsedGraph graph = chain(3, 1.0, Matrix6d::Identity());
  Vertex orphan;
  orphan.id = 7;
  graph.vertices.push_back(orphan);
  const UncertaintyReport report = computeUncertainty(graph);
  EXPECT_FALSE(report.ok);
  EXPECT_NE(report.error.find("not positive definite"), std::string::npos);
}

TEST(PoseGraphUncertainty, ParsesG2oTextAndCountsOtherTags)
{
  std::istringstream text(
    "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n"
    "VERTEX_SE3:QUAT 1 1 0 0 0 0 0 1\n"
    "FIX 0\n"
    "EDGE_SE3:QUAT 0 1 1 0 0 0 0 0 1 "
    "100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 400 0 0 400 0 400\n"
    "EDGE_GRAVITY_PRIOR 1 0 0 1 1 0 0 1 0 1\n");
  ParsedGraph graph;
  std::string error;
  ASSERT_TRUE(parseG2o(text, graph, error)) << error;
  ASSERT_EQ(graph.vertices.size(), 2U);
  EXPECT_TRUE(graph.vertices[0].fixed);
  EXPECT_FALSE(graph.vertices[1].fixed);
  ASSERT_EQ(graph.edges.size(), 1U);
  EXPECT_DOUBLE_EQ(graph.edges[0].information(3, 3), 400.0);
  EXPECT_DOUBLE_EQ(graph.edges[0].information(0, 1), 0.0);
  EXPECT_EQ(graph.ignored_tags.at("EDGE_GRAVITY_PRIOR"), 1U);

  const UncertaintyReport report = computeUncertainty(graph);
  ASSERT_TRUE(report.ok) << report.error;
  EXPECT_NEAR(report.per_vertex[1].sigma_translation_max, 0.1, 1e-6);
}

TEST(PoseGraphUncertainty, RejectsMalformedAndDanglingLines)
{
  ParsedGraph graph;
  std::string error;
  std::istringstream short_edge(
    "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\nEDGE_SE3:QUAT 0 0 0 0 0 0 0 0 1 1 2 3\n");
  EXPECT_FALSE(parseG2o(short_edge, graph, error));
  EXPECT_NE(error.find("line 2"), std::string::npos);

  std::istringstream dangling(
    "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n"
    "EDGE_SE3:QUAT 0 5 0 0 0 0 0 0 1 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n");
  EXPECT_FALSE(parseG2o(dangling, graph, error));

  std::istringstream duplicate(
    "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\nVERTEX_SE3:QUAT 0 1 0 0 0 0 0 1\n");
  EXPECT_FALSE(parseG2o(duplicate, graph, error));
}

TEST(PoseGraphUncertainty, CsvIsReproducible)
{
  const ParsedGraph graph = chain(5, 1.0, Matrix6d::Identity() * 50.0);
  const std::string first = formatCsv(computeUncertainty(graph));
  const std::string second = formatCsv(computeUncertainty(graph));
  EXPECT_EQ(first, second);
  EXPECT_EQ(first.substr(0, 9), "id,fixed,");
  EXPECT_EQ(std::count(first.begin(), first.end(), '\n'), 6);
}

}  // namespace
