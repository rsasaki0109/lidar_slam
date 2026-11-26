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

#ifndef GRAPH_BASED_SLAM__POSE_GRAPH_UNCERTAINTY_HPP_
#define GRAPH_BASED_SLAM__POSE_GRAPH_UNCERTAINTY_HPP_

// Per-vertex marginal pose uncertainty of a saved pose graph
// (docs/roadmap/active-slam-2026-11.md, P1). Pure and ROS-free: it reads the
// g2o text the backend writes to the map bundle (pose_graph.g2o), linearizes
// every EDGE_SE3:QUAT at the saved (optimized) estimates exactly as g2o's
// EdgeSE3 does, builds the Gauss-Newton information matrix H = sum J^T W J
// over the free vertices, and reports for each vertex the diagonal 6x6 block
// of H^-1 (its marginal covariance) and, for the graph, log det H
// (D-optimality).
//
// What the numbers mean: the backend's adjacent-edge information is a tuning
// weight (AdjacentEdgeConfig::info_weight), not a calibrated covariance, so
// the sigmas are in the units those weights imply. They rank vertices (which
// part of the trajectory is least constrained, in which direction) and
// compare graphs built with the same weights; they are not metres of error.
// Robust kernels are not in the g2o text and are ignored. Edge types other
// than EDGE_SE3:QUAT that are in the text are counted in
// ParsedGraph::ignored_tags; the backend's own edge types (GravityPriorEdge,
// PlaneRevisitEdge) are not registered with g2o's factory, so g2o does not
// write them at all and nothing here can see or count them. With the default
// parameters (gravity_prior_info 0, plane revisit off) the saved graph is
// complete; with either on, the report covers the graph without those edges
// (it overstates the uncertainty they would reduce, e.g. vertical with
// gravity priors off the file).
//
// Determinism: no randomness, no wall clock; vertices and edges are processed
// in file order and the sparse factorization uses a fixed (AMD) ordering, so
// the same file gives the same numbers.

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <Eigen/SparseCholesky>  // NOLINT(build/include_order)
#include <Eigen/SparseCore>  // NOLINT(build/include_order)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <istream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace graphslam
{
namespace pose_graph_uncertainty
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Vertex
{
  int id {0};
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
  bool fixed {false};
};

struct Edge
{
  int from {0};
  int to {0};
  Eigen::Isometry3d measurement {Eigen::Isometry3d::Identity()};
  // In g2o's EdgeSE3 error coordinates: translation, then the vector part of
  // the unit quaternion (about half the rotation angle).
  Matrix6d information {Matrix6d::Identity()};
};

struct ParsedGraph
{
  std::vector<Vertex> vertices;  // in file order
  std::vector<Edge> edges;  // in file order
  // Tag -> number of lines with a tag this module does not use.
  std::map<std::string, std::size_t> ignored_tags;
};

// g2o's VertexSE3/EdgeSE3 minimal coordinates (g2o internal::toVectorMQT and
// fromVectorMQT): translation and the quaternion's vector part, w >= 0.
inline Vector6d toVectorMQT(const Eigen::Isometry3d & t)
{
  Eigen::Quaterniond q(t.linear());
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  Vector6d v;
  v.head<3>() = t.translation();
  v.tail<3>() = q.vec();
  return v;
}

inline Eigen::Isometry3d fromVectorMQT(const Vector6d & v)
{
  Eigen::Vector3d qv = v.tail<3>();
  double w = 1.0 - qv.squaredNorm();
  if (w < 0.0) {
    qv.normalize();
    w = 0.0;
  } else {
    w = std::sqrt(w);
  }
  const Eigen::Quaterniond q(w, qv.x(), qv.y(), qv.z());
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = v.head<3>();
  return t;
}

// The error g2o's EdgeSE3 computes for this edge at these vertex poses.
inline Vector6d edgeError(
  const Eigen::Isometry3d & measurement, const Eigen::Isometry3d & from,
  const Eigen::Isometry3d & to)
{
  return toVectorMQT(measurement.inverse() * from.inverse() * to);
}

namespace detail
{

inline Eigen::Isometry3d poseFromTranslationQuaternion(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

// Central-difference Jacobians of edgeError with respect to a right
// perturbation of each vertex (g2o VertexSE3::oplusImpl:
// X <- X * fromVectorMQT(dx)).
inline void edgeJacobians(
  const Edge & edge, const Eigen::Isometry3d & from, const Eigen::Isometry3d & to,
  Matrix6d & jacobian_from, Matrix6d & jacobian_to)
{
  constexpr double kStep = 1e-6;
  for (int k = 0; k < 6; ++k) {
    Vector6d dx = Vector6d::Zero();
    dx(k) = kStep;
    const Eigen::Isometry3d plus = fromVectorMQT(dx);
    dx(k) = -kStep;
    const Eigen::Isometry3d minus = fromVectorMQT(dx);
    jacobian_from.col(k) =
      (edgeError(edge.measurement, from * plus, to) -
      edgeError(edge.measurement, from * minus, to)) / (2.0 * kStep);
    jacobian_to.col(k) =
      (edgeError(edge.measurement, from, to * plus) -
      edgeError(edge.measurement, from, to * minus)) / (2.0 * kStep);
  }
}

}  // namespace detail

// Parses g2o text: VERTEX_SE3:QUAT, EDGE_SE3:QUAT and FIX lines; any other tag
// is counted in ignored_tags. Returns false (with a message naming the line)
// on a malformed known line, a duplicate vertex id, or an edge or FIX naming
// a vertex that is not defined.
inline bool parseG2o(std::istream & input, ParsedGraph & graph, std::string & error)
{
  graph = ParsedGraph();
  std::map<int, std::size_t> index_of;
  std::vector<int> fixed_ids;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    std::istringstream fields(line);
    std::string tag;
    if (!(fields >> tag) || tag[0] == '#') {
      continue;
    }
    const std::string where = "line " + std::to_string(line_number) + " (" + tag + ")";
    if (tag == "VERTEX_SE3:QUAT") {
      Vertex vertex;
      double x, y, z, qx, qy, qz, qw;
      if (!(fields >> vertex.id >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        error = where + ": expected id x y z qx qy qz qw";
        return false;
      }
      if (index_of.count(vertex.id) != 0) {
        error = where + ": duplicate vertex id " + std::to_string(vertex.id);
        return false;
      }
      vertex.pose = detail::poseFromTranslationQuaternion(x, y, z, qx, qy, qz, qw);
      index_of[vertex.id] = graph.vertices.size();
      graph.vertices.push_back(vertex);
    } else if (tag == "EDGE_SE3:QUAT") {
      Edge edge;
      double x, y, z, qx, qy, qz, qw;
      if (!(fields >> edge.from >> edge.to >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        error = where + ": expected from to x y z qx qy qz qw";
        return false;
      }
      edge.measurement = detail::poseFromTranslationQuaternion(x, y, z, qx, qy, qz, qw);
      for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
          if (!(fields >> edge.information(i, j))) {
            error = where + ": expected 21 upper-triangular information entries";
            return false;
          }
          edge.information(j, i) = edge.information(i, j);
        }
      }
      graph.edges.push_back(edge);
    } else if (tag == "FIX") {
      int id = 0;
      while (fields >> id) {
        fixed_ids.push_back(id);
      }
    } else {
      ++graph.ignored_tags[tag];
    }
  }
  for (const Edge & edge : graph.edges) {
    if (index_of.count(edge.from) == 0 || index_of.count(edge.to) == 0) {
      error = "edge " + std::to_string(edge.from) + " -> " + std::to_string(edge.to) +
        " names a vertex that is not defined";
      return false;
    }
  }
  for (const int id : fixed_ids) {
    const auto it = index_of.find(id);
    if (it == index_of.end()) {
      error = "FIX names vertex " + std::to_string(id) + ", which is not defined";
      return false;
    }
    graph.vertices[it->second].fixed = true;
  }
  return true;
}

struct VertexUncertainty
{
  int id {0};
  bool fixed {false};
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  // Marginal covariance in g2o's minimal coordinates (zero for a fixed vertex).
  Matrix6d covariance {Matrix6d::Zero()};
  // Square roots of the largest / smallest eigenvalue of the translation block.
  double sigma_translation_max {0.0};
  double sigma_translation_min {0.0};
  // Unit direction (vertex frame) of sigma_translation_max; its largest
  // component is positive so the sign is reproducible.
  Eigen::Vector3d weakest_translation_direction {Eigen::Vector3d::Zero()};
  // Largest rotation standard deviation [rad] (twice the quaternion-vector sigma).
  double sigma_rotation_max {0.0};
  // log det of the 6x6 marginal covariance (D-optimality of this vertex alone).
  double log_det_covariance {0.0};
};

struct UncertaintyReport
{
  bool ok {false};
  std::string error;
  std::size_t vertices {0};
  std::size_t free_vertices {0};
  std::size_t edges {0};
  // True when the file fixed no vertex and the first vertex was held fixed
  // to remove the gauge freedom.
  bool gauge_fixed_first_vertex {false};
  // log det of the free vertices' information matrix, and the D-optimality
  // criterion exp(log_det_information / n) with n = 6 * free_vertices.
  double log_det_information {0.0};
  double d_optimality {0.0};
  std::vector<VertexUncertainty> per_vertex;  // in file order
};

namespace detail
{
inline Eigen::Vector3d canonicalDirection(const Eigen::Vector3d & direction)
{
  Eigen::Index largest = 0;
  direction.cwiseAbs().maxCoeff(&largest);
  return direction(largest) < 0.0 ? Eigen::Vector3d(-direction) : direction;
}
}  // namespace detail

// Fills the summary fields of a vertex from its covariance.
inline void summarizeCovariance(VertexUncertainty & vertex)
{
  using detail::canonicalDirection;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation(
    vertex.covariance.topLeftCorner<3, 3>());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation(
    vertex.covariance.bottomRightCorner<3, 3>());
  vertex.sigma_translation_max = std::sqrt(std::max(0.0, translation.eigenvalues()(2)));
  vertex.sigma_translation_min = std::sqrt(std::max(0.0, translation.eigenvalues()(0)));
  vertex.weakest_translation_direction = canonicalDirection(translation.eigenvectors().col(2));
  vertex.sigma_rotation_max = 2.0 * std::sqrt(std::max(0.0, rotation.eigenvalues()(2)));
  const Eigen::LLT<Matrix6d> llt(vertex.covariance);
  double log_det = 0.0;
  for (int i = 0; i < 6; ++i) {
    log_det += 2.0 * std::log(llt.matrixL()(i, i));
  }
  vertex.log_det_covariance = log_det;
}

// Computes the marginal covariance of every free vertex. A vertex not reached
// by any edge, or a direction no edge constrains, makes H singular; the
// report then has ok = false and says so.
inline UncertaintyReport computeUncertainty(const ParsedGraph & graph)
{
  UncertaintyReport report;
  report.vertices = graph.vertices.size();
  report.edges = graph.edges.size();
  if (graph.vertices.empty()) {
    report.error = "the graph has no VERTEX_SE3:QUAT";
    return report;
  }

  std::vector<bool> fixed(graph.vertices.size(), false);
  bool any_fixed = false;
  for (std::size_t i = 0; i < graph.vertices.size(); ++i) {
    fixed[i] = graph.vertices[i].fixed;
    any_fixed = any_fixed || fixed[i];
  }
  if (!any_fixed) {
    fixed[0] = true;
    report.gauge_fixed_first_vertex = true;
  }

  std::map<int, std::size_t> index_of;
  for (std::size_t i = 0; i < graph.vertices.size(); ++i) {
    index_of[graph.vertices[i].id] = i;
  }
  // Column block of each free vertex in H; -1 for a fixed vertex.
  std::vector<int> block(graph.vertices.size(), -1);
  int free_count = 0;
  for (std::size_t i = 0; i < graph.vertices.size(); ++i) {
    if (!fixed[i]) {
      block[i] = free_count++;
    }
  }
  report.free_vertices = static_cast<std::size_t>(free_count);

  report.per_vertex.resize(graph.vertices.size());
  for (std::size_t i = 0; i < graph.vertices.size(); ++i) {
    report.per_vertex[i].id = graph.vertices[i].id;
    report.per_vertex[i].fixed = fixed[i];
    report.per_vertex[i].position = graph.vertices[i].pose.translation();
  }
  if (free_count == 0) {
    report.ok = true;
    return report;
  }

  const int n = 6 * free_count;
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(graph.edges.size() * 144);
  for (const Edge & edge : graph.edges) {
    const std::size_t a = index_of.at(edge.from);
    const std::size_t b = index_of.at(edge.to);
    if (a == b) {
      continue;
    }
    Matrix6d jacobian_a, jacobian_b;
    detail::edgeJacobians(
      edge, graph.vertices[a].pose, graph.vertices[b].pose, jacobian_a, jacobian_b);
    const Matrix6d information = 0.5 * (edge.information + edge.information.transpose());
    const std::size_t ends[2] = {a, b};
    const Matrix6d * jacobians[2] = {&jacobian_a, &jacobian_b};
    for (int r = 0; r < 2; ++r) {
      if (block[ends[r]] < 0) {
        continue;
      }
      for (int c = 0; c < 2; ++c) {
        if (block[ends[c]] < 0) {
          continue;
        }
        const Matrix6d h = jacobians[r]->transpose() * information * *jacobians[c];
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) {
            triplets.emplace_back(6 * block[ends[r]] + i, 6 * block[ends[c]] + j, h(i, j));
          }
        }
      }
    }
  }
  Eigen::SparseMatrix<double> hessian(n, n);
  hessian.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt(hessian);
  if (llt.info() != Eigen::Success) {
    report.error =
      "the information matrix is not positive definite: some free vertex or direction is "
      "constrained by no edge";
    return report;
  }
  double log_det = 0.0;
  const Eigen::SparseMatrix<double> lower = llt.matrixL();
  for (int k = 0; k < n; ++k) {
    log_det += 2.0 * std::log(lower.coeff(k, k));
  }
  report.log_det_information = log_det;
  report.d_optimality = std::exp(log_det / static_cast<double>(n));

  // Diagonal blocks of H^-1, solved a few vertices at a time to bound memory.
  constexpr int kVerticesPerSolve = 16;
  std::vector<std::size_t> free_index(static_cast<std::size_t>(free_count));
  for (std::size_t i = 0; i < graph.vertices.size(); ++i) {
    if (block[i] >= 0) {
      free_index[static_cast<std::size_t>(block[i])] = i;
    }
  }
  for (int first = 0; first < free_count; first += kVerticesPerSolve) {
    const int count = std::min(kVerticesPerSolve, free_count - first);
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n, 6 * count);
    for (int v = 0; v < count; ++v) {
      rhs.block<6, 6>(6 * (first + v), 6 * v).setIdentity();
    }
    const Eigen::MatrixXd solution = llt.solve(rhs);
    for (int v = 0; v < count; ++v) {
      VertexUncertainty & out =
        report.per_vertex[free_index[static_cast<std::size_t>(first + v)]];
      const Matrix6d covariance = solution.block<6, 6>(6 * (first + v), 6 * v);
      out.covariance = 0.5 * (covariance + covariance.transpose());
      summarizeCovariance(out);
    }
  }
  report.ok = true;
  return report;
}

// The per-vertex report of marginal covariances computed elsewhere (e.g. by
// g2o in pose_graph::optimizePoseGraph with compute_marginals): vertex i has
// id i, the position of poses[i], and is fixed when its covariance is zero.
// Graph-level fields (log_det_information, d_optimality) are left at zero.
template<typename CovarianceVector>
UncertaintyReport reportFromMarginals(
  const std::vector<Eigen::Isometry3d> & poses, const CovarianceVector & covariances)
{
  UncertaintyReport report;
  if (poses.size() != covariances.size()) {
    report.error = "poses and covariances differ in count";
    return report;
  }
  report.vertices = poses.size();
  report.per_vertex.resize(poses.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    VertexUncertainty & v = report.per_vertex[i];
    v.id = static_cast<int>(i);
    v.position = poses[i].translation();
    v.covariance = covariances[i];
    v.fixed = v.covariance.isZero(0.0);
    if (!v.fixed) {
      ++report.free_vertices;
      summarizeCovariance(v);
    }
  }
  report.ok = true;
  return report;
}

namespace detail
{
inline void writeVertexRow(std::ostream & out, const VertexUncertainty & v)
{
  constexpr double kRadToDeg = 57.29577951308232;
  out << v.id << ',' << (v.fixed ? 1 : 0) << ',' << v.position.x() << ',' << v.position.y() <<
    ',' << v.position.z() << ',' << v.sigma_translation_max << ',' <<
    v.sigma_translation_min << ',' << v.weakest_translation_direction.x() << ',' <<
    v.weakest_translation_direction.y() << ',' << v.weakest_translation_direction.z() <<
    ',' << v.sigma_rotation_max * kRadToDeg << ',' << v.log_det_covariance << '\n';
}

constexpr const char * kVertexColumns =
  "id,fixed,x,y,z,sigma_translation_max,sigma_translation_min,"
  "weak_direction_x,weak_direction_y,weak_direction_z,sigma_rotation_max_deg,"
  "log_det_covariance";
}  // namespace detail

// One CSV row per vertex, in file order, fixed precision (byte-reproducible).
inline std::string formatCsv(const UncertaintyReport & report)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  out << detail::kVertexColumns << '\n';
  for (const VertexUncertainty & v : report.per_vertex) {
    detail::writeVertexRow(out, v);
  }
  return out.str();
}

// Timeline CSV (one block of rows per pose-graph state, e.g. after each
// causal optimization): the header, then timelineRows for each state.
inline std::string timelineCsvHeader()
{
  return std::string("step,stamp_sec,submaps,loop_edges,") + detail::kVertexColumns + '\n';
}

inline std::string timelineRows(
  std::size_t step, double stamp_sec, std::size_t loop_edges, const UncertaintyReport & report)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  for (const VertexUncertainty & v : report.per_vertex) {
    out << step << ',' << stamp_sec << ',' << report.per_vertex.size() << ',' << loop_edges << ',';
    detail::writeVertexRow(out, v);
  }
  return out.str();
}

// Summary of a report as YAML, fixed precision (byte-reproducible). Graph-level
// D-optimality is written only when it was computed (computeUncertainty).
inline std::string formatReportSummaryYaml(const UncertaintyReport & report)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  out << "ok: " << (report.ok ? "true" : "false") << '\n';
  if (!report.ok) {
    out << "error: \"" << report.error << "\"\n";
  }
  out << "vertices: " << report.vertices << '\n';
  out << "free_vertices: " << report.free_vertices << '\n';
  if (!report.ok) {
    return out.str();
  }
  const VertexUncertainty * worst = nullptr;
  for (const VertexUncertainty & v : report.per_vertex) {
    if (!v.fixed && (worst == nullptr || v.sigma_translation_max > worst->sigma_translation_max)) {
      worst = &v;
    }
  }
  if (worst != nullptr) {
    out << "max_sigma_translation: " << worst->sigma_translation_max << '\n';
    out << "max_sigma_translation_vertex: " << worst->id << '\n';
  }
  return out.str();
}

// Graph-level summary as YAML, fixed precision (byte-reproducible).
inline std::string formatSummaryYaml(const ParsedGraph & graph, const UncertaintyReport & report)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  out << "ok: " << (report.ok ? "true" : "false") << '\n';
  if (!report.ok) {
    out << "error: \"" << report.error << "\"\n";
  }
  out << "vertices: " << report.vertices << '\n';
  out << "free_vertices: " << report.free_vertices << '\n';
  out << "edges_se3: " << report.edges << '\n';
  out << "gauge_fixed_first_vertex: " << (report.gauge_fixed_first_vertex ? "true" : "false") <<
    '\n';
  out << "ignored_tags:";
  if (graph.ignored_tags.empty()) {
    out << " {}";
  }
  out << '\n';
  for (const auto & entry : graph.ignored_tags) {
    out << "  " << entry.first << ": " << entry.second << '\n';
  }
  if (!report.ok) {
    return out.str();
  }
  out << "log_det_information: " << report.log_det_information << '\n';
  out << "d_optimality: " << report.d_optimality << '\n';
  const VertexUncertainty * worst = nullptr;
  for (const VertexUncertainty & v : report.per_vertex) {
    if (!v.fixed && (worst == nullptr || v.sigma_translation_max > worst->sigma_translation_max)) {
      worst = &v;
    }
  }
  if (worst != nullptr) {
    out << "max_sigma_translation: " << worst->sigma_translation_max << '\n';
    out << "max_sigma_translation_vertex: " << worst->id << '\n';
  }
  return out.str();
}

}  // namespace pose_graph_uncertainty
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__POSE_GRAPH_UNCERTAINTY_HPP_
