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

#ifndef GRAPH_BASED_SLAM__ACTIVE_EXPLORATION_HPP_
#define GRAPH_BASED_SLAM__ACTIVE_EXPLORATION_HPP_

// Where to go next while exploring an unknown place (docs/roadmap/active-slam-2026-11.md,
// P2/P3). Pure and ROS-free. From the submaps so far (pose and local points) and their pose
// uncertainty (pose_graph_uncertainty.hpp) it
//
//   1. builds a 2D coverage grid (unknown / free / occupied) by casting each point's ray from
//      its submap's sensor position, projected onto the ground plane;
//   2. finds frontiers (free cells next to unknown ones, clustered) and scores each by the
//      unknown area around it (what going there would reveal);
//   3. finds revisit goals: earlier, well-localized submaps the robot can drive back to, scored
//      by how much a loop closure there could lower the newest submap's uncertainty;
//   4. plans the path length to every goal on the free cells (obstacles inflated) and picks one
//      by utility = info_weight * unknown area + uncertainty_weight * uncertainty drop
//                    - distance_weight * path length,
//      revisiting only while the newest submap's sigma is above a trigger (with hysteresis).
//
// The goal is a 2D position in the map frame; driving there is left to a navigation stack.
// Sigmas are in the units of pose_graph_uncertainty (the backend's edge information weights),
// so the revisit thresholds are set in those units for a given parameter set.
// Deterministic: fixed iteration orders, ties broken by index.

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace graphslam
{
namespace active_exploration
{

// ---------------------------------------------------------------- coverage grid

enum class Cell : std::uint8_t
{
  UNKNOWN = 0,
  FREE = 1,
  OCCUPIED = 2,
};

struct CoverageGridConfig
{
  double resolution {0.5};  // [m] cell size
  // Points are classified by their height relative to their submap's sensor (z of the submap
  // pose): below ground_max_z they are ground (their whole ray is free space), between
  // obstacle_min_z and obstacle_max_z they are obstacles (free up to the cell before the hit),
  // above obstacle_max_z they are ignored (ceilings, overhangs the robot passes under).
  double ground_max_z {-0.4};
  double obstacle_min_z {-0.4};
  double obstacle_max_z {1.0};
  double max_range {20.0};  // [m] rays are cut here (horizontal distance)
  // A cell is occupied when its obstacle hits are at least this fraction of all rays that
  // reached it (hits + pass-throughs); it is free when only passed through.
  double occupied_ratio {0.25};
};

struct SubmapScan
{
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};  // sensor (submap) pose in the map
  std::vector<Eigen::Vector3d> points;  // in the submap (sensor) frame
};

namespace detail
{

// Cells of the 2D line from a to b (Bresenham), a included, b excluded.
inline void walkCells(
  Eigen::Vector2i a, const Eigen::Vector2i & b, const std::function<void(int, int)> & visit)
{
  const int dx = std::abs(b.x() - a.x());
  const int dy = -std::abs(b.y() - a.y());
  const int sx = a.x() < b.x() ? 1 : -1;
  const int sy = a.y() < b.y() ? 1 : -1;
  int err = dx + dy;
  while (a != b) {
    visit(a.x(), a.y());
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      a.x() += sx;
    }
    if (e2 <= dx) {
      err += dx;
      a.y() += sy;
    }
  }
}

}  // namespace detail

// Hit and pass-through counts per cell; grows to hold every scan added.
class CoverageGrid
{
public:
  explicit CoverageGrid(double resolution = 0.5)
  : resolution_(resolution) {}

  double resolution() const {return resolution_;}
  // Integer cell coordinates are global (cell (0, 0) has its corner at the world origin); the
  // stored block covers [x0, x0 + width) x [y0, y0 + height).
  int x0() const {return x0_;}
  int y0() const {return y0_;}
  int width() const {return width_;}
  int height() const {return height_;}
  bool empty() const {return width_ == 0;}
  bool inside(int x, int y) const
  {
    return x >= x0_ && y >= y0_ && x < x0_ + width_ && y < y0_ + height_;
  }
  // Index of an inside cell in a width x height array.
  int index(int x, int y) const {return (y - y0_) * width_ + (x - x0_);}

  Eigen::Vector2i cellOf(const Eigen::Vector2d & world) const
  {
    return Eigen::Vector2i(
      static_cast<int>(std::floor(world.x() / resolution_)),
      static_cast<int>(std::floor(world.y() / resolution_)));
  }

  Eigen::Vector2d centerOf(int x, int y) const
  {
    return resolution_ * Eigen::Vector2d(x + 0.5, y + 0.5);
  }

  // The state of the cell holding a world position; outside the grid: unknown.
  Cell stateAt(const Eigen::Vector2d & world, double occupied_ratio) const
  {
    const Eigen::Vector2i c = cellOf(world);
    return state(c.x(), c.y(), occupied_ratio);
  }

  // A cell's state; outside the grid: unknown. Occupied when its obstacle hits are at least
  // occupied_ratio of the rays that reached it, free when it was only passed through.
  Cell state(int x, int y, double occupied_ratio) const
  {
    if (!inside(x, y)) {
      return Cell::UNKNOWN;
    }
    const auto i = static_cast<std::size_t>(index(x, y));
    const double total = static_cast<double>(hits_[i]) + passes_[i];
    if (total <= 0.0) {
      return Cell::UNKNOWN;
    }
    return (hits_[i] > 0 && hits_[i] >= occupied_ratio * total) ? Cell::OCCUPIED : Cell::FREE;
  }

  // Adds one scan: each point's ray from the scan's sensor position, on the ground plane.
  void addScan(const SubmapScan & scan, const CoverageGridConfig & config)
  {
    const Eigen::Vector3d sensor = scan.pose.translation();
    const Eigen::Vector2i a = cellOf(sensor.head<2>());
    struct Ray
    {
      Eigen::Vector2i end;
      bool obstacle;
    };
    std::vector<Ray> rays;
    rays.reserve(scan.points.size());
    Eigen::Vector2i lo = a;
    Eigen::Vector2i hi = a;
    for (const Eigen::Vector3d & local : scan.points) {
      if (!local.allFinite()) {
        continue;
      }
      const Eigen::Vector3d world = scan.pose * local;
      const double rel_z = world.z() - sensor.z();
      bool obstacle = false;
      if (rel_z <= config.ground_max_z) {
        obstacle = false;
      } else if (rel_z >= config.obstacle_min_z && rel_z <= config.obstacle_max_z) {
        obstacle = true;
      } else {
        continue;
      }
      Eigen::Vector2d end = world.head<2>();
      const Eigen::Vector2d delta = end - sensor.head<2>();
      const double range = delta.norm();
      if (range > config.max_range) {
        end = sensor.head<2>() + delta * (config.max_range / range);
        obstacle = false;  // the hit is beyond the cut: only free space up to it
      }
      const Eigen::Vector2i b = cellOf(end);
      lo = lo.cwiseMin(b);
      hi = hi.cwiseMax(b);
      rays.push_back({b, obstacle});
    }
    if (rays.empty()) {
      return;
    }
    reserve(lo, hi);
    for (const Ray & ray : rays) {
      detail::walkCells(a, ray.end, [this](int x, int y) {++passes_[cell(x, y)];});
      if (ray.obstacle) {
        ++hits_[cell(ray.end.x(), ray.end.y())];
      } else {
        ++passes_[cell(ray.end.x(), ray.end.y())];
      }
    }
  }

private:
  std::size_t cell(int x, int y) const {return static_cast<std::size_t>(index(x, y));}

  // Grows the stored block (with a margin, so most scans fit without a copy) to hold lo..hi.
  void reserve(const Eigen::Vector2i & lo, const Eigen::Vector2i & hi)
  {
    if (!empty() && inside(lo.x(), lo.y()) && inside(hi.x(), hi.y())) {
      return;
    }
    constexpr int kMargin = 64;
    int nx0 = lo.x() - kMargin;
    int ny0 = lo.y() - kMargin;
    int nx1 = hi.x() + kMargin;
    int ny1 = hi.y() + kMargin;
    if (!empty()) {
      nx0 = std::min(nx0, x0_);
      ny0 = std::min(ny0, y0_);
      nx1 = std::max(nx1, x0_ + width_ - 1);
      ny1 = std::max(ny1, y0_ + height_ - 1);
    }
    const int nw = nx1 - nx0 + 1;
    const int nh = ny1 - ny0 + 1;
    std::vector<std::uint32_t> hits(static_cast<std::size_t>(nw) * nh, 0U);
    std::vector<std::uint32_t> passes(hits.size(), 0U);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const std::size_t from = static_cast<std::size_t>(y) * width_ + x;
        const std::size_t to =
          static_cast<std::size_t>(y + y0_ - ny0) * nw + (x + x0_ - nx0);
        hits[to] = hits_[from];
        passes[to] = passes_[from];
      }
    }
    x0_ = nx0;
    y0_ = ny0;
    width_ = nw;
    height_ = nh;
    hits_ = std::move(hits);
    passes_ = std::move(passes);
  }

  double resolution_ {0.5};
  int x0_ {0};
  int y0_ {0};
  int width_ {0};
  int height_ {0};
  std::vector<std::uint32_t> hits_;
  std::vector<std::uint32_t> passes_;
};

// The coverage grid of the scans.
inline CoverageGrid buildCoverageGrid(
  const std::vector<SubmapScan> & scans, const CoverageGridConfig & config)
{
  CoverageGrid grid(config.resolution);
  for (const SubmapScan & scan : scans) {
    grid.addScan(scan, config);
  }
  return grid;
}

// ---------------------------------------------------------------- planning on the grid

struct PlannerConfig
{
  // [m] cells this close to an occupied cell are not driven through (robot radius + margin),
  // except within this distance of the start, so a robot next to a wall can still leave.
  double inflation_radius {0.6};
  bool allow_unknown {false};  // drive through unknown cells (optimistic) or not
};

// Traversable cells and the path length [m] from a start cell to every cell (Dijkstra, 8-way).
class PathField
{
public:
  PathField(
    const CoverageGrid & grid, const CoverageGridConfig & grid_config,
    const PlannerConfig & config, const Eigen::Vector2d & start)
  : grid_(grid),
    distance_(static_cast<std::size_t>(grid.width()) * grid.height(),
      std::numeric_limits<double>::infinity())
  {
    const int w = grid.width();
    const int h = grid.height();
    const int gx0 = grid.x0();
    const int gy0 = grid.y0();
    std::vector<Cell> state(static_cast<std::size_t>(w) * h);
    for (int y = gy0; y < gy0 + h; ++y) {
      for (int x = gx0; x < gx0 + w; ++x) {
        state[static_cast<std::size_t>(grid.index(x, y))] =
          grid.state(x, y, grid_config.occupied_ratio);
      }
    }
    // Inflate obstacles.
    blocked_.assign(state.size(), false);
    const int reach = static_cast<int>(std::ceil(config.inflation_radius / grid.resolution()));
    const double reach2 = std::pow(config.inflation_radius / grid.resolution(), 2);
    for (int y = gy0; y < gy0 + h; ++y) {
      for (int x = gx0; x < gx0 + w; ++x) {
        const Cell c = state[static_cast<std::size_t>(grid.index(x, y))];
        if (c == Cell::UNKNOWN && !config.allow_unknown) {
          blocked_[static_cast<std::size_t>(grid.index(x, y))] = true;
        }
        if (c != Cell::OCCUPIED) {
          continue;
        }
        for (int oy = -reach; oy <= reach; ++oy) {
          for (int ox = -reach; ox <= reach; ++ox) {
            if (ox * ox + oy * oy <= reach2 && grid.inside(x + ox, y + oy)) {
              blocked_[static_cast<std::size_t>(grid.index(x + ox, y + oy))] = true;
            }
          }
        }
      }
    }
    const Eigen::Vector2i s = grid.cellOf(start);
    if (!grid.inside(s.x(), s.y())) {
      return;
    }
    // Free the inflation around the robot (not the obstacles themselves).
    for (int oy = -reach; oy <= reach; ++oy) {
      for (int ox = -reach; ox <= reach; ++ox) {
        const int x = s.x() + ox;
        const int y = s.y() + oy;
        if (ox * ox + oy * oy <= reach2 && grid.inside(x, y) &&
          state[static_cast<std::size_t>(grid.index(x, y))] == Cell::FREE)
        {
          blocked_[static_cast<std::size_t>(grid.index(x, y))] = false;
        }
      }
    }
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    distance_[static_cast<std::size_t>(grid.index(s.x(), s.y()))] = 0.0;
    open.emplace(0.0, grid.index(s.x(), s.y()));
    const double r = grid.resolution();
    while (!open.empty()) {
      const Item item = open.top();
      open.pop();
      const int i = item.second;
      if (item.first > distance_[static_cast<std::size_t>(i)]) {
        continue;
      }
      const int x = gx0 + i % w;
      const int y = gy0 + i / w;
      for (int oy = -1; oy <= 1; ++oy) {
        for (int ox = -1; ox <= 1; ++ox) {
          if ((ox == 0 && oy == 0) || !grid.inside(x + ox, y + oy)) {
            continue;
          }
          const int j = grid.index(x + ox, y + oy);
          if (blocked_[static_cast<std::size_t>(j)]) {
            continue;
          }
          if (ox != 0 && oy != 0 &&  // no corner cutting
            (blocked_[static_cast<std::size_t>(grid.index(x + ox, y))] ||
            blocked_[static_cast<std::size_t>(grid.index(x, y + oy))]))
          {
            continue;
          }
          const double d = item.first + ((ox != 0 && oy != 0) ? r * std::sqrt(2.0) : r);
          if (d < distance_[static_cast<std::size_t>(j)]) {
            distance_[static_cast<std::size_t>(j)] = d;
            open.emplace(d, j);
          }
        }
      }
    }
  }

  // Path length [m] to the cell of a world position (infinity: unreachable).
  double distanceTo(const Eigen::Vector2d & world) const
  {
    const Eigen::Vector2i c = grid_.cellOf(world);
    return distanceToCell(c.x(), c.y());
  }

  double distanceToCell(int x, int y) const
  {
    if (!grid_.inside(x, y)) {
      return std::numeric_limits<double>::infinity();
    }
    return distance_[static_cast<std::size_t>(grid_.index(x, y))];
  }

  bool blocked(int x, int y) const
  {
    return !grid_.inside(x, y) || blocked_[static_cast<std::size_t>(grid_.index(x, y))];
  }

private:
  const CoverageGrid & grid_;
  std::vector<double> distance_;
  std::vector<bool> blocked_;
};

// ---------------------------------------------------------------- frontiers

struct FrontierConfig
{
  int min_cells {3};  // smaller frontier clusters are noise
  double gain_radius {4.0};  // [m] unknown area counted around a frontier's goal
};

struct Frontier
{
  std::vector<Eigen::Vector2i> cells;  // in discovery order
  Eigen::Vector2d centroid {Eigen::Vector2d::Zero()};
  // The goal: the frontier cell nearest the centroid that the robot can reach (shortest path
  // on ties); unreachable frontiers have reachable = false.
  Eigen::Vector2d goal {Eigen::Vector2d::Zero()};
  bool reachable {false};
  double path_length {std::numeric_limits<double>::infinity()};
  double unknown_area {0.0};  // [m^2] unknown cells within gain_radius of the goal
};

inline std::vector<Frontier> findFrontiers(
  const CoverageGrid & grid, const CoverageGridConfig & grid_config, const PathField & paths,
  const FrontierConfig & config)
{
  const int w = grid.width();
  const int h = grid.height();
  auto state = [&](int x, int y) {return grid.state(x, y, grid_config.occupied_ratio);};
  auto is_frontier = [&](int x, int y) {
      if (state(x, y) != Cell::FREE) {
        return false;
      }
      return state(x + 1, y) == Cell::UNKNOWN || state(x - 1, y) == Cell::UNKNOWN ||
             state(x, y + 1) == Cell::UNKNOWN || state(x, y - 1) == Cell::UNKNOWN;
    };
  std::vector<bool> seen(static_cast<std::size_t>(w) * h, false);
  std::vector<Frontier> frontiers;
  for (int y = grid.y0(); y < grid.y0() + h; ++y) {
    for (int x = grid.x0(); x < grid.x0() + w; ++x) {
      if (seen[static_cast<std::size_t>(grid.index(x, y))] || !is_frontier(x, y)) {
        continue;
      }
      Frontier frontier;
      std::deque<Eigen::Vector2i> queue{Eigen::Vector2i(x, y)};
      seen[static_cast<std::size_t>(grid.index(x, y))] = true;
      while (!queue.empty()) {
        const Eigen::Vector2i c = queue.front();
        queue.pop_front();
        frontier.cells.push_back(c);
        for (int oy = -1; oy <= 1; ++oy) {
          for (int ox = -1; ox <= 1; ++ox) {
            const int nx = c.x() + ox;
            const int ny = c.y() + oy;
            if (!grid.inside(nx, ny) || seen[static_cast<std::size_t>(grid.index(nx, ny))] ||
              !is_frontier(nx, ny))
            {
              continue;
            }
            seen[static_cast<std::size_t>(grid.index(nx, ny))] = true;
            queue.emplace_back(nx, ny);
          }
        }
      }
      if (static_cast<int>(frontier.cells.size()) < config.min_cells) {
        continue;
      }
      for (const Eigen::Vector2i & c : frontier.cells) {
        frontier.centroid += grid.centerOf(c.x(), c.y());
      }
      frontier.centroid /= static_cast<double>(frontier.cells.size());
      // Goal: reachable frontier cell nearest the centroid.
      double best = std::numeric_limits<double>::infinity();
      for (const Eigen::Vector2i & c : frontier.cells) {
        const double path = paths.distanceToCell(c.x(), c.y());
        if (!std::isfinite(path)) {
          continue;
        }
        const double to_centroid = (grid.centerOf(c.x(), c.y()) - frontier.centroid).norm();
        if (to_centroid < best - 1e-9 ||
          (std::abs(to_centroid - best) <= 1e-9 && path < frontier.path_length))
        {
          best = to_centroid;
          frontier.goal = grid.centerOf(c.x(), c.y());
          frontier.path_length = path;
          frontier.reachable = true;
        }
      }
      if (!frontier.reachable) {
        frontier.goal = frontier.centroid;
      }
      // Unknown area around the goal.
      const Eigen::Vector2i g = grid.cellOf(frontier.goal);
      const int reach = static_cast<int>(std::ceil(config.gain_radius / grid.resolution()));
      const double reach2 = std::pow(config.gain_radius / grid.resolution(), 2);
      int unknown = 0;
      for (int oy = -reach; oy <= reach; ++oy) {
        for (int ox = -reach; ox <= reach; ++ox) {
          if (ox * ox + oy * oy <= reach2 && state(g.x() + ox, g.y() + oy) == Cell::UNKNOWN) {
            ++unknown;
          }
        }
      }
      frontier.unknown_area = unknown * grid.resolution() * grid.resolution();
      frontiers.push_back(std::move(frontier));
    }
  }
  return frontiers;
}

// ---------------------------------------------------------------- goal selection

enum class GoalKind
{
  FRONTIER,
  REVISIT,
};

struct SelectionConfig
{
  double info_weight {1.0};  // per m^2 of unknown area
  double uncertainty_weight {1.0};  // per unit of sigma the revisit could remove
  double distance_weight {0.5};  // per m of path
  // Revisit mode starts when the newest submap's sigma exceeds revisit_trigger_sigma and ends
  // when it falls below revisit_release_sigma (hysteresis).
  double revisit_trigger_sigma {std::numeric_limits<double>::infinity()};
  double revisit_release_sigma {0.0};
  // Revisit targets: submaps at least min_index_gap older than the newest whose sigma is at
  // most anchor_sigma_ratio times the newest's.
  int min_index_gap {20};
  double anchor_sigma_ratio {0.5};
  // [m] a revisit goal this close to the robot is pointless (the loop search has seen it).
  double min_revisit_distance {3.0};
};

struct GoalCandidate
{
  GoalKind kind {GoalKind::FRONTIER};
  Eigen::Vector2d goal {Eigen::Vector2d::Zero()};
  int source {-1};  // frontier index, or the submap index of a revisit target
  double path_length {0.0};
  double unknown_area {0.0};
  double uncertainty_drop {0.0};  // newest sigma minus the target's sigma (revisits)
  double utility {0.0};
};

struct Decision
{
  enum class Status
  {
    GOAL,  // `chosen` indexes candidates
    EXPLORATION_DONE,  // no reachable frontier and no revisit needed
    NO_REACHABLE_GOAL,  // revisit wanted but nothing reachable, and no frontier
  };
  Status status {Status::EXPLORATION_DONE};
  bool revisit_mode {false};
  std::vector<GoalCandidate> candidates;  // frontiers first, then revisits; scored
  int chosen {-1};
  std::string reason;
};

struct ExplorerState
{
  bool revisit_mode {false};
};

// Submap positions and sigmas of the graph so far (index = submap index, last = newest).
struct GraphUncertainty
{
  std::vector<Eigen::Vector2d> positions;
  std::vector<double> sigma;  // pose_graph_uncertainty sigma_translation_max (0: fixed)
};

inline Decision selectGoal(
  const PathField & paths, const std::vector<Frontier> & frontiers,
  const GraphUncertainty & graph, const SelectionConfig & config, ExplorerState & state)
{
  Decision decision;
  const double newest_sigma = graph.sigma.empty() ? 0.0 : graph.sigma.back();
  if (state.revisit_mode) {
    state.revisit_mode = newest_sigma > config.revisit_release_sigma;
  } else {
    state.revisit_mode = newest_sigma > config.revisit_trigger_sigma;
  }
  decision.revisit_mode = state.revisit_mode;

  for (std::size_t i = 0; i < frontiers.size(); ++i) {
    const Frontier & f = frontiers[i];
    if (!f.reachable) {
      continue;
    }
    GoalCandidate c;
    c.kind = GoalKind::FRONTIER;
    c.goal = f.goal;
    c.source = static_cast<int>(i);
    c.path_length = f.path_length;
    c.unknown_area = f.unknown_area;
    c.utility = config.info_weight * c.unknown_area - config.distance_weight * c.path_length;
    decision.candidates.push_back(c);
  }
  const std::size_t frontier_count = decision.candidates.size();

  if (state.revisit_mode && graph.sigma.size() == graph.positions.size()) {
    const int newest = static_cast<int>(graph.sigma.size()) - 1;
    const Eigen::Vector2d here = graph.positions.empty() ? Eigen::Vector2d::Zero() :
      graph.positions.back();
    for (int k = 0; k + config.min_index_gap <= newest; ++k) {
      const double s = graph.sigma[static_cast<std::size_t>(k)];
      if (s > config.anchor_sigma_ratio * newest_sigma) {
        continue;
      }
      const Eigen::Vector2d & target = graph.positions[static_cast<std::size_t>(k)];
      if ((target - here).norm() < config.min_revisit_distance) {
        continue;
      }
      const double path = paths.distanceTo(target);
      if (!std::isfinite(path)) {
        continue;
      }
      GoalCandidate c;
      c.kind = GoalKind::REVISIT;
      c.goal = target;
      c.source = k;
      c.path_length = path;
      c.uncertainty_drop = newest_sigma - s;
      c.utility = config.uncertainty_weight * c.uncertainty_drop -
        config.distance_weight * c.path_length;
      decision.candidates.push_back(c);
    }
  }

  // In revisit mode the best revisit wins if there is one; otherwise the best frontier.
  auto best_in = [&](std::size_t from, std::size_t to) {
      int best = -1;
      for (std::size_t i = from; i < to; ++i) {
        if (best < 0 ||
          decision.candidates[i].utility >
          decision.candidates[static_cast<std::size_t>(best)].utility)
        {
          best = static_cast<int>(i);
        }
      }
      return best;
    };
  const int best_revisit = best_in(frontier_count, decision.candidates.size());
  const int best_frontier = best_in(0, frontier_count);
  if (state.revisit_mode && best_revisit >= 0) {
    decision.status = Decision::Status::GOAL;
    decision.chosen = best_revisit;
    decision.reason = "revisit: newest sigma above the trigger";
  } else if (best_frontier >= 0) {
    decision.status = Decision::Status::GOAL;
    decision.chosen = best_frontier;
    decision.reason = state.revisit_mode ? "explore: no reachable revisit target" : "explore";
  } else if (state.revisit_mode) {
    decision.status = Decision::Status::NO_REACHABLE_GOAL;
    decision.reason = "revisit wanted but no target or frontier is reachable";
  } else {
    decision.status = Decision::Status::EXPLORATION_DONE;
    decision.reason = "no reachable frontier";
  }
  return decision;
}

}  // namespace active_exploration
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__ACTIVE_EXPLORATION_HPP_
