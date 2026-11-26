// Copyright 2026 lidarslam_ros2 contributors.
// SPDX-License-Identifier: BSD-2-Clause
#ifndef LIDARSLAM_CORE__PYTHON_MAP_CHANGES_HPP_
#define LIDARSLAM_CORE__PYTHON_MAP_CHANGES_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

#include "graph_based_slam/map_change_evidence.hpp"
#include "graph_based_slam/map_store.hpp"

namespace lidarslam_core
{
namespace changes = graphslam::map_change;

using ChangeScan = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

inline std::vector<changes::Point> scanEndpoints(
  const changes::Point & origin, const Eigen::Ref<const ChangeScan> & endpoints)
{
  if (!changes::usable(origin)) {
    throw std::invalid_argument("invalid change-evidence sensor origin");
  }
  std::unordered_set<changes::Voxel, changes::VoxelHash> occupied;
  std::vector<changes::Point> rays;
  for (Eigen::Index i = 0; i < endpoints.rows(); ++i) {
    const changes::Point point{endpoints(i, 0), endpoints(i, 1), endpoints(i, 2)};
    if (changes::usable(point) && changes::squaredDistance(origin, point) <= 1600.0 &&
      occupied.insert(changes::voxel(point)).second)
    {
      rays.push_back(point);
    }
  }
  return rays;
}

struct VoxelCentroid
{
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  std::size_t count{0};
};

inline std::unordered_map<changes::Voxel, VoxelCentroid, changes::VoxelHash> submapCentroids(
  const graphslam::map_store::StoredSubmap & stored)
{
  std::unordered_map<changes::Voxel, VoxelCentroid, changes::VoxelHash> centers;
  const auto cloud = graphslam::map_store::submapCloud(stored);
  for (const auto & point : *cloud) {
    const changes::Point local{point.x, point.y, point.z};
    if (changes::usable(local)) {
      auto & center = centers[changes::voxel(local)];
      center.sum += point.getVector3fMap().cast<double>();
      ++center.count;
    }
  }
  return centers;
}

// Return the heap's free pages to the system (glibc keeps them otherwise; elsewhere nothing to
// do). Building the change evidence frees many small temporary blocks between the ones it keeps.
inline void releaseFreeMemory()
{
#if defined(__GLIBC__)
  malloc_trim(0);
#endif
}

class MapChangeEvidence
{
public:
  using Poses = std::vector<std::vector<Eigen::Matrix4d>>;
  using Scan = ChangeScan;
  // session, submap, local voxel xyz, hits, misses, separated views, decision
  using EvidenceRow = std::array<std::int64_t, 9>;
  MapChangeEvidence(const std::vector<std::string> & directories, const Poses & poses)
  {
    if (directories.size() != poses.size()) {
      throw std::invalid_argument("expected one absolute pose list per reference session");
    }
    for (std::size_t session = 0; session < directories.size(); ++session) {
      graphslam::map_store::SessionMap stored;
      std::string error;
      if (!graphslam::map_store::readSession(directories[session], stored, error)) {
        throw std::runtime_error(error);
      }
      if (poses[session].size() != stored.submaps.size()) {
        throw std::invalid_argument("change evidence poses do not match the stored submaps");
      }
      for (std::size_t submap = 0; submap < stored.submaps.size(); ++submap) {
        addSubmap(session, submap, stored.submaps[submap], poses[session][submap]);
      }
    }
    index();
    releaseFreeMemory();
  }

  void observe(const changes::Point & origin, const Eigen::Ref<const Scan> & endpoints)
  {
    observeRays(origin, scanEndpoints(origin, endpoints));
  }

  std::vector<EvidenceRow> snapshot() const
  {
    std::vector<EvidenceRow> rows;
    for (const auto & state : states_) {
      if (!state.marked) {
        continue;
      }
      const bool remove = state.separated &&
        state.misses >= 8 * std::max(std::int32_t{1}, state.hits);
      const bool keep = state.hits >= 2 && state.hits >= state.misses;
      rows.push_back({state.session, state.submap, state.local[0], state.local[1],
        state.local[2], state.hits, state.misses, state.separated, remove ? 2 : (keep ? 1 : 0)});
    }
    return rows;
  }

private:
  friend class RefinedNewPointEvidence;

  void observeRays(const changes::Point & origin, const std::vector<changes::Point> & rays)
  {
    ++frame_;
    const double hit_radius = refined_ ? .10 : .09;
    // Finish counting hits first so the same frame cannot also count a miss.
    for (const auto & point : rays) {
      const auto cell = changes::voxel(point);
      for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
          for (int z = -1; z <= 1; ++z) {
            visitCell({cell[0] + x, cell[1] + y, cell[2] + z}, [&](State & state) {
              if (state.last_hit != frame_ &&
                changes::squaredDistance(point, state.center) <= hit_radius * hit_radius)
              {
                state.last_hit = frame_;
                ++state.hits;
                state.marked = true;
              }
            });
          }
        }
      }
    }
    for (const auto & point : rays) {
      const Eigen::Vector3d ray(point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]);
      const double length = refined_ ? ray.norm() : 0.;
      changes::traceFree(origin, point, [&](const auto & cell) {
        visitCell(cell, [&](State & state) {
          if (state.last_hit == frame_ || state.last_miss == frame_) {
            return;
          }
          if (refined_) {
            const Eigen::Vector3d delta(state.center[0] - origin[0],
              state.center[1] - origin[1], state.center[2] - origin[2]);
            const double depth = ray.dot(delta) / length;
            if (depth <= 0. || depth > length - changes::ray_margin_m ||
              delta.squaredNorm() - depth * depth > .085 * .085)
            {
              return;
            }
          }
          state.last_miss = frame_;
          if (state.misses++ == 0) {
            state.first_view = origin;
          } else if (changes::squaredDistance(origin, state.first_view) >= .5 * .5) {
            state.separated = true;
          }
          state.marked = true;
        });
      });
    }
  }
  // One stored voxel centroid: 88 bytes. The frame counters fit 32 bits (a frame is one
  // submap of the session being recorded).
  struct State
  {
    std::int32_t session, submap;
    changes::Voxel local;
    changes::Point center;
    changes::Point first_view{};
    std::int32_t hits{0}, misses{0}, last_hit{-1}, last_miss{-1};
    bool separated{false}, marked{false};
  };

  void addSubmap(
    std::size_t session, std::size_t submap,
    const graphslam::map_store::StoredSubmap & stored, const Eigen::Matrix4d & pose,
    const Eigen::Vector3d * range_origin = nullptr)
  {
    for (const auto & entry : submapCentroids(stored)) {
      const Eigen::Vector3d local = entry.second.sum / entry.second.count;
      if (range_origin && (local - *range_origin).squaredNorm() > 1600.) {
        continue;
      }
      const Eigen::Vector3d center = pose.topLeftCorner<3, 3>() * local + pose.topRightCorner<3, 1>();
      const changes::Point world{center.x(), center.y(), center.z()};
      if (changes::usable(world)) {
        states_.push_back({static_cast<std::int32_t>(session), static_cast<std::int32_t>(submap),
          entry.first, world});
      }
    }
  }

  // Group the states by world cell in place, so that a cell is one range of states_ (the rows
  // are sorted when they are written, so their order here does not matter). states_ is a deque:
  // it grows and sorts without a second copy of the states.
  void index()
  {
    if (states_.size() >= static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
      throw std::length_error("too many stored voxels for change evidence");
    }
    const auto key = [](const State & state) {return changes::voxel(state.center);};
    std::sort(states_.begin(), states_.end(), [&](const State & a, const State & b) {
        return key(a) < key(b);
      });
    cells_.clear();
    std::size_t count = 0;
    for (std::size_t i = 0; i < states_.size(); ++i) {
      count += i == 0 || key(states_[i]) != key(states_[i - 1]);
    }
    cells_.reserve(count);
    for (std::size_t begin = 0; begin < states_.size(); ) {
      const auto cell = key(states_[begin]);
      std::size_t end = begin + 1;
      while (end < states_.size() && key(states_[end]) == cell) {
        ++end;
      }
      cells_.emplace(cell, std::make_pair(static_cast<std::uint32_t>(begin),
        static_cast<std::uint32_t>(end)));
      begin = end;
    }
  }

  template<typename Visit>
  void visitCell(const changes::Voxel & key, Visit visit)
  {
    const auto found = cells_.find(key);
    if (found != cells_.end()) {
      for (auto id = found->second.first; id < found->second.second; ++id) {
        visit(states_[id]);
      }
    }
  }

  std::unordered_map<changes::Voxel, std::pair<std::uint32_t, std::uint32_t>,
    changes::VoxelHash> cells_;
  std::deque<State> states_;
  std::int64_t frame_{0};
  bool refined_{false};
};

class RefinedNewPointEvidence
{
public:
  using MaskRow = std::array<std::int64_t, 4>;

  void observe(const changes::Point & origin, const Eigen::Ref<const ChangeScan> & endpoints)
  {
    auto rays = scanEndpoints(origin, endpoints);
    if (!rays.empty()) {
      scans_.push_back({origin, std::move(rays)});
    }
  }

  std::vector<MaskRow> suppressionMask(
    const std::string & directory, const std::map<std::size_t, Eigen::Matrix4d> & capture_poses,
    const Eigen::Vector3d & sensor_in_base) const
  {
    if (scans_.size() < 3) {
      return {};
    }
    graphslam::map_store::SessionMap session;
    std::string error;
    if (!graphslam::map_store::readSession(directory, session, error)) {
      throw std::runtime_error(error);
    }
    MapChangeEvidence current({}, {});
    current.refined_ = true;
    for (const auto & [index, pose] : capture_poses) {
      if (index >= session.submaps.size()) {
        throw std::invalid_argument("new-point evidence refers to a missing submap");
      }
      current.addSubmap(0, index, session.submaps[index], pose, &sensor_in_base);
    }
    current.index();
    // Build the entire candidate index before voting, so early free space counts.
    for (const auto & scan : scans_) {
      current.observeRays(scan.origin, scan.rays);
    }
    std::vector<MaskRow> mask;
    for (const auto & state : current.states_) {
      if (state.hits < 2 || state.hits < state.misses) {
        mask.push_back({state.submap, state.local[0], state.local[1], state.local[2]});
      }
    }
    return mask;
  }

private:
  struct ScanRays
  {
    changes::Point origin;
    std::vector<changes::Point> rays;
  };

  std::vector<ScanRays> scans_;
};

// Per-session support for the new scan endpoints. Free-space votes apply from
// the first hit onward, and a hit takes precedence within the same scan.
class NewPointEvidence
{
public:
  using MaskRow = std::array<std::int64_t, 4>;

  void observe(const changes::Point & origin, const Eigen::Ref<const ChangeScan> & endpoints)
  {
    const auto rays = scanEndpoints(origin, endpoints);
    ++frame_;
    for (const auto & point : rays) {
      auto & cell = cells_[changes::voxel(point)];
      if (cell.hits++ == 0) {
        cell.representative = point;
        cell.first_hit = frame_;
      }
      cell.last_hit = frame_;
      cell.trailing_misses = 0;
    }
    for (const auto & point : rays) {
      changes::traceFree(origin, point, [&](const auto & key) {
        const auto found = cells_.find(key);
        if (found == cells_.end()) {
          return;
        }
        auto & cell = found->second;
        if (cell.last_hit != frame_ && cell.last_miss != frame_) {
          ++cell.misses;
          ++cell.trailing_misses;
          cell.last_miss = frame_;
        }
      });
    }
  }

  std::vector<MaskRow> suppressionMask(
    const std::string & directory, const std::map<std::size_t, Eigen::Matrix4d> & capture_poses,
    const Eigen::Vector3d & sensor_in_base) const
  {
    graphslam::map_store::SessionMap session;
    std::string error;
    if (!graphslam::map_store::readSession(directory, session, error)) {
      throw std::runtime_error(error);
    }
    std::vector<MaskRow> mask;
    // A too-short session cannot support a persistence decision.
    if (frame_ < 3) {
      return mask;
    }
    for (const auto & [index, pose] : capture_poses) {
      if (index >= session.submaps.size()) {
        throw std::invalid_argument("new-point evidence refers to a missing submap");
      }
      for (const auto & [key, centroid] : submapCentroids(session.submaps[index])) {
        const Eigen::Vector3d local = centroid.sum / centroid.count;
        if ((local - sensor_in_base).squaredNorm() > 1600.0) {
          continue;
        }
        const Eigen::Vector3d world = pose.topLeftCorner<3, 3>() * local +
          pose.topRightCorner<3, 1>();
        const changes::Point point{world.x(), world.y(), world.z()};
        if (changes::usable(point) && !supported(point)) {
          mask.push_back({static_cast<std::int64_t>(index), key[0], key[1], key[2]});
        }
      }
    }
    return mask;
  }

  // Per-cell votes for diagnostics: key; hits, misses, first and last hit frame and misses since
  // the last hit; and the first hit point (the representative).
  std::tuple<Eigen::Matrix<std::int32_t, Eigen::Dynamic, 3, Eigen::RowMajor>,
    Eigen::Matrix<std::int64_t, Eigen::Dynamic, 5, Eigen::RowMajor>,
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> cells() const
  {
    Eigen::Matrix<std::int32_t, Eigen::Dynamic, 3, Eigen::RowMajor> keys(cells_.size(), 3);
    Eigen::Matrix<std::int64_t, Eigen::Dynamic, 5, Eigen::RowMajor> votes(cells_.size(), 5);
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> points(cells_.size(), 3);
    Eigen::Index row = 0;
    for (const auto & [key, cell] : cells_) {
      keys.row(row) << key[0], key[1], key[2];
      votes.row(row) << cell.hits, cell.misses, cell.first_hit, cell.last_hit,
        cell.trailing_misses;
      points.row(row) << cell.representative[0], cell.representative[1], cell.representative[2];
      ++row;
    }
    return {keys, votes, points};
  }

  std::int64_t frames() const {return frame_;}

private:
  bool supported(const changes::Point & point) const
  {
    const auto key = changes::voxel(point);
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          const auto found = cells_.find({key[0] + x, key[1] + y, key[2] + z});
          if (found != cells_.end()) {
            const auto & cell = found->second;
            if (cell.hits >= 3 && cell.hits >= cell.misses &&
              changes::squaredDistance(point, cell.representative) <= .15 * .15)
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  struct State
  {
    changes::Point representative{};
    std::int64_t hits{0}, misses{0}, last_hit{-1}, last_miss{-1};
    std::int64_t first_hit{-1}, trailing_misses{0};
  };

  std::unordered_map<changes::Voxel, State, changes::VoxelHash> cells_;
  std::int64_t frame_{0};
};
}  // namespace lidarslam_core
#endif  // LIDARSLAM_CORE__PYTHON_MAP_CHANGES_HPP_
