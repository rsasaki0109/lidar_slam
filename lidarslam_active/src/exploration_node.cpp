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

// lidarslam_active exploration node: frontier exploration goals for a navigation stack
// (docs/roadmap/active-slam-2026-11.md, P4). Clouds and TF in, goal_pose out; the decision is
// lidarslam_active::ExplorationPlanner's.


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lidarslam_active/exploration_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace lidarslam_active
{

class ExplorationNode : public rclcpp::Node
{
public:
  explicit ExplorationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidarslam_active", options)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    const std::string policy = declare_parameter<std::string>("policy", "frontier_info");
    if (policy != "frontier_info") {
      throw std::invalid_argument(
              "policy '" + policy + "' is not available: only frontier_info (the 'active' "
              "policy needs the graph's uncertainty published live, not implemented yet)");
    }
    point_stride_ = std::max<int64_t>(1, declare_parameter<int64_t>("point_stride", 1));
    const double decision_period = declare_parameter<double>("decision_period", 1.0);

    PlannerParams p;
    p.grid.resolution = declare_parameter<double>("grid_resolution", p.grid.resolution);
    p.grid.ground_max_z = declare_parameter<double>("ground_max_z", p.grid.ground_max_z);
    p.grid.obstacle_min_z = declare_parameter<double>("obstacle_min_z", p.grid.obstacle_min_z);
    p.grid.obstacle_max_z = declare_parameter<double>("obstacle_max_z", p.grid.obstacle_max_z);
    p.grid.max_range = declare_parameter<double>("max_range", p.grid.max_range);
    p.grid.occupied_ratio = declare_parameter<double>("occupied_ratio", p.grid.occupied_ratio);
    p.grid.clear_radius = declare_parameter<double>("clear_radius", p.grid.clear_radius);
    p.planner.inflation_radius =
      declare_parameter<double>("inflation_radius", p.planner.inflation_radius);
    p.frontier.min_cells =
      static_cast<int>(declare_parameter<int64_t>("frontier_min_cells", p.frontier.min_cells));
    p.frontier.gain_radius = declare_parameter<double>("gain_radius", p.frontier.gain_radius);
    p.frontier.min_goal_distance =
      declare_parameter<double>("min_goal_distance", p.frontier.min_goal_distance);
    p.selection.info_weight = declare_parameter<double>("info_weight", p.selection.info_weight);
    p.selection.distance_weight =
      declare_parameter<double>("distance_weight", p.selection.distance_weight);
    p.selection.visited_radius =
      declare_parameter<double>("visited_radius", p.selection.visited_radius);
    p.keyframe_distance = declare_parameter<double>("keyframe_distance", p.keyframe_distance);
    p.keyframe_angle = declare_parameter<double>("keyframe_angle", p.keyframe_angle);
    p.goal_timeout = declare_parameter<double>("goal_timeout", p.goal_timeout);
    p.stall_timeout = declare_parameter<double>("stall_timeout", p.stall_timeout);
    p.stall_distance = declare_parameter<double>("stall_distance", p.stall_distance);
    p.goal_change_distance =
      declare_parameter<double>("goal_change_distance", p.goal_change_distance);
    p.done_confirmations = static_cast<int>(
      declare_parameter<int64_t>("done_confirmations", p.done_confirmations));
    planner_ = std::make_unique<ExplorationPlanner>(p);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("exploration_path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("exploration_status", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "exploration_grid", rclcpp::QoS(1).transient_local());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(*msg);});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(decision_period), [this]() {decide();});
    RCLCPP_INFO(
      get_logger(), "frontier exploration in '%s' for '%s' (goals on %s)", global_frame_.c_str(),
      robot_frame_.c_str(), goal_pub_->get_topic_name());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    Eigen::Isometry3d sensor_pose;
    try {
      sensor_pose = tf2::transformToEigen(
        tf_buffer_->lookupTransform(
          global_frame_, msg.header.frame_id, msg.header.stamp,
          rclcpp::Duration::from_seconds(0.1)));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cloud skipped: %s", e.what());
      return;
    }
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(msg.width) * msg.height / point_stride_ + 1U);
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
    int64_t i = 0;
    for (; x != x.end(); ++x, ++y, ++z, ++i) {
      if (i % point_stride_ != 0 || !std::isfinite(*x) || !std::isfinite(*y) ||
        !std::isfinite(*z))
      {
        continue;
      }
      points.emplace_back(*x, *y, *z);
    }
    if (planner_->addScan(sensor_pose, std::move(points))) {
      grid_dirty_ = true;
    }
  }

  void decide()
  {
    Eigen::Vector2d robot_xy;
    try {
      const Eigen::Isometry3d robot = tf2::transformToEigen(
        tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero));
      robot_xy = robot.translation().head<2>();
    } catch (const tf2::TransformException & e) {
      publishStatus("waiting_for_data");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no robot pose: %s", e.what());
      return;
    }
    const PlanResult result = planner_->plan(robot_xy, now().seconds());
    if (!result.rejected.empty()) {
      RCLCPP_INFO(get_logger(), "goal rejected (%s)", result.rejected.c_str());
    }
    publishStatus(toString(result.status));
    if (grid_dirty_) {
      publishGrid();
      grid_dirty_ = false;
    }
    if (result.status != PlanStatus::EXPLORING || !result.has_goal) {
      if (result.status != last_status_ &&
        (result.status == PlanStatus::EXPLORATION_DONE ||
        result.status == PlanStatus::NO_REACHABLE_GOAL))
      {
        RCLCPP_INFO(get_logger(), "%s: no further goals", toString(result.status));
      }
      last_status_ = result.status;
      return;
    }
    last_status_ = result.status;
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = now();
    for (const Eigen::Vector2d & p : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    if (!result.new_goal) {
      return;
    }
    // Face along the last stretch of the planned path (towards the unknown).
    double yaw = 0.0;
    if (result.path.size() >= 2U) {
      const Eigen::Vector2d d =
        result.path.back() - result.path[result.path.size() >= 4U ? result.path.size() - 4U : 0U];
      yaw = std::atan2(d.y(), d.x());
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header = path.header;
    goal.pose.position.x = result.goal.x();
    goal.pose.position.y = result.goal.y();
    goal.pose.orientation.z = std::sin(0.5 * yaw);
    goal.pose.orientation.w = std::cos(0.5 * yaw);
    goal_pub_->publish(goal);
    RCLCPP_INFO(
      get_logger(), "goal (%.2f, %.2f)%s, %zu frontiers", result.goal.x(), result.goal.y(),
      result.through_unknown ? " through unknown cells" : "", result.frontiers);
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void publishGrid()
  {
    namespace ae = graphslam::active_exploration;
    const ae::CoverageGrid & grid = planner_->grid();
    if (grid.empty()) {
      return;
    }
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = now();
    msg.info.resolution = static_cast<float>(grid.resolution());
    msg.info.width = static_cast<uint32_t>(grid.width());
    msg.info.height = static_cast<uint32_t>(grid.height());
    msg.info.origin.position.x = grid.x0() * grid.resolution();
    msg.info.origin.position.y = grid.y0() * grid.resolution();
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(static_cast<std::size_t>(grid.width()) * grid.height());
    const double occupied_ratio = planner_->params().grid.occupied_ratio;
    for (int y = grid.y0(); y < grid.y0() + grid.height(); ++y) {
      for (int x = grid.x0(); x < grid.x0() + grid.width(); ++x) {
        const ae::Cell cell = grid.state(x, y, occupied_ratio);
        msg.data[static_cast<std::size_t>(grid.index(x, y))] =
          cell == ae::Cell::UNKNOWN ? -1 : cell == ae::Cell::OCCUPIED ? 100 : 0;
      }
    }
    grid_pub_->publish(msg);
  }

  std::string global_frame_;
  std::string robot_frame_;
  int64_t point_stride_ {1};
  std::unique_ptr<ExplorationPlanner> planner_;
  bool grid_dirty_ {false};
  PlanStatus last_status_ {PlanStatus::WAITING_FOR_DATA};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lidarslam_active

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidarslam_active::ExplorationNode>());
  rclcpp::shutdown();
  return 0;
}
