cmake_minimum_required(VERSION 3.8)
project(lidarslam_active)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graph_based_slam REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# The exploration logic is header-only and ROS-free (active_exploration.hpp of graph_based_slam).
add_library(lidarslam_active_planner INTERFACE)
target_include_directories(lidarslam_active_planner INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${graph_based_slam_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(exploration_node src/exploration_node.cpp)
target_link_libraries(exploration_node lidarslam_active_planner)
ament_target_dependencies(exploration_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_eigen tf2_ros)

install(TARGETS exploration_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  if(DEFINED ENV{ROS_DISTRO} AND "$ENV{ROS_DISTRO}" STREQUAL "humble")
    # humble's uncrustify formats differently from jazzy's (see graph_based_slam).
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  endif()
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_exploration_planner test/test_exploration_planner.cpp)
  if(TARGET test_exploration_planner)
    target_link_libraries(test_exploration_planner lidarslam_active_planner)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
