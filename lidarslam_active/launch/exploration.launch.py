# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Start the lidarslam_active frontier exploration node (opt-in; goals on goal_pose)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the exploration node with its parameter file and topic remaps."""
    default_params = os.path.join(
        get_package_share_directory('lidarslam_active'), 'config', 'exploration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        # RKO-LIO's deskewed scan (rko_lio_slam.launch.py publishes it as /rko_lio/frame).
        DeclareLaunchArgument('cloud_topic', default_value='/rko_lio/frame'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='lidarslam_active',
            executable='exploration_node',
            name='lidarslam_active',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('cloud', LaunchConfiguration('cloud_topic')),
                        ('goal_pose', LaunchConfiguration('goal_topic'))],
        ),
    ])
