#!/usr/bin/env python3
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration
)
from launch_ros.actions import Node
from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def launch_setup(context,
                 robot_name,
                 use_navigation):
    robot_name_value = context.perform_substitution(robot_name)
    use_navigation_value = context.perform_substitution(use_navigation)

    hsrb_gazebo_bringup_launch = os.path.join(
        get_package_share_directory('hsrb_gazebo_bringup'),
        'launch',
        f'{robot_name_value}_gazebo_bringup.launch.py')
    gazebo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_bringup_launch),
        launch_arguments={'world_file_name': LaunchConfiguration('world_name')}.items())

    # rviz_node
    if use_navigation_value == "true":
        rviz_config = os.path.join(
            get_package_share_directory('hsrb_gazebo_launch'),
            'config/display_full_config.rviz')
    else:
        rviz_config = os.path.join(
            get_package_share_directory('hsrb_gazebo_launch'),
            'config/display_config.rviz')

    robot_description = load_robot_description()

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config],
                     parameters=[robot_description, {'use_sim_time': True}],
                     condition=IfCondition(LaunchConfiguration('rviz')))

    # Launch the navigation node.
    hsrb_common_launch_dir = get_package_share_directory('hsrb_common_launch')

    hsrb_navigation_launch = os.path.join(hsrb_common_launch_dir, 'launch', 'navigation.py')
    hsrb_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_navigation_launch),
        launch_arguments={'use_sim_time': 'true',
                          'map': LaunchConfiguration('map'),
                          'odom_topic': 'odom'}.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation')))

    # manipulation
    hsrb_manipulation_launch = os.path.join(hsrb_common_launch_dir, 'launch', f'{robot_name_value}_manipulation.py')
    hsrb_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_manipulation_launch),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('use_manipulation')))

    # teleop
    hsrb_teleop_launch = os.path.join(hsrb_common_launch_dir, 'launch', 'teleop.py')
    hsrb_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_teleop_launch),
        launch_arguments={'description_package': LaunchConfiguration('description_package'),
                          'description_file': LaunchConfiguration('description_file'),
                          'teleop_runtime_config_package': 'hsrb_common_launch',
                          'use_joy_node': LaunchConfiguration('use_joy_node'),
                          'use_sim_time': 'true',
                          'hand_close_force': '0.08'}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop')))

    nodes = [rviz_node,
             hsrb_navigation,
             hsrb_manipulation,
             hsrb_teleop,
             gazebo_bringup_launch]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup,
                        args=[LaunchConfiguration('robot_name'),
                              LaunchConfiguration('use_navigation')])])
