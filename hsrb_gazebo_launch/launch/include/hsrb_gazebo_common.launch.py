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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration
)
from launch_ros.actions import Node
from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def launch_setup(context,
                 robot_name,
                 use_navigation,
                 ground_truth_xyz,
                 ground_truth_rpy,
                 gazebo_visualization):
    robot_name_value = context.perform_substitution(robot_name)
    use_navigation_value = context.perform_substitution(use_navigation)

    ground_truth_xyz_value = context.perform_substitution(ground_truth_xyz)
    ground_truth_rpy_value = context.perform_substitution(ground_truth_rpy)
    gazebo_visualization_value = context.perform_substitution(gazebo_visualization)

    robot_description = load_robot_description(
        xacro_arg='gazebo_sim:=True'
        + ' ground_truth_xyz_offset:=' + ground_truth_xyz_value
        + ' ground_truth_rpy_offset:=' + ground_truth_rpy_value
        + ' gazebo_visualization_enabled:=' + gazebo_visualization_value)

    # gazebo_bringup
    pkg_hsrb_gazebo_bringup_dir = get_package_share_directory('hsrb_gazebo_bringup')
    gazebo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_hsrb_gazebo_bringup_dir,
                'launch',
                'gazebo_bringup.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'gui': LaunchConfiguration('gui')}.items())

    # spawn_hsr_node
    spawn_hsr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hsrb_gazebo_bringup_dir, 'launch', 'spawn_hsr.py')
        ),
        launch_arguments={**robot_description,
                          **{'robot_name': LaunchConfiguration('robot_name'),
                             'robot_pos': LaunchConfiguration('robot_pos'),
                             'use_odom_ground_truth': LaunchConfiguration('use_odom_ground_truth'),
                             'use_sim_time': 'true'}}.items())
    # rviz_node
    if use_navigation_value == "true":
        rviz_config = os.path.join(
            get_package_share_directory('hsrb_rosnav_config'),
            'rviz/hsr_navigation2.rviz')
    else:
        rviz_config = os.path.join(
            get_package_share_directory('hsrb_rviz_simulator'),
            'config/display_config.rviz')

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
                          'map': LaunchConfiguration('map')}.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation')))

    tmc_grid_map_server = Node(package='tmc_grid_map_server',
                               executable='grid_map_server',
                               name='grid_map_server',
                               output='screen',
                               parameters=[{"map_yaml_path": LaunchConfiguration('map')},
                                           {'use_sim_time': True}],
                               condition=UnlessCondition(LaunchConfiguration('use_navigation')))

    tmc_grid_map_server_tf = Node(package='tf2_ros',
                                  executable='static_transform_publisher',
                                  name='static_transform_publisher',
                                  output='log',
                                  arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'map'],
                                  condition=UnlessCondition(LaunchConfiguration('use_navigation')))

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
                          'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop')))

    nodes = [rviz_node,
             hsrb_navigation,
             tmc_grid_map_server,
             tmc_grid_map_server_tf,
             hsrb_manipulation,
             hsrb_teleop,
             gazebo_bringup_launch,
             spawn_hsr_launch]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup,
                        args=[LaunchConfiguration('robot_name'),
                              LaunchConfiguration('use_navigation'),
                              LaunchConfiguration('ground_truth_xyz'),
                              LaunchConfiguration('ground_truth_rpy'),
                              LaunchConfiguration('gazebo_visualization')])])
