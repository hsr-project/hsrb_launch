#!/usr/bin/env python3
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import (
    Node,
    SetParameter
)


def declare_arguments():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument('map',
                              default_value='',
                              description='Full path to map yaml file to load'))
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'))
    declared_arguments.append(
        DeclareLaunchArgument('use_rear_scan',
                              default_value='false',
                              description='Use rear scan if true'))
    declared_arguments.append(
        DeclareLaunchArgument('robot_tf_name',
                              default_value='base_footprint',
                              description='Base tf name'))
    declared_arguments.append(
        DeclareLaunchArgument('odom_tf_name',
                              default_value='odom',
                              description='Odometry tf name'))
    declared_arguments.append(
        DeclareLaunchArgument('path_planner_name',
                              default_value='base_path_planner',
                              description='Path planner node name'))
    declared_arguments.append(
        DeclareLaunchArgument('path_follower_name',
                              default_value='base_path_follower',
                              description='Path follower node name'))
    declared_arguments.append(
        DeclareLaunchArgument('odom_topic',
                              default_value='omni_base_controller/wheel_odom',
                              description='Odometry topic name'))

    return declared_arguments


def generate_launch_description():
    planner_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'base_path_planner.yaml')
    follower_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'base_path_follower.yaml')
    pose_integrator_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'pose_integrator.yaml')
    map_merger_params_file = os.environ.get(
        'MAP_MERGER_CONFIG',
        os.path.join(get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'map_merger.yaml'))
    map_merger_two_scan_params_file = os.environ.get(
        'MAP_MERGER_CONFIG',
        os.path.join(get_package_share_directory('hsrb_common_launch'),
                     'config/navigation', 'map_merger_merge_two_scans.yaml'))
    laser_2d_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'laser_2d_localizer.yaml')
    point_cloud_merger_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'point_cloud_merger.yaml')
    viewpoint_controller_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'viewpoint_controller.yaml')
    safety_velocity_limiter_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'safety_velocity_limiter.yaml')
    velocity_switcher_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'velocity_switcher.yaml')
    marker_based_localizer_params_file = os.path.join(
        get_package_share_directory('hsrb_common_launch'), 'config/navigation', 'marker_based_localizer.yaml')

    # base_local_planner
    base_path_planner_launch = Node(
        package='tmc_base_path_planner',
        executable='base_path_planner',
        name=LaunchConfiguration('path_planner_name'),
        output='screen',
        parameters=[planner_params_file]
    )
    # base_path_follower
    base_path_follower_launch = Node(
        package='tmc_base_path_follower',
        executable='base_path_follower',
        name=LaunchConfiguration('path_follower_name'),
        output='screen',
        parameters=[follower_params_file],
    )
    # pose_integrator
    pose_integrator_launch = Node(
        package='tmc_pose_integrator',
        executable='pose_integrator',
        name='pose_integrator',
        output='screen',
        parameters=[pose_integrator_params_file,
                    {'odom_tf_name': LaunchConfiguration('odom_tf_name')},
                    {'base_tf_name': LaunchConfiguration('robot_tf_name')},]
    )
    # grid_map_server
    grid_map_launch = Node(
        package='tmc_grid_map_server',
        executable='grid_map_server',
        name='grid_map_server',
        output='screen',
        parameters=[{'map_yaml_path': LaunchConfiguration('map')}],
    )
    # map_merger
    # When use_rear_scan=false
    map_merger_launch_for_robot_no_rear = Node(
        package='tmc_map_merger',
        executable='map_merger',
        name='map_merger',
        output='screen',
        remappings=[('merged_map', 'dynamic_obstacle_map')],
        parameters=[map_merger_params_file,
                    {'origin_frame': LaunchConfiguration('robot_tf_name')}],
        condition=UnlessCondition(LaunchConfiguration('use_rear_scan'))
    )
    # When use_rear_scan=true
    map_merger_launch_for_sim_use_rear = Node(
        package='tmc_map_merger',
        executable='map_merger',
        name='map_merger',
        output='screen',
        remappings=[('merged_map', 'dynamic_obstacle_map')],
        parameters=[map_merger_two_scan_params_file,
                    {'origin_frame': LaunchConfiguration('robot_tf_name')}],
        condition=IfCondition(LaunchConfiguration('use_rear_scan'))
    )
    # laser_2d_localizer
    laser_2d_localizer_launch = Node(
        package='tmc_laser_2d_localizer',
        executable='laser_2d_localizer',
        name='laser_2d_localizer',
        output='screen',
        remappings=[('input_cloud', 'urg_cloud')],
        parameters=[laser_2d_params_file,
                    {'base_tf_name': LaunchConfiguration('robot_tf_name')},
                    {'odometry_tf_name': LaunchConfiguration('odom_tf_name')}]
    )
    # move_base
    move_base_launch = Node(
        package='tmc_move_base',
        executable='move_base',
        name='move_base',
        output='screen',
        remappings=[('move_base_simple/goal', 'goal')],
        parameters=[{'planning_timeout': 20.0},]
    )
    # pointcloud_to_laserscan
    laserscan_to_pointcloud = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        remappings=[('scan_in', 'scan'),
                    ('cloud', 'urg_cloud')]
    )
    laserscan_to_pointcloud_rear = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud_rear',
        remappings=[('scan_in', 'rear_scan'),
                    ('cloud', 'rear_urg_cloud')],
        condition=IfCondition(LaunchConfiguration('use_rear_scan'))
    )
    # point_cloud_merger
    point_cloud_merger_launch = Node(
        package='tmc_point_cloud_merger',
        executable='point_cloud_merger',
        name='point_cloud_merger',
        output='screen',
        parameters=[point_cloud_merger_params_file],
        condition=IfCondition(LaunchConfiguration('use_rear_scan'))
    )
    # viewpoint_controller
    viewpoint_controller_launch = Node(
        package='tmc_viewpoint_controller',
        executable='viewpoint_controller',
        name='viewpoint_controller',
        output='screen',
        parameters=[viewpoint_controller_params_file],
    )
    # safety_velocity_limiter
    # When there is no rear
    safety_velocity_limiter_launch = Node(
        package='tmc_safety_velocity_limiter',
        executable='safety_velocity_limiter',
        name='safety_velocity_limiter',
        output='screen',
        remappings=[('obstacle_cloud', "urg_cloud"),
                    ('input_velocity', "base_velocity"),
                    ('output_velocity', "command_velocity_autonomy")],
        parameters=[safety_velocity_limiter_params_file,
                    {'enable_function': False}],
        condition=UnlessCondition(LaunchConfiguration('use_rear_scan'))
    )
    # When there is a rear
    safety_velocity_limiter_rear_launch = Node(
        package='tmc_safety_velocity_limiter',
        executable='safety_velocity_limiter',
        name='safety_velocity_limiter',
        output='screen',
        remappings=[('obstacle_cloud', "urg_merged_cloud"),
                    ('input_velocity', "base_velocity"),
                    ('output_velocity', "command_velocity_autonomy")],
        parameters=[safety_velocity_limiter_params_file,
                    {'enable_function': False}],
        condition=IfCondition(LaunchConfiguration('use_rear_scan')))
    # velocity_switcher
    velocity_switcher_launch = Node(
        package='tmc_velocity_switcher',
        executable='velocity_switcher',
        name='velocity_switcher',
        output='screen',
        remappings=[('velocity_1', "command_velocity_teleop"),
                    ('velocity_2', "command_velocity_autonomy"),
                    ('output_velocity', "omni_base_controller/cmd_vel")],
        parameters=[velocity_switcher_params_file],
    )
    # marker_based_localizer
    marker_based_localizer_launch = Node(
        package='tmc_marker_based_localizer',
        executable='marker_based_localizer',
        name='marker_based_localizer', output='screen',
        remappings=[('localized_pose', 'laser_2d_correct_pose'),
                    ('marker/object_info', 'recognized_object'),
                    ('odom', LaunchConfiguration('odom_topic'))],
        parameters=[marker_based_localizer_params_file],
    )

    return LaunchDescription(declare_arguments()
                             + [SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
                                base_path_planner_launch,
                                base_path_follower_launch,
                                pose_integrator_launch,
                                grid_map_launch,
                                map_merger_launch_for_robot_no_rear,
                                map_merger_launch_for_sim_use_rear,
                                laser_2d_localizer_launch,
                                move_base_launch,
                                laserscan_to_pointcloud,
                                laserscan_to_pointcloud_rear,
                                point_cloud_merger_launch,
                                viewpoint_controller_launch,
                                safety_velocity_limiter_launch,
                                safety_velocity_limiter_rear_launch,
                                velocity_switcher_launch,
                                marker_based_localizer_launch])
