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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import (
    Node,
    SetParameter,
)


def declare_arguments():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument('rrt_planner_launch',
                              default_value='hsrb_planner.launch.py',
                              description='Launch file for rrt planner'))
    declared_arguments.append(
        DeclareLaunchArgument('timeopt_filter_launch',
                              default_value='hsrb_timeopt_filter.launch.py',
                              description='Launch file for timeopt filter'))
    declared_arguments.append(
        DeclareLaunchArgument('safe_pose_changer_launch',
                              default_value='hsrb_safe_pose_changer.launch.py',
                              description='Launch file for safe pose changer'))

    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'))

    return declared_arguments


def generate_launch_description():
    hsrb_manipulation_launch_dir = get_package_share_directory('hsrb_manipulation_launch')

    planner_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [hsrb_manipulation_launch_dir, 'launch', LaunchConfiguration('rrt_planner_launch')])))
    timeopt_ros_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [hsrb_manipulation_launch_dir, 'launch', LaunchConfiguration('timeopt_filter_launch')])))
    safe_pose_changer_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [hsrb_manipulation_launch_dir, 'launch', LaunchConfiguration('safe_pose_changer_launch')])))

    collision_environment_node = Node(package='tmc_collision_environment',
                                      executable='collision_environment_node',
                                      name='collision_environment_server',
                                      parameters=[{'origin_frame_id': 'odom'}])

    attached_object_publisher_node = Node(package='tmc_collision_environment',
                                          executable='attached_object_publisher',
                                          name='attached_object_publisher',
                                          parameters=[{'grasping_frame_id': 'hand_palm_link'}])

    return LaunchDescription(declare_arguments()
                             + [SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
                                planner_launch,
                                timeopt_ros_launch,
                                safe_pose_changer_launch,
                                collision_environment_node,
                                attached_object_publisher_node])
