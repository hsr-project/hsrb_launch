#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('description_package',
                                                    default_value='hsrb_description',
                                                    description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(DeclareLaunchArgument('description_file',
                                                    default_value='hsrb4s.urdf.xacro',
                                                    description='URDF/XACRO description file with the robot.'))

    declared_arguments.append(DeclareLaunchArgument('common_launch_package',
                                                    default_value='hsrb_common_launch',
                                                    description='Package with the common launch files.'))

    declared_arguments.append(DeclareLaunchArgument('use_sim_time',
                                                    default_value='false',
                                                    description='Use simulation clock if true'))

    declared_arguments.append(DeclareLaunchArgument('use_manipulation',
                                                    default_value=os.environ.get('USE_MANIPULATION', 'true'),
                                                    description='Use manipulation if true'))
    declared_arguments.append(DeclareLaunchArgument('manipulation_launch',
                                                    default_value='hsrb_manipulation.py',
                                                    description='Launch file for manipulation'))

    declared_arguments.append(DeclareLaunchArgument('use_navigation',
                                                    default_value=os.environ.get('USE_NAVIGATION', 'true'),
                                                    description='Use navigation if true'))
    default_map = get_package_share_directory('tmc_potential_maps') + '/maps/white_space/map.yaml'
    declared_arguments.append(DeclareLaunchArgument('map',
                                                    default_value=os.environ.get('MAP', default_map),
                                                    description='Map yaml file'))
    declared_arguments.append(DeclareLaunchArgument('odom_topic',
                              default_value='omni_base_controller/wheel_odom',
                              description='Odometry topic name'))

    declared_arguments.append(DeclareLaunchArgument('use_teleop',
                                                    default_value=os.environ.get('USE_TELEOP', 'true'),
                                                    description='Use teleop if true'))
    declared_arguments.append(DeclareLaunchArgument('use_joy_node',
                                                    default_value='true',
                                                    description='Use joy node if true'))

    return declared_arguments


def generate_launch_description():
    hsrb_manipulation_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch',
         LaunchConfiguration('manipulation_launch')])
    hsrb_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_manipulation_launch),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('use_manipulation')))

    hsrb_navigation_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', 'navigation.py'])
    hsrb_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_navigation_launch),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'map': LaunchConfiguration('map'),
                          'odom_topic': LaunchConfiguration('odom_topic')}.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation')))

    hsrb_teleop_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', 'teleop.py'])
    hsrb_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_teleop_launch),
        launch_arguments={'description_package': LaunchConfiguration('description_package'),
                          'description_file': LaunchConfiguration('description_file'),
                          'teleop_runtime_config_package': LaunchConfiguration('common_launch_package'),
                          'odom_topic': LaunchConfiguration('odom_topic'),
                          'use_joy_node': LaunchConfiguration('use_joy_node'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop')))

    return LaunchDescription(declare_arguments() + [hsrb_manipulation, hsrb_navigation, hsrb_teleop])
