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
    declared_arguments.append(DeclareLaunchArgument('common_launch_package',
                                                    default_value='hsrb_common_launch',
                                                    description='Package with the common launch files.'))

    declared_arguments.append(
        DeclareLaunchArgument('use_manipulation',
                              default_value=os.environ.get('USE_MANIPULATION', 'true'),
                              description='Use manipulation if true'))
    declared_arguments.append(
        DeclareLaunchArgument('use_navigation',
                              default_value=os.environ.get('USE_NAVIGATION', 'true'),
                              description='Use navigation if true'))
    declared_arguments.append(
        DeclareLaunchArgument('use_teleop',
                              default_value=os.environ.get('USE_TELEOP', 'true'),
                              description='Use teleop if true'))

    default_map = get_package_share_directory('tmc_potential_maps') + '/maps/white_space/map.yaml'
    declared_arguments.append(
        DeclareLaunchArgument('map',
                              default_value=os.environ.get('MAP_PATH', default_map),
                              description='Map yaml file'))

    return declared_arguments


def generate_launch_description():
    robot_version = os.environ.get("ROBOT_VERSION")
    robot_name = robot_version.replace('"', '').split('-')[0].lower()

    if robot_name == 'hsrb':
        manipulation_launch = 'hsrb_manipulation.py'
        description_package = 'hsrb_description'
        description_file = 'hsrb4s.urdf.xacro'
    elif robot_name == 'hsrc' or robot_name == 'hsrd':
        manipulation_launch = 'hsrc_manipulation.py'
        description_package = 'hsrc_description'
        description_file = 'hsrc1s.urdf.xacro'
    else:
        raise Exception("Invalid ROBOT_VERSION:{0}".format(robot_version))

    hsrb_manipulation_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', manipulation_launch])
    hsrb_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_manipulation_launch),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_manipulation')))

    hsrb_navigation_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', 'navigation.py'])
    hsrb_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_navigation_launch),
        launch_arguments={'use_sim_time': 'false',
                          'map': LaunchConfiguration('map'),
                          'odom_topic': 'switched_odom'}.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation')))

    hsrb_teleop_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', 'teleop.py'])
    hsrb_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_teleop_launch),
        launch_arguments={'description_package': description_package,
                          'description_file': description_file,
                          'teleop_runtime_config_package': 'hsrb_common_launch',
                          'odom_topic': 'switched_odom',
                          'use_joy_node': 'true',
                          'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop')))

    return LaunchDescription(declare_arguments() + [hsrb_manipulation, hsrb_navigation, hsrb_teleop])
