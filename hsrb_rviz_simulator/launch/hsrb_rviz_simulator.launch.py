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
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tmc_launch_ros_utils.ros2_control import (
    create_spawner_node,
    set_on_process_exit_event_handler,
)
from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrb_description',
                                                    description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro',
                                                    description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(DeclareLaunchArgument('runtime_config_package', default_value='hsrb_rviz_simulator',
                                                    description='Package with the controller\'s configuration.'))
    declared_arguments.append(DeclareLaunchArgument('common_controllers_file', default_value='controllers.yaml',
                                                    description='YAML file with the common controllers configuration.'))
    declared_arguments.append(DeclareLaunchArgument('robot_specific_controllers_file',
                                                    default_value='controllers_hsrb.yaml',
                                                    description='YAML file with the robot specific controllers '
                                                                'configuration.'))

    declared_arguments.append(DeclareLaunchArgument('common_launch_package',
                                                    default_value='hsrb_common_launch',
                                                    description='Package with the common launch files.'))

    declared_arguments.append(DeclareLaunchArgument('use_teleop',
                                                    default_value='false',
                                                    description='Use teleop if true'))
    declared_arguments.append(DeclareLaunchArgument('use_joy_node',
                                                    default_value='true',
                                                    description='Use joy node if true'))

    declared_arguments.append(DeclareLaunchArgument('manipulation_launch',
                                                    default_value='hsrb_manipulation.py',
                                                    description='Launch file for manipulation'))

    declared_arguments.append(DeclareLaunchArgument('use_navigation',
                                                    default_value='false',
                                                    description='Use navigation if true'))

    return declared_arguments


def generate_launch_description():
    robot_description = load_robot_description(xacro_arg='rviz_sim:=True')

    runtime_config_package = LaunchConfiguration('runtime_config_package')
    common_controllers_file = LaunchConfiguration('common_controllers_file')
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', common_controllers_file])

    robot_specific_controllers_file = LaunchConfiguration('robot_specific_controllers_file')
    robot_specific_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', robot_specific_controllers_file])

    control_node = Node(package='controller_manager',
                        executable='ros2_control_node',
                        parameters=[robot_description, robot_controllers, robot_specific_controllers],
                        remappings=[('odom', '~/wheel_odom')])

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[
                                     {'source_list': ['/joint_states']}],
                                 namespace='whole_body',
                                 remappings=[('robot_description', '/robot_description')])
    robot_state_pub_node = Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                parameters=[robot_description],
                                namespace='whole_body',
                                output={'both': 'log'},
                                remappings=[('robot_description', '/robot_description')])
    wheel_odom_connector_tf = Node(package='tf2_ros',
                                   executable='static_transform_publisher',
                                   name='static_transform_publisher',
                                   output='log',
                                   arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                                              'base_footprint_wheel', 'base_footprint'])

    rviz_config = os.path.join(get_package_share_directory(
        'hsrb_rviz_simulator'), 'config/display_config.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config],
                     parameters=[robot_description])

    hsrb_teleop_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', 'teleop.py'])
    hsrb_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_teleop_launch),
        launch_arguments={'description_package': LaunchConfiguration('description_package'),
                          'description_file': LaunchConfiguration('description_file'),
                          'teleop_runtime_config_package': 'hsrb_common_launch',
                          'use_joy_node': LaunchConfiguration('use_joy_node'),
                          'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop')))

    hsrb_manipulation_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch',
         LaunchConfiguration('manipulation_launch')])
    hsrb_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_manipulation_launch),
        launch_arguments={'use_sim_time': 'false'}.items())

    hsrb_navigation_launch = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration('common_launch_package')), 'launch', 'navigation.py'])
    hsrb_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_navigation_launch),
        launch_arguments={'use_sim_time': 'false',
                          'map': PathJoinSubstitution([FindPackageShare('tmc_potential_maps'),
                                                       'maps', 'white_space', 'map.yaml'])}.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation')))

    # If the scan topic is not published, rosnav will not work.
    dummy_scan_publisher = ExecuteProcess(cmd=['ros2', 'topic', 'pub', '/scan', 'sensor_msgs/msg/LaserScan',
                                               '{header: {stamp: now, frame_id: base_range_sensor_link}}'],
                                          condition=IfCondition(LaunchConfiguration('use_navigation')))

    motion_command_limitter_controller_spawner = create_spawner_node('motion_command_limitter_controller')
    omni_base_controller_spawner = create_spawner_node('omni_base_controller')
    nodes = [control_node,
             joint_state_publisher,
             robot_state_pub_node,
             wheel_odom_connector_tf,
             create_spawner_node('joint_state_broadcaster'),
             create_spawner_node('head_trajectory_controller'),
             create_spawner_node('arm_trajectory_controller'),
             create_spawner_node('gripper_controller'),
             motion_command_limitter_controller_spawner,
             set_on_process_exit_event_handler(motion_command_limitter_controller_spawner.actions[0],
                                               omni_base_controller_spawner.actions),
             rviz_node,
             hsrb_teleop,
             hsrb_manipulation,
             hsrb_navigation,
             dummy_scan_publisher]

    return LaunchDescription(declare_arguments() + nodes)
