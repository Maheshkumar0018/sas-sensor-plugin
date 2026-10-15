# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Bring up Gazebo with a test world, the test carrier, and the SAS sensor. No processing nodes.

Example:
  ros2 launch sas_bringup sas_sensor.launch.py                           # seabed, gpu_ray, 1.5 m/s
  ros2 launch sas_bringup sas_sensor.launch.py world:=sas_point_target.world speed:=0.0
  ros2 launch sas_bringup sas_sensor.launch.py sensor_type:=ray h_samples:=128 v_samples:=128
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    if not os.path.isabs(world):
        world = os.path.join(get_package_share_directory('sas_bringup'), 'worlds', world)
    xacro_file = os.path.join(
        get_package_share_directory('sas_description'), 'urdf', 'sas_test_carrier.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        k: arg(k) for k in ('sensor_type', 'prf', 'h_samples', 'v_samples', 'debug')}).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': arg('gui'),
            'verbose': 'true',
            # gpu_ray needs Gazebo's lockstep; CPU ray is gated by the plugin (ADR-0010).
            'extra_gazebo_args': '--lockstep',
        }.items())

    return [
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'sas_test_carrier', '-topic', 'robot_description',
                        '-x', arg('x'), '-y', arg('y'), '-z', arg('z')]),
        # Constant along-track speed for the carrier (stock planar_move, ADR-0005).
        ExecuteProcess(cmd=['ros2', 'topic', 'pub', '-r', '10', '/sas/cmd_vel',
                            'geometry_msgs/msg/Twist',
                            '{linear: {x: %s}}' % arg('speed')],
                       output='log'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sas_seabed.world',
                              description='file in sas_bringup/worlds, or an absolute path'),
        DeclareLaunchArgument('sensor_type', default_value='gpu_ray', description='gpu_ray | ray'),
        DeclareLaunchArgument('prf', default_value='10.0', description='Hz'),
        DeclareLaunchArgument('h_samples', default_value='512'),
        DeclareLaunchArgument('v_samples', default_value='512'),
        DeclareLaunchArgument('speed', default_value='1.5', description='m/s along +x'),
        DeclareLaunchArgument('x', default_value='-10.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='10.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='log scatterer/retro summary every ping'),
        OpaqueFunction(function=nodes),
    ])
