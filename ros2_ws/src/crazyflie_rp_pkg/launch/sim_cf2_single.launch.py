#!/usr/bin/env python3
"""
Launch file for sim_cf2 with a single simulated Crazyflie drone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='crazyflie_rp_pkg').find('crazyflie_rp_pkg')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description='Full path to world file to load')
    
    # Include sim_cf2 main launch file
    sim_cf2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sim_cf2'),
                'launch',
                'main.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file,
        }.items()
    )
    
    # Static transform from map to odom (for now, identity)
    # This will be updated by SLAM later
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        sim_cf2_launch,
        static_tf_map_to_odom,
    ])

