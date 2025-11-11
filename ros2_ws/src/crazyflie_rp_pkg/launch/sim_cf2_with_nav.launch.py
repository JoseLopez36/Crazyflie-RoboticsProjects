#!/usr/bin/env python3
"""
Launch file for sim_cf2 with a single simulated Crazyflie drone, SLAM, and Navigation2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([
        FindPackageShare('crazyflie_rp_pkg'),
        'config',
        'slam_toolbox_params.yaml'
    ]))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=PathJoinSubstitution([
        FindPackageShare('crazyflie_rp_pkg'),
        'config',
        'nav2_params.yaml'
    ]))
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to the SLAM parameters file to use')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_params_file,
        description='Full path to the Nav2 parameters file to use')
    
    # Include sim_cf2 with SLAM launch
    sim_cf2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crazyflie_rp_pkg'),
                'launch',
                'sim_cf2_with_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )
    
    # Navigation2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        sim_cf2_slam_launch,
        nav2_bringup,
    ])

