from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    # Get paths to the package configuration files
    package_dir = get_package_share_directory('crazyflie_robotics_projects_pkg')
    mission_config_path = os.path.join(package_dir, 'config', 'mission.yaml')
    nodes_config_path = os.path.join(package_dir, 'config', 'nodes.yaml')
    launch_config_path = os.path.join(package_dir, 'config', 'launch.yaml')
    
    # Load package configuration files
    with open(launch_config_path, 'r') as f:
        launch = yaml.safe_load(f)
    
    # Get node enable/log-level configuration
    nodes = {
        'transform': launch.get('transform', [True, 'info'])
    }

    # Initialize action list
    actions = []

    # Conditionally add the transform node
    if nodes['transform'][0]:
        actions.append(
            Node(
                package='crazyflie_robotics_projects_pkg',
                executable='transform_node',
                name='transform_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['transform'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )

    # Add test node
    actions.append(
        Node(
            package='crazyflie_robotics_projects_pkg',
            executable='test_node',
            name='test_node',
            output='screen',
            parameters=[mission_config_path, nodes_config_path]
        )
    )

    return actions