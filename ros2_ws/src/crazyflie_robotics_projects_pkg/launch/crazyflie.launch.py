from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Declare launch argument for the Crazyflie configuration file
    cf_config_arg = DeclareLaunchArgument(
        'cf_config',
        default_value='simulation_config.yaml',
        description='Crazyflie configuration file name (must be in config/crazyflie/)'
    )
    
    return LaunchDescription([
        cf_config_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    # Get the launch argument value
    cf_config_filename = context.launch_configurations.get('cf_config', 'simulation_config.yaml')
    
    # Get paths to the package configuration files
    package_dir = get_package_share_directory('crazyflie_robotics_projects_pkg')
    cf_config_path = os.path.join(package_dir, 'config', 'crazyflie', cf_config_filename)

    # Get paths to the Crazyflie configuration files
    cf_package_dir = get_package_share_directory('crazyflie')
    cf_server_yaml = os.path.join(cf_package_dir, 'config', 'server.yaml')
    cf_urdf = os.path.join(cf_package_dir, 'urdf', 'crazyflie_description.urdf')
    
    # Load package configuration files
    with open(cf_config_path, 'r') as f:
        cf_config = yaml.safe_load(f)

    # Load Crazyflie configuration files
    with open(cf_server_yaml, 'r') as f:
        cf_server_config = yaml.safe_load(f)
    with open(cf_urdf, 'r') as f:
        cf_robot_desc = f.read()

    # Create parameters for the Crazyflie server node
    cf_server_params = [cf_config] + [cf_server_config['/crazyflie_server']['ros__parameters']]
    cf_server_params[1]['robot_description'] = cf_robot_desc

    # Initialize action list
    actions = []

    # Add Crazyflie server node
    actions.append(
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=cf_server_params
        )
    )

    return actions