import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Load crazyflie configuration
    crazyflies_yaml = os.path.join(
        get_package_share_directory('roboticsprojects_crazyflie'),
        'config',
        'single_agent_crazyflie_sim.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # Server parameters
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # Robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc
    
    launch_description = []

    # Start crazyflie server node
    launch_description.append(
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=server_params
        ))
    
    # Start TransformWorld2Odom node
    launch_description.append(
        Node(
            package='roboticsprojects_crazyflie',
            executable='TransformWorld2Odom.py',
            name='TransformWorld2Odom',
            output='screen'
        ))
        
    # Start vel_mux node
    vel_mux_node = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name='vel_mux',
        output='screen',
        namespace='cf',
        parameters=[
            {"hover_height": 1.0},
            {"incoming_twist_topic": "cmd_vel"},
            {"robot_prefix": "/cf"}
        ]
    )
    launch_description.append(vel_mux_node)
    
    # Start Rviz2 node
    launch_description.append(       
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('roboticsprojects_crazyflie'), 'config', 'config.rviz')],
            parameters=[{
                "use_sim_time": True,
            }]
        )
        
    )
    return LaunchDescription(launch_description)
