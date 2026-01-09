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
        get_package_share_directory('crazyflie_robotics_projects_pkg'),
        'config',
        'hardware_config.yaml')

    mission_yaml = os.path.join(
        get_package_share_directory('crazyflie_robotics_projects_pkg'),
        'config',
        'mission.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    # Cargar configuración de misión (para obtener agents.ids)
    with open(mission_yaml, 'r') as f:
        mission = yaml.safe_load(f)

    # Obtener IDs de agentes desde mission.yaml
    agents_ids = []
    try:
        agents_ids = mission.get('/**', {}).get('ros__parameters', {}).get('agents', {}).get('ids', [])
    except Exception:
        agents_ids = []

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

    # Start test node
    test_node = Node(
        package='crazyflie_robotics_projects_pkg',
        executable='test_node',
        name='test_node',
        output='screen',
        parameters=[
            mission_yaml
        ]
    )
    launch_description.append(test_node)

    # # Start planning node
    # planning_node = Node(
    #     package='crazyflie_robotics_projects_pkg',
    #     executable='planning_node',
    #     name='planning_node',
    #     output='screen',
    #     parameters=[
    #         mission_yaml
    #     ]
    # )
    # launch_description.append(planning_node)

    # # Start control node
    # for agent_id in agents_ids:
    #     launch_description.append(
    #         Node(
    #             package='crazyflie_robotics_projects_pkg',
    #             executable='control_node',
    #             namespace=agent_id,
    #             name='control_node',
    #             output='screen',
    #             parameters=[mission_yaml, {'agent_id': agent_id}]
    #         )
    #     )

    # # Start visualization node
    # visualization_node = Node(
    #     package='crazyflie_robotics_projects_pkg',
    #     executable='visualization_node',
    #     name='visualization_node',
    #     output='screen',
    #     parameters=[
    #         mission_yaml
    #     ]
    # )
    # launch_description.append(visualization_node)
    
    # Start Rviz2 node
    launch_description.append(       
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('crazyflie_robotics_projects_pkg'), 'config', 'config.rviz')],
            parameters=[{
                "use_sim_time": True,
            }]
        )
        
    )
    return LaunchDescription(launch_description)
