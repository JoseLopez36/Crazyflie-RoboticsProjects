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
    # Obtener las rutas a los archivos de configuración del paquete
    package_dir = get_package_share_directory('crazyflie_robotics_projects_pkg')
    mission_config_path = os.path.join(package_dir, 'config', 'mission.yaml')
    nodes_config_path = os.path.join(package_dir, 'config', 'nodes.yaml')
    launch_config_path = os.path.join(package_dir, 'config', 'launch.yaml')
    
    # Cargar archivos de configuración del paquete
    with open(launch_config_path, 'r') as f:
        launch = yaml.safe_load(f)
    with open(mission_config_path, 'r') as f:
        mission = yaml.safe_load(f)
    
    # Obtener las configuraciones de activación de los nodos
    nodes = {
        'planning': launch.get('planning', [True, 'info']),
        'darp': launch.get('darp', [True, 'info']),
        'control': launch.get('control', [True, 'info']),
        'transform': launch.get('transform', [True, 'info'])
    }

    # Obtener IDs de agentes desde mission.yaml
    agents_ids = []
    try:
        agents_ids = mission.get('/**', {}).get('ros__parameters', {}).get('agents', {}).get('ids', [])
    except Exception:
        agents_ids = []

    # Inicializar la lista de acciones
    actions = []

    # Condicionalmente agregar el nodo de planificación
    if nodes['planning'][0]:
        actions.append(
            Node(
                package='crazyflie_robotics_projects_pkg',
                executable='planning_node',
                name='planning_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['planning'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )

    # Condicionalmente agregar el nodo DARP
    if nodes['darp'][0]:
        actions.append(
            Node(
                package='crazyflie_robotics_projects_pkg',
                executable='darp_node',
                name='darp_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['darp'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )

    # Condicionalmente agregar N nodos de control (uno por agente)
    if nodes['control'][0]:
        for agent_id in agents_ids:
            actions.append(
                Node(
                    package='crazyflie_robotics_projects_pkg',
                    executable='control_node',
                    namespace=agent_id,
                    name='control_node',
                    output='screen',
                    arguments=['--ros-args', '--log-level', nodes['control'][1]],
                    parameters=[mission_config_path, {'agent_id': agent_id}]
                )
            )

    # Condicionalmente agregar el nodo de transformacion
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

    return actions