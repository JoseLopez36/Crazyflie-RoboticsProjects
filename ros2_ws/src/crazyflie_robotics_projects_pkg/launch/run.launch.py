from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Declarar argumento de lanzamiento para el archivo de configuración de Crazyflie
    cf_config_arg = DeclareLaunchArgument(
        'cf_config',
        default_value='simulation_config.yaml',
        description='Nombre del archivo de configuración de Crazyflie (debe estar en config/crazyflie/)'
    )
    
    return LaunchDescription([
        cf_config_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    # Obtener el valor del argumento de lanzamiento
    cf_config_filename = context.launch_configurations.get('cf_config', 'simulation_config.yaml')
    
    # Obtener las rutas a los archivos de configuración del paquete
    package_dir = get_package_share_directory('crazyflie_robotics_projects_pkg')
    mission_config_path = os.path.join(package_dir, 'config', 'mission.yaml')
    nodes_config_path = os.path.join(package_dir, 'config', 'nodes.yaml')
    cf_config_path = os.path.join(package_dir, 'config', 'crazyflie', cf_config_filename)
    launch_config_path = os.path.join(package_dir, 'config', 'launch.yaml')

    # Obtener las rutas a los archivos de configuración de Crazyflie
    cf_package_dir = get_package_share_directory('crazyflie')
    cf_server_yaml = os.path.join(cf_package_dir, 'config', 'server.yaml')
    cf_urdf = os.path.join(cf_package_dir, 'urdf', 'crazyflie_description.urdf')
    
    # Cargar archivos de configuración del paquete
    with open(launch_config_path, 'r') as f:
        launch = yaml.safe_load(f)
    with open(mission_config_path, 'r') as f:
        mission = yaml.safe_load(f)
    with open(nodes_config_path, 'r') as f:
        nodes_cfg = yaml.safe_load(f)
    with open(cf_config_path, 'r') as f:
        cf_config = yaml.safe_load(f)

    # Cargar archivos de configuración de Crazyflie
    with open(cf_server_yaml, 'r') as f:
        cf_server_config = yaml.safe_load(f)
    with open(cf_urdf, 'r') as f:
        cf_robot_desc = f.read()

    # Crear parámetros para el nodo de server Crazyflie
    cf_server_params = [cf_config] + [cf_server_config['/crazyflie_server']['ros__parameters']]
    cf_server_params[1]['robot_description'] = cf_robot_desc
    
    # Obtener las configuraciones de activación de los nodos
    nodes = {
        'planning': launch.get('planning', [True, 'info']),
        'darp': launch.get('darp', [True, 'info']),
        'control': launch.get('control', [True, 'info']),
        'transform': launch.get('transform', [True, 'info']),
        'visualization': launch.get('visualization', [True, 'info'])
    }

    # Obtener IDs de agentes desde mission.yaml
    agents_ids = []
    try:
        agents_ids = mission.get('/**', {}).get('ros__parameters', {}).get('agents', {}).get('ids', [])
    except Exception:
        agents_ids = []

    # Defaults de control_node desde nodes.yaml (opcional)
    control_params = nodes_cfg.get('control_node', {}).get('ros__parameters', {}) if isinstance(nodes_cfg, dict) else {}

    # Inicializar la lista de acciones
    actions = []

    # Agregar nodo de server Crazyflie
    actions.append(
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=cf_server_params
        )
    )

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
                    parameters=[mission_config_path, {'agent_id': agent_id}, control_params]
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

    # Condicionalmente agregar el nodo de visualización
    if nodes['visualization'][0]:
        actions.append(
            Node(
                package='crazyflie_robotics_projects_pkg',
                executable='visualization_node',
                name='visualization_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['visualization'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )

    return actions