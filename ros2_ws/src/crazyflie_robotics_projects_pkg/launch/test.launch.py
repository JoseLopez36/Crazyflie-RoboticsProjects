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
    
    # Obtener las configuraciones de activación de los nodos
    nodes = {
        'transform': launch.get('transform', [True, 'info'])
    }

    # Inicializar la lista de acciones
    actions = []

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

    # Agregar nodo de test
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