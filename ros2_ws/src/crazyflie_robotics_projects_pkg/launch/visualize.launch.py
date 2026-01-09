from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Declare the config file argument
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='default.rviz',
        description='RViz config file name'
    )

    return LaunchDescription([
        config_arg,
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

    # Get the RViz config file name from launch argument
    config = context.launch_configurations.get('config', 'default.rviz')
    rviz_config_file = '/root/Crazyflie-RoboticsProjects/ros2_ws/src/crazyflie_robotics_projects_pkg/rviz/' + config
    
    # Obtener las configuraciones de activación de los nodos
    nodes = {
        'visualization': launch.get('visualization', [True, 'info'])
    }

    # Inicializar la lista de acciones
    actions = []

    # Agregar nodo de RViz
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2', 
            name='rviz2',
            arguments=['-d', str(rviz_config_file)]
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