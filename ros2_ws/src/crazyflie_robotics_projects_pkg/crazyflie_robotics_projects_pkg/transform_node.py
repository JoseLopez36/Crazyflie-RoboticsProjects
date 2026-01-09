#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from crazyflie_interfaces.msg import Position
from geometry_msgs.msg import PointStamped, TransformStamped

class TransformNode(Node):

    def __init__(self):
        super().__init__('transform_node')

        # Log
        self.get_logger().info("Iniciando nodo de gestion de transformaciones...")

        # Obtener parámetros
        self.declare_parameter('agents.ids', [''])
        self.declare_parameter('agents.initial_positions_x', [0.0])
        self.declare_parameter('agents.initial_positions_y', [0.0])
        self.agents_ids = self.get_parameter('agents.ids').get_parameter_value().string_array_value
        self.initial_positions_x = self.get_parameter('agents.initial_positions_x').get_parameter_value().double_array_value
        self.initial_positions_y = self.get_parameter('agents.initial_positions_y').get_parameter_value().double_array_value

        # Crear diccionario de orígenes de marcos locales
        self.local_frame_origins = {}
        for i in range(len(self.agents_ids)):
            self.local_frame_origins[self.agents_ids[i]] = (self.initial_positions_x[i], self.initial_positions_y[i], 0.0)

        # Inicializar diccionarios
        self.position_cf_subscribers = {}
        self.position_ros2_publishers = {}
        self.trajectory_setpoint_cf_publishers = {}
        self.trajectory_setpoint_ros2_subscribers = {}

        # Crear broadcast de transformaciones
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS para Crazyflie
        qos_profile_cf = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS para posiciones y setpoints
        qos_profile_points = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Iterar sobre los UAVs para crear suscriptores y publicadores
        for agent_id in self.agents_ids:
            # Crear suscriptores de Crazyflie
            self.position_cf_subscribers[agent_id] = self.create_subscription(
                PoseStamped,
                f"/{agent_id}/pose",
                lambda msg, uid=agent_id: self.position_cf_callback(msg, uid),
                qos_profile_cf
            )
            # Crear publicadores para Crazyflie
            self.trajectory_setpoint_cf_publishers[agent_id] = self.create_publisher(
                Position,
                f"/{agent_id}/cmd_position",
                qos_profile_cf
            )
            # Crear suscriptores de ROS2
            self.trajectory_setpoint_ros2_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f'/{agent_id}/control/setpoint',
                lambda msg, uid=agent_id: self.trajectory_setpoint_ros2_callback(msg, uid),
                qos_profile_points
            )
            # Crear publicadores para ROS2
            self.position_ros2_publishers[agent_id] = self.create_publisher(
                PointStamped,
                f'/{agent_id}/state/position',
                qos_profile_points
            )
        
        # Publicar marco global
        self.publish_global_frame()
    
        # Log
        self.get_logger().info("Nodo de gestion de transformaciones iniciado")

    def position_cf_callback(self, msg, agent_id):
        # Callback para recibir posición de Crazyflie y publicarla en ROS2
        # Crear mensaje de posición para ROS2
        position_msg = PointStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'global'
        position_msg.point = msg.pose.position

        # Publicar posición en ROS2
        self.position_ros2_publishers[agent_id].publish(position_msg)

        # Publicar marco del cuerpo del UAV
        self.publish_body_frame(agent_id, position_msg.point.x, position_msg.point.y, position_msg.point.z)

    def trajectory_setpoint_ros2_callback(self, msg, agent_id):
        # Callback para recibir setpoints de ROS2 y publicarlos en Crazyflie
        # Crear mensaje Position para Crazyflie
        setpoint_msg = Position()
        setpoint_msg.x = float(msg.point.x)
        setpoint_msg.y = float(msg.point.y)
        setpoint_msg.z = float(msg.point.z)
        setpoint_msg.yaw = 0.0
        
        # Publicar setpoint a Crazyflie
        self.trajectory_setpoint_cf_publishers[agent_id].publish(setpoint_msg)

    def publish_global_frame(self):
        # Publicar transformación del frame 'global' (origen)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'global'
        
        # Establecer posición relativa al origen
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Establecer orientación (identidad)
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)

    def publish_body_frame(self, agent_id, x, y, z):
        # Publicar transformación del frame 'local' al frame 'body' (posición y orientación del UAV)
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global'
        t.child_frame_id = f'{agent_id}/body'
        
        # Establecer posición relativa al marco local
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación dinámica
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    transform_node = TransformNode()
    rclpy.spin(transform_node)
    transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()