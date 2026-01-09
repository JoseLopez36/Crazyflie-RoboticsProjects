import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from vision_msgs.msg import Point2D
from crazyflie_robotics_projects_msgs.msg import Trajectory2D
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Takeoff, Arm, Land
from builtin_interfaces.msg import Duration

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Parámetros
        self.declare_parameter("agent_id", "")
        self.declare_parameter("target_altitude", 0.5)
        self.declare_parameter("acceptance_radius", 0.1)
        self.agent_id = self.get_parameter("agent_id").get_parameter_value().string_value
        self.target_altitude = self.get_parameter("target_altitude").get_parameter_value().double_value
        self.acceptance_radius = self.get_parameter("acceptance_radius").get_parameter_value().double_value

        # QoS para trayectorias
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS profile
        qos_profile_crazyflie = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Estado del agente
        self.position = PointStamped()  # PointStamped
        self.trajectory_points = []  # list[vision_msgs/Point2D]
        self.current_index = 0
        self.setpoint_count = 0
        self.agent_state = "INIT"  # INIT, TAKEOFF, MISSION

        # Suscripciones
        self.position_sub = self.create_subscription(
            PoseStamped,
            f"/{self.agent_id}/pose",
            self.position_callback,
            qos_profile_crazyflie
        )
        self.traj_sub = self.create_subscription(
            Trajectory2D,
            f"/{self.agent_id}/planning/trajectory",
            self.trajectory_callback,
            trajectory_qos
        )

        # Publishers
        self.setpoint_pub = self.create_publisher(
            Position,
            f"/{self.agent_id}/cmd_position",
            qos_profile_crazyflie
        )

        # Publicador: resto de trayectoria (solo visualización)
        self.remaining_traj_pub = self.create_publisher(
            Trajectory2D,
            f"/{self.agent_id}/planning/trajectory_remaining",
            trajectory_qos,
        )

        # Service clients for takeoff, land, and arm
        self.takeoff_client = self.create_client(Takeoff, f'/{self.agent_id}/takeoff')
        self.land_client = self.create_client(Land, f'/{self.agent_id}/land')
        self.arm_client = self.create_client(Arm, f'/{self.agent_id}/arm')

        # Timer: 10 Hz bucle de control
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Nodo de control iniciado para {self.agent_id}"
        )

    def position_callback(self, msg: PoseStamped):
        self.position.point = msg.pose.position

    def trajectory_callback(self, msg: Trajectory2D):
        self.trajectory_points = list(msg.points)
        self.current_index = 0
        self.publish_remaining_trajectory()

        self.get_logger().info(
            f"{self.agent_id}: Nueva trayectoria recibida (len={len(self.trajectory_points)}). Reiniciando seguimiento"
        )

    def publish_remaining_trajectory(self):
        if not self.trajectory_points:
            return
        start = max(0, min(int(self.current_index), len(self.trajectory_points)))
        out = Trajectory2D()
        # Re-publicar solo los puntos pendientes
        out.points = [
            Point2D(x=float(p.x), y=float(p.y))
            for p in self.trajectory_points[start:]
        ]
        self.remaining_traj_pub.publish(out)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        # Publish position command using crazyswarm2 Position message
        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)
        self.setpoint_pub.publish(msg)

    def arm(self):
        """Arm the Crazyflie using crazyswarm2 service"""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.agent_id}: Arm service not available')
            return
        
        req = Arm.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        self.get_logger().info(f'{self.agent_id}: Sending ARM command')

    def disarm(self):
        """Disarm the Crazyflie using crazyswarm2 service"""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.agent_id}: Arm service not available')
            return
        
        req = Arm.Request()
        req.arm = False
        future = self.arm_client.call_async(req)
        self.get_logger().info(f'{self.agent_id}: Sending DISARM command')

    def takeoff(self, height, duration_sec=2.0):
        """Takeoff the Crazyflie using crazyswarm2 service"""
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.agent_id}: Takeoff service not available')
            return
        
        req = Takeoff.Request()
        req.height = float(height)
        req.duration = Duration()
        req.duration.sec = int(duration_sec)
        req.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        req.group_mask = 0
        future = self.takeoff_client.call_async(req)
        self.get_logger().info(f'{self.agent_id}: Sending TAKEOFF command to height {height}m')

    def control_loop(self):
        if self.position is None:
            return

        current_pos = self.position

        # --- INIT ---
        if self.agent_state == "INIT":
            # Arm the drone
            self.arm()
            
            # After arming, request takeoff
            self.takeoff(self.target_altitude, duration_sec=2.0)
            self.agent_state = 'TAKEOFF'
            self.get_logger().info(f"{self.agent_id}: Starting TAKEOFF")

        # --- TAKEOFF ---
        elif self.agent_state == "TAKEOFF":
            self.publish_trajectory_setpoint(
                current_pos.point.x, current_pos.point.y, self.target_altitude
            )

            # Comprobar altitud
            if current_pos.point.z >= (self.target_altitude - 0.5):
                self.agent_state = "MISSION"
                self.get_logger().info(
                    f"{self.agent_id}: Despegue completado. Iniciando MISION"
                )

        # --- MISSION ---
        elif self.agent_state == "MISSION":
            if not self.trajectory_points:
                # No hay planificación: hover
                self.publish_trajectory_setpoint(
                    current_pos.point.x, current_pos.point.y, self.target_altitude
                )
                return

            idx = self.current_index

            if idx >= len(self.trajectory_points):
                target_pos = self.trajectory_points[-1]
            else:
                target_pos = self.trajectory_points[idx]

                dx = float(target_pos.x) - float(current_pos.point.x)
                dy = float(target_pos.y) - float(current_pos.point.y)
                d = math.sqrt(dx * dx + dy * dy)

                if d < self.acceptance_radius:
                    self.current_index += 1
                    if self.current_index < len(self.trajectory_points):
                        self.get_logger().info(
                            f"{self.agent_id} ha llegado al punto {idx}. Moviendo a {self.current_index}"
                        )
                        target_pos = self.trajectory_points[self.current_index]

            self.publish_trajectory_setpoint(
                target_pos.x, target_pos.y, self.target_altitude
            )
            self.publish_remaining_trajectory()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


