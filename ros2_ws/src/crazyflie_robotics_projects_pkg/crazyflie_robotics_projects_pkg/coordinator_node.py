import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from crazyflie_robotics_projects_msgs.srv import DarpPetition
from crazyflie_robotics_projects_msgs.msg import Trajectory2D
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Takeoff, Land, Arm
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Point2D
from builtin_interfaces.msg import Duration
import math

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        # Agents parameters
        self.declare_parameter('agents.ids', [''])

        # DARP parameters
        self.declare_parameter('tasks.min_x', 0)
        self.declare_parameter('tasks.max_x', 0)
        self.declare_parameter('tasks.min_y', 0)
        self.declare_parameter('tasks.max_y', 0)
        self.declare_parameter('tasks.obstacles_positions_x', [0.0])
        self.declare_parameter('tasks.obstacles_positions_y', [0.0])

        # Other parameters
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('acceptance_radius', 0.5)

        # Get parameters
        self.agent_ids = self.get_parameter('agents.ids').get_parameter_value().string_array_value
        self.min_x = self.get_parameter('tasks.min_x').get_parameter_value().integer_value
        self.max_x = self.get_parameter('tasks.max_x').get_parameter_value().integer_value
        self.min_y = self.get_parameter('tasks.min_y').get_parameter_value().integer_value
        self.max_y = self.get_parameter('tasks.max_y').get_parameter_value().integer_value
        self.obstacles_positions_x = self.get_parameter('tasks.obstacles_positions_x').get_parameter_value().double_array_value
        self.obstacles_positions_y = self.get_parameter('tasks.obstacles_positions_y').get_parameter_value().double_array_value
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value
        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value

        if not self.agent_ids:
            self.get_logger().warn('No agents configured in agents.ids; the coordinator will not request DARP.')
        
        # DARP client
        self.darp_client = self.create_client(DarpPetition, 'darp_service')
        while not self.darp_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('DARP service not available, waiting for it...')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State management of the UAVs
        self.position_subscribers = {}
        self.position_publishers = {}
        self.takeoff_clients = {}
        self.land_clients = {}
        self.arm_clients = {}
        
        self.positions = {}
        self.trajectories = {} # Key: agent_id, Value: List of Point2D
        self.current_wp_indices = {} # Key: agent_id, Value: int
        self.agent_states = {} # States: INIT, TAKEOFF, MISSION, LANDED
        self.position_received = {} # Key: agent_id, Value: bool
        self.takeoff_requested = {} # Key: agent_id, Value: bool
        self.arm_requested = {} # Key: agent_id, Value: bool
        self.darp_request_sent = False

        # Iterate over the agents
        for agent_id in self.agent_ids:
            # Subscribers
            # Subscription to the agent position (crazyswarm2 uses /<agent_id>/pose)
            self.position_subscribers[agent_id] = self.create_subscription(
                PoseStamped,
                f'/{agent_id}/pose',
                lambda msg, uid=agent_id: self.position_callback(msg, uid),
                qos_profile
            )

            # Publishers
            # Publisher of position commands (crazyswarm2 uses /<agent_id>/cmd_position)
            self.position_publishers[agent_id] = self.create_publisher(
                Position,
                f'/{agent_id}/cmd_position',
                qos_profile
            )
            
            # Service clients for takeoff, land, and arm
            self.takeoff_clients[agent_id] = self.create_client(Takeoff, f'/{agent_id}/takeoff')
            self.land_clients[agent_id] = self.create_client(Land, f'/{agent_id}/land')
            self.arm_clients[agent_id] = self.create_client(Arm, f'/{agent_id}/arm')

            self.positions[agent_id] = PoseStamped()
            self.trajectories[agent_id] = []
            self.current_wp_indices[agent_id] = 0
            self.agent_states[agent_id] = 'INIT'
            self.position_received[agent_id] = False
            self.takeoff_requested[agent_id] = False
            self.arm_requested[agent_id] = False

        # Timer for the control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Waiting for initial positions of the agents to request DARP...')

    def position_callback(self, msg, agent_id):
        self.positions[agent_id] = msg
        self.position_received[agent_id] = True

        if not self.darp_request_sent and self._all_positions_received():
            self.get_logger().info("Initial positions received. Requesting DARP...")
            self.request_darp()

    def _all_positions_received(self):
        return (
            bool(self.agent_ids) and
            len(self.position_received) == len(self.agent_ids) and
            all(self.position_received.values())
        )

    def publish_position_command(self, agent_id, x, y, z, yaw=0.0):
        # Publish position command using crazyswarm2 Position message
        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)
        self.position_publishers[agent_id].publish(msg)

    def arm(self, agent_id):
        """Arm the Crazyflie using crazyswarm2 service"""
        if self.arm_requested.get(agent_id, False):
            return
        
        if not self.arm_clients[agent_id].wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_id}: Arm service not available')
            return
        
        req = Arm.Request()
        req.arm = True
        future = self.arm_clients[agent_id].call_async(req)
        self.arm_requested[agent_id] = True
        self.get_logger().info(f'{agent_id}: Sending ARM command')

    def disarm(self, agent_id):
        """Disarm the Crazyflie using crazyswarm2 service"""
        if not self.arm_clients[agent_id].wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_id}: Arm service not available')
            return
        
        req = Arm.Request()
        req.arm = False
        future = self.arm_clients[agent_id].call_async(req)
        self.get_logger().info(f'{agent_id}: Sending DISARM command')

    def takeoff(self, agent_id, height, duration_sec=2.0):
        """Takeoff the Crazyflie using crazyswarm2 service"""
        if self.takeoff_requested.get(agent_id, False):
            return
        
        if not self.takeoff_clients[agent_id].wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_id}: Takeoff service not available')
            return
        
        req = Takeoff.Request()
        req.height = float(height)
        req.duration = Duration()
        req.duration.sec = int(duration_sec)
        req.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        req.group_mask = 0
        future = self.takeoff_clients[agent_id].call_async(req)
        self.takeoff_requested[agent_id] = True
        self.get_logger().info(f'{agent_id}: Sending TAKEOFF command to height {height}m')

    def request_darp(self):
        if not self.agent_ids:
            self.get_logger().error("No agents configured, cannot request DARP.")
            return

        if self.darp_request_sent:
            return

        if not self._all_positions_received():
            self.get_logger().warn("Not all positions of the agents have been received. DARP is not requested yet.")
            return

        self.get_logger().info("Requesting DARP algorithm...")
        self.darp_request_sent = True
        req = DarpPetition.Request()
        req.min_x = self.min_x
        req.max_x = self.max_x
        req.min_y = self.min_y
        req.max_y = self.max_y
        req.visualization = False

        # Obstacles
        obs_x = self.obstacles_positions_x
        obs_y = self.obstacles_positions_y
        if len(obs_x) != len(obs_y):
            self.get_logger().warn("Lists of obstacles with different lengths, the least number of elements will be used.")
        for x, y in zip(obs_x, obs_y):
            p = Point2D()
            p.x = x
            p.y = y
            req.obstacle_points.append(p)

        # Initial positions from the current measurements
        for agent_id in self.agent_ids:
            pos = self.positions[agent_id].pose.position
            p = Point2D()
            p.x = pos.x
            p.y = pos.y
            req.initial_positions.append(p)

        future = self.darp_client.call_async(req)
        future.add_done_callback(self.darp_callback)

    def darp_callback(self, future):
        try:
            response = future.result()
            if not response.trajectories:
                self.get_logger().error("DARP returned empty trajectories")
                self.darp_request_sent = False
                return

            self.get_logger().info(f"DARP solution received with {len(response.trajectories)} trajectories")
            
            for i, traj in enumerate(response.trajectories):
                if i < len(self.agent_ids):
                    agent_id = self.agent_ids[i]
                    self.trajectories[agent_id] = traj.points
                    self.current_wp_indices[agent_id] = 0
                    self.get_logger().info(f"Trajectory assigned of length {len(traj.points)} to {agent_id}")
                else:
                    self.get_logger().warn(f"More trajectories than UAVs: Trajectory {i} ignored")

        except Exception as e:
            self.get_logger().error(f'Call to service failed: {e}')
            self.darp_request_sent = False

    def control_loop(self):
        for i, agent_id in enumerate(self.agent_ids):
            # Get the state of the agent
            state = self.agent_states[agent_id]
            current_pos = self.positions[agent_id]
            
            # --- State: INIT (Initial wait and Arming) ---
            if state == 'INIT':
                # Wait for position to be received
                if not self.position_received.get(agent_id, False):
                    continue
                
                # Arm the drone
                if not self.arm_requested.get(agent_id, False):
                    self.arm(agent_id)
                    continue
                
                # After arming, request takeoff
                if not self.takeoff_requested.get(agent_id, False):
                    self.takeoff(agent_id, self.target_altitude, duration_sec=2.0)
                    self.agent_states[agent_id] = 'TAKEOFF'
                    self.get_logger().info(f"{agent_id}: Starting TAKEOFF")

            # --- State: TAKEOFF (Vertical takeoff) ---
            elif state == 'TAKEOFF':
                # Maintain position at current x,y and target altitude
                current_x = current_pos.pose.position.x
                current_y = current_pos.pose.position.y
                self.publish_position_command(agent_id, current_x, current_y, self.target_altitude)
                
                # Check altitude (with margin of 0.5m)
                if current_pos.pose.position.z >= (self.target_altitude - 0.5):
                    self.agent_states[agent_id] = 'MISSION'
                    self.get_logger().info(f"{agent_id}: Takeoff completed. Starting MISSION")

            # --- State: MISSION (Follow DARP trajectory) ---
            elif state == 'MISSION':
                traj_points = self.trajectories.get(agent_id, [])
                if not traj_points:
                    # If there is no trajectory, maintain current position (hover)
                    current_x = current_pos.pose.position.x
                    current_y = current_pos.pose.position.y
                    self.publish_position_command(agent_id, current_x, current_y, self.target_altitude)
                    continue

                idx = self.current_wp_indices.get(agent_id, 0)
                
                if idx >= len(traj_points):
                    # End of trajectory: Maintain last point
                    target_pos = traj_points[-1]
                else:
                    target_pos = traj_points[idx]
                    
                    # Calculate distance to target
                    current_x = current_pos.pose.position.x
                    current_y = current_pos.pose.position.y
                    dx = float(target_pos.x) - float(current_x)
                    dy = float(target_pos.y) - float(current_y)
                    d = math.sqrt(dx**2 + dy**2)
                    
                    if d < self.acceptance_radius:
                        self.current_wp_indices[agent_id] += 1
                        if self.current_wp_indices[agent_id] < len(traj_points):
                            self.get_logger().info(f"{agent_id} has reached point {idx}. Moving to {self.current_wp_indices[agent_id]}")
                            target_pos = traj_points[self.current_wp_indices[agent_id]]

                # Publish trajectory point
                self.publish_position_command(agent_id, target_pos.x, target_pos.y, self.target_altitude)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
