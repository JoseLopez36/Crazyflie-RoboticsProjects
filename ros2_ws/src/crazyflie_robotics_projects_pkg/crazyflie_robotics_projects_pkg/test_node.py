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
from pynput import keyboard

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Agents parameters
        self.declare_parameter('agents.ids', [''])

        # Other parameters
        self.declare_parameter('target_altitude', 0.5)

        # Get parameters
        self.agent_ids = self.get_parameter('agents.ids').get_parameter_value().string_array_value
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value

        if not self.agent_ids:
            self.get_logger().warn('No agents configured in agents.ids; the test node will not start.')
        
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
        self.agent_states = {} # States: INIT, TAKEOFF, MISSION, LAND, LANDED
        self.position_received = {} # Key: agent_id, Value: bool
        self.initial_position = {} # Key: agent_id, Value: PoseStamped
        self.takeoff_requested = {} # Key: agent_id, Value: bool
        self.arm_requested = {} # Key: agent_id, Value: bool
        self.land_requested = {} # Key: agent_id, Value: bool
        self.space_pressed = False  # Flag for space key press
        self.counter = {} # Key: agent_id, Value: int

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
            self.agent_states[agent_id] = 'INIT'
            self.position_received[agent_id] = False
            self.initial_position[agent_id] = PoseStamped()
            self.takeoff_requested[agent_id] = False
            self.arm_requested[agent_id] = False
            self.land_requested[agent_id] = False

            # 10 second counter
            self.counter[agent_id] = 0

        # Timer for the control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Start keyboard listener in a separate thread
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()
        self.get_logger().info('Keyboard listener started. Press SPACE in MISSION state to land agents.')

        self.get_logger().info('Waiting for initial positions of the agents...')

    def on_key_press(self, key):
        """Callback for keyboard key press events"""
        try:
            if key == keyboard.Key.space:
                self.space_pressed = True
                self.get_logger().info('SPACE key pressed - Landing command received')
        except AttributeError:
            pass

    def position_callback(self, msg, agent_id):
        self.positions[agent_id] = msg
        self.position_received[agent_id] = True

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

    def land(self, agent_id, height=0.0, duration_sec=2.0):
        """Land the Crazyflie using crazyswarm2 service"""
        if self.land_requested.get(agent_id, False):
            return
        
        if not self.land_clients[agent_id].wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_id}: Land service not available')
            return
        
        req = Land.Request()
        req.height = float(height)
        req.duration = Duration()
        req.duration.sec = int(duration_sec)
        req.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        req.group_mask = 0
        future = self.land_clients[agent_id].call_async(req)
        self.land_requested[agent_id] = True
        self.get_logger().info(f'{agent_id}: Sending LAND command')

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
                self.initial_position[agent_id] = current_pos
                initial_pos = self.initial_position[agent_id]
                self.publish_position_command(agent_id, initial_pos.pose.position.x, initial_pos.pose.position.y, self.target_altitude)
                
                # Check altitude (with margin of 0.5m)
                if current_pos.pose.position.z >= (self.target_altitude - 0.1):
                    self.agent_states[agent_id] = 'MISSION'
                    self.get_logger().info(f"{agent_id}: Takeoff completed. Starting MISSION")

            # --- State: MISSION (Hover at target altitude) ---
            elif state == 'MISSION':
                initial_pos = self.initial_position[agent_id]
                if (agent_id == 'cf_0'):
                    self.publish_position_command(agent_id, initial_pos.pose.position.x, initial_pos.pose.position.y, self.target_altitude)
                elif (agent_id == 'cf_1'):
                    self.publish_position_command(agent_id, initial_pos.pose.position.x, initial_pos.pose.position.y, self.target_altitude)

                self.counter[agent_id] += 1
                if self.counter[agent_id] >= 50:
                    self.agent_states[agent_id] = 'LAND'
                    self.get_logger().info(f"{agent_id}: 5 seconds passed. Starting LAND")

            # --- State: LAND (Landing) ---
            elif state == 'LAND':
                # Maintain current x,y position while descending
                current_x = current_pos.pose.position.x
                current_y = current_pos.pose.position.y
                # Gradually decrease altitude
                current_z = current_pos.pose.position.z
                target_z = max(0.0, current_z - 0.1)  # Descend at 0.1m per control cycle
                self.publish_position_command(agent_id, current_x, current_y, target_z)
                
                # Check if landed (altitude below 0.2m)
                if current_pos.pose.position.z < 0.2:
                    self.agent_states[agent_id] = 'LANDED'
                    self.get_logger().info(f"{agent_id}: Landing completed. State: LANDED")

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
