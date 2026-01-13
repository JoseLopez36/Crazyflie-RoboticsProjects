import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Point2D

from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from crazyflie_robotics_projects_msgs.srv import DarpPetition
from crazyflie_robotics_projects_msgs.msg import Trajectory2D

class PlanningNode(Node):
    def __init__(self):
        super().__init__("planning_node")

        # Parameters
        self.declare_parameter("agents.ids", [""])
        self.declare_parameter("cell_size", 2.0)
        self.declare_parameter("visited.update_period", 0.5)
        self.declare_parameter("tasks.min_x", 0.0)
        self.declare_parameter("tasks.max_x", 0.0)
        self.declare_parameter("tasks.min_y", 0.0)
        self.declare_parameter("tasks.max_y", 0.0)
        self.declare_parameter("tasks.obstacles_positions_x", [0.0])
        self.declare_parameter("tasks.obstacles_positions_y", [0.0])
        self.agent_ids = self.get_parameter("agents.ids").get_parameter_value().string_array_value
        self.cell_size = self.get_parameter("cell_size").get_parameter_value().double_value
        self.visited_update_period = self.get_parameter("visited.update_period").get_parameter_value().double_value
        self.min_x = self.get_parameter("tasks.min_x").get_parameter_value().double_value
        self.max_x = self.get_parameter("tasks.max_x").get_parameter_value().double_value
        self.min_y = self.get_parameter("tasks.min_y").get_parameter_value().double_value
        self.max_y = self.get_parameter("tasks.max_y").get_parameter_value().double_value
        self.obstacles_positions_x = self.get_parameter("tasks.obstacles_positions_x").get_parameter_value().double_array_value
        self.obstacles_positions_y = self.get_parameter("tasks.obstacles_positions_y").get_parameter_value().double_array_value

        # Grid (cache)
        self.rows, self.cols = self.compute_rows_cols()

        # Visited map state: cells already covered by any agent
        self.visited_mask = set()  # set[int] of flat indices (row*cols+col)
        self.static_obstacle_cells = self.compute_static_obstacle_cells()
        self.zones_current = None  # list[int] with visited forced to 0
        self.last_published_zones = None  # list[int]

        # QoS for trajectories
        qos_profile_trajectory = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS for positions and setpoints
        qos_profile_points = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for zones
        qos_profile_zones = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Position subscriptions per agent
        self.position_subscribers = {}
        self.positions = {}  # agent_id -> PointStamped
        self.position_received = {}  # agent_id -> bool

        for agent_id in self.agent_ids:
            self.positions[agent_id] = None
            self.position_received[agent_id] = False
            self.position_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f"/{agent_id}/state/position",
                lambda msg, uid=agent_id: self.position_callback(msg, uid),
                qos_profile_points
            )

        # Trajectory publishers per agent
        self.trajectory_publishers = {}
        for agent_id in self.agent_ids:
            self.trajectory_publishers[agent_id] = self.create_publisher(
                Trajectory2D, 
                f"/{agent_id}/planning/trajectory",
                qos_profile_trajectory
            )

        # Zones publisher
        self.zones_pub = self.create_publisher(
            Int32MultiArray,
            "/planning/zones",
            qos_profile_zones,
        )

        # DARP service client
        self.darp_client = self.create_client(DarpPetition, "darp_service")
        while not self.darp_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("DARP service not available, waiting again...")

        self.plan_requested = False
        self.plan_done = False

        # Timer to request the initial plan once all positions have been received
        self.timer = self.create_timer(0.5, self.maybe_request_initial_plan)

        # Timer to mark visited cells and refresh zones
        self.visited_timer = self.create_timer(
            float(self.visited_update_period) if self.visited_update_period > 0.0 else 0.5,
            self.update_visited_from_positions,
        )

        # On-demand replanning service
        self.replan_srv = self.create_service(Trigger, "/planning/replan", self.replan_callback)

        self.get_logger().info("Planning node started. Waiting for initial positions...")

    def position_callback(self, msg: PointStamped, agent_id: str):
        self.positions[agent_id] = msg
        self.position_received[agent_id] = True

    def all_positions_received(self) -> bool:
        return all(self.position_received.get(agent_id, False) for agent_id in self.agent_ids)

    def maybe_request_initial_plan(self):
        if self.plan_done or self.plan_requested:
            return
        if not self.all_positions_received():
            return

        self.plan_requested = True
        self.request_darp(include_visited_as_obstacles=False)

    def request_darp(self, include_visited_as_obstacles: bool):
        if include_visited_as_obstacles:
            self.get_logger().info("DARP algorithm request (replanning with visited)...")
        else:
            self.get_logger().info("DARP algorithm request (initial planning)...")
        req = DarpPetition.Request()
        req.min_x = float(self.min_x)
        req.max_x = float(self.max_x)
        req.min_y = float(self.min_y)
        req.max_y = float(self.max_y)
        req.visualization = False

        # Obstacles from parameters
        for x, y in zip(self.obstacles_positions_x, self.obstacles_positions_y):
            p = Point2D()
            p.x = float(x)
            p.y = float(y)
            req.obstacle_points.append(p)

        # Initial positions from subscriptions
        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            if pos is None:
                continue
            p = Point2D()
            p.x = float(pos.point.x)
            p.y = float(pos.point.y)
            req.initial_positions.append(p)

        # Visited cells as obstacles (replanning only)
        if include_visited_as_obstacles:
            start_cells = set()
            for agent_id in self.agent_ids:
                pos = self.positions.get(agent_id)
                if pos is None:
                    continue
                cell = self.world_to_cell(float(pos.point.x), float(pos.point.y))
                if cell is not None:
                    start_cells.add(int(cell))

            for cell in sorted(self.visited_mask):
                if cell in start_cells:
                    continue
                if cell in self.static_obstacle_cells:
                    continue
                pt = self.cell_to_world_center(int(cell))
                if pt is None:
                    continue
                p = Point2D()
                p.x = float(pt[0])
                p.y = float(pt[1])
                req.obstacle_points.append(p)

        future = self.darp_client.call_async(req)
        future.add_done_callback(self.darp_callback)

    def darp_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"DARP service call failed: {e}")
            self.plan_requested = False
            return

        if not response.trajectories:
            self.get_logger().error("DARP returned empty trajectories")
            self.plan_requested = False
            return

        self.get_logger().info(
            f"DARP solution received with {len(response.trajectories)} trajectories"
        )

        # Cache and publish zones for visualization (visited = 0)
        if response.zones:
            self.zones_current = list(response.zones)
            self.apply_visited_to_zones_inplace(self.zones_current)
            self.publish_zones(self.zones_current)

        # Publish trajectories per agent based on index order
        for i, traj in enumerate(response.trajectories):
            if i >= len(self.agent_ids):
                self.get_logger().warn(
                    f"More trajectories than UAVs: trajectory {i} ignored"
                )
                continue
            agent_id = self.agent_ids[i]
            msg = Trajectory2D()
            msg.points = traj.points
            self.trajectory_publishers[agent_id].publish(msg)
            self.get_logger().info(
                f"Published trajectory (len={len(msg.points)}) on /{agent_id}/planning/trajectory"
            )

        self.plan_done = True
        self.plan_requested = False

    def replan_callback(self, request, response):
        if self.plan_requested:
            response.success = False
            response.message = "Replan rejected: there is already a DARP request in progress"
            return response
        if not self.all_positions_received():
            response.success = False
            response.message = "Replan rejected: initial positions are missing"
            return response

        self.plan_requested = True
        self.get_logger().info("Replan requested: calling DARP with visited as obstacles")
        self.request_darp(include_visited_as_obstacles=True)
        response.success = True
        response.message = "Replan requested"
        return response

    def compute_rows_cols(self):
        extent_x = float(self.max_x) - float(self.min_x)
        extent_y = float(self.max_y) - float(self.min_y)
        cols = int(round(float(extent_x) / float(self.cell_size))) if self.cell_size > 0.0 else 0
        rows = int(round(float(extent_y) / float(self.cell_size))) if self.cell_size > 0.0 else 0
        return max(0, rows), max(0, cols)

    def world_to_cell(self, x: float, y: float):
        if self.rows <= 0 or self.cols <= 0 or self.cell_size <= 0.0:
            return None
        gx = int(math.floor((x - float(self.min_x)) / float(self.cell_size)))
        gy = int(math.floor((y - float(self.min_y)) / float(self.cell_size)))
        gx = max(0, min(self.cols - 1, gx))
        gy = max(0, min(self.rows - 1, gy))
        return int(gy * self.cols + gx)

    def cell_to_world_center(self, cell: int):
        if self.rows <= 0 or self.cols <= 0 or self.cell_size <= 0.0:
            return None
        if cell < 0 or cell >= (self.rows * self.cols):
            return None
        r = int(cell // self.cols)
        c = int(cell % self.cols)
        x = float(self.min_x) + (float(c) + 0.5) * float(self.cell_size)
        y = float(self.min_y) + (float(r) + 0.5) * float(self.cell_size)
        return (x, y)

    def compute_static_obstacle_cells(self):
        cells = set()
        for x, y in zip(self.obstacles_positions_x, self.obstacles_positions_y):
            cell = self.world_to_cell(float(x), float(y))
            if cell is not None:
                cells.add(int(cell))
        return cells

    def apply_visited_to_zones_inplace(self, zones_list):
        if zones_list is None:
            return
        for cell in self.visited_mask:
            idx = int(cell)
            if idx < 0 or idx >= len(zones_list):
                continue
            # Do not overwrite obstacles
            if int(zones_list[idx]) == -1:
                continue
            zones_list[idx] = 0

    def publish_zones(self, zones_list):
        if zones_list is None:
            return
        if self.rows <= 0 or self.cols <= 0:
            return
        expected = int(self.rows * self.cols)
        data = list(zones_list[:expected]) if len(zones_list) >= expected else list(zones_list)

        zones_msg = Int32MultiArray()
        zones_msg.layout.dim = [
            MultiArrayDimension(label="rows", size=int(self.rows), stride=int(self.rows * self.cols)),
            MultiArrayDimension(label="cols", size=int(self.cols), stride=int(self.cols)),
        ]
        zones_msg.layout.data_offset = 0
        zones_msg.data = data
        self.zones_pub.publish(zones_msg)
        self.last_published_zones = list(data)

    def update_visited_from_positions(self):
        changed = False
        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            if pos is None:
                continue
            cell = self.world_to_cell(float(pos.point.x), float(pos.point.y))
            if cell is None:
                continue
            if cell in self.static_obstacle_cells:
                continue
            if cell not in self.visited_mask:
                self.visited_mask.add(int(cell))
                changed = True

        if not changed:
            return

        # If we already have zones, apply visited and republish
        if self.zones_current is not None:
            self.apply_visited_to_zones_inplace(self.zones_current)
            self.publish_zones(self.zones_current)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


