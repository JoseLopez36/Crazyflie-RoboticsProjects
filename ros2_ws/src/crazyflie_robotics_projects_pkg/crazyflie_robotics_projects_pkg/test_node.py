import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped
from crazyflie_interfaces.srv import Takeoff, Arm
from builtin_interfaces.msg import Duration

@dataclass
class AgentIO:
    position: Optional[PointStamped] = None
    armed_sent: bool = False
    disarm_sent: bool = False
    takeoff_sent: bool = False

class TestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_node")

        # Parameters
        self.declare_parameter("agents.ids", [""])
        self.declare_parameter("target_altitude", 1.0)
        self.declare_parameter("acceptance_radius", 0.15)
        self.declare_parameter("land_height", 0.03)
        self.declare_parameter("takeoff_duration", 2.0)
        self.declare_parameter("preflight_setpoints", 15)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("takeoff_tolerance", 0.25)
        self.declare_parameter("landing_tolerance", 0.05)
        self.declare_parameter("takeoff_timeout_sec", 10.0)
        self.declare_parameter("goto_timeout_sec", 30.0)
        self.declare_parameter("landing_timeout_sec", 15.0)
        self.declare_parameter("targets.x", [0.0])
        self.declare_parameter("targets.y", [0.0])

        self.agent_ids: List[str] = list(self.get_parameter("agents.ids").get_parameter_value().string_array_value)
        self.target_altitude = self.get_parameter("target_altitude").get_parameter_value().double_value
        self.acceptance_radius = self.get_parameter("acceptance_radius").get_parameter_value().double_value
        self.land_height = self.get_parameter("land_height").get_parameter_value().double_value
        self.takeoff_duration = self.get_parameter("takeoff_duration").get_parameter_value().double_value
        self.preflight_setpoints = self.get_parameter("preflight_setpoints").get_parameter_value().integer_value
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.takeoff_tolerance = self.get_parameter("takeoff_tolerance").get_parameter_value().double_value
        self.landing_tolerance = self.get_parameter("landing_tolerance").get_parameter_value().double_value
        self.takeoff_timeout_sec = self.get_parameter("takeoff_timeout_sec").get_parameter_value().double_value
        self.goto_timeout_sec = self.get_parameter("goto_timeout_sec").get_parameter_value().double_value
        self.landing_timeout_sec = self.get_parameter("landing_timeout_sec").get_parameter_value().double_value
        targets_x = list(self.get_parameter("targets.x").get_parameter_value().double_array_value)
        targets_y = list(self.get_parameter("targets.y").get_parameter_value().double_array_value)

        # Map agent_id -> (x, y)
        self.targets: Dict[str, Tuple[float, float]] = {}
        for i, agent_id in enumerate(self.agent_ids):
            tx = float(targets_x[i]) if i < len(targets_x) else 0.0
            ty = float(targets_y[i]) if i < len(targets_y) else 0.0
            self.targets[agent_id] = (tx, ty)

        # QoS like the rest of the stack
        qos_profile_points = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.io: Dict[str, AgentIO] = {aid: AgentIO() for aid in self.agent_ids}
        self.pos_subs = {}
        self.setpoint_pubs = {}
        self.takeoff_clients = {}
        self.arm_clients = {}

        for agent_id in self.agent_ids:
            self.pos_subs[agent_id] = self.create_subscription(
                PointStamped,
                f"/{agent_id}/state/position",
                lambda msg, uid=agent_id: self.position_cb(msg, uid),
                qos_profile_points,
            )
            self.setpoint_pubs[agent_id] = self.create_publisher(
                PointStamped,
                f"/{agent_id}/control/setpoint",
                qos_profile_points,
            )
            self.takeoff_clients[agent_id] = self.create_client(Takeoff, f"/{agent_id}/takeoff")
            self.arm_clients[agent_id] = self.create_client(Arm, f"/{agent_id}/arm")

        self.stage = "WAIT_POS"  # WAIT_POS -> PREFLIGHT -> TAKEOFF -> GOTO -> LANDING -> DONE
        self.stage_start = self.get_clock().now()
        self.preflight_ticks = 0

        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(
            f"test_node started for agents={self.agent_ids} targets={self.targets}"
        )

    def position_cb(self, msg: PointStamped, agent_id: str) -> None:
        if agent_id not in self.io:
            return
        self.io[agent_id].position = msg

    def all_positions_received(self) -> bool:
        return all(self.io[aid].position is not None for aid in self.agent_ids)

    def publish_setpoint(self, agent_id: str, x: float, y: float, z: float) -> None:
        msg = PointStamped()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.setpoint_pubs[agent_id].publish(msg)

    def send_arm(self, agent_id: str, arm: bool) -> None:
        client = self.arm_clients[agent_id]
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{agent_id}: arm service not available")
            return
        req = Arm.Request()
        req.arm = bool(arm)
        client.call_async(req)

    def send_takeoff(self, agent_id: str, height: float, duration_sec: float) -> None:
        client = self.takeoff_clients[agent_id]
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{agent_id}: takeoff service not available")
            return
        req = Takeoff.Request()
        req.height = float(height)
        req.duration = Duration()
        req.duration.sec = int(duration_sec)
        req.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        req.group_mask = 0
        client.call_async(req)

    def elapsed_stage_sec(self) -> float:
        return (self.get_clock().now() - self.stage_start).nanoseconds * 1e-9

    def transition(self, new_stage: str) -> None:
        self.stage = new_stage
        self.stage_start = self.get_clock().now()
        self.get_logger().info(f"test_node stage -> {new_stage}")

    def loop(self) -> None:
        if not self.agent_ids:
            # Nothing to do (misconfigured agents list)
            return

        if self.stage == "WAIT_POS":
            if self.all_positions_received():
                self.transition("PREFLIGHT")
            return

        # We rely on positions for all stages below
        if not self.all_positions_received():
            return

        # --- PREFLIGHT: stream a few setpoints before takeoff ---
        if self.stage == "PREFLIGHT":
            for agent_id in self.agent_ids:
                pos = self.io[agent_id].position
                if pos is None:
                    continue
                self.publish_setpoint(
                    agent_id, pos.point.x, pos.point.y, self.target_altitude
                )

            self.preflight_ticks += 1
            if self.preflight_ticks >= max(1, int(self.preflight_setpoints)):
                for agent_id in self.agent_ids:
                    if not self.io[agent_id].armed_sent:
                        self.send_arm(agent_id, True)
                        self.io[agent_id].armed_sent = True
                    if not self.io[agent_id].takeoff_sent:
                        self.send_takeoff(agent_id, self.target_altitude, self.takeoff_duration)
                        self.io[agent_id].takeoff_sent = True
                self.transition("TAKEOFF")
            return

        # --- TAKEOFF: keep hovering until everyone is up ---
        if self.stage == "TAKEOFF":
            for agent_id in self.agent_ids:
                pos = self.io[agent_id].position
                if pos is None:
                    continue
                self.publish_setpoint(
                    agent_id, pos.point.x, pos.point.y, self.target_altitude
                )

            all_up = True
            for agent_id in self.agent_ids:
                z = float(self.io[agent_id].position.point.z)  # type: ignore[union-attr]
                if z < (self.target_altitude - self.takeoff_tolerance):
                    all_up = False
                    break

            if all_up:
                self.transition("GOTO")
                return

            if self.elapsed_stage_sec() > self.takeoff_timeout_sec:
                self.get_logger().error("Takeoff timeout; proceeding to LANDING")
                self.transition("LANDING")
            return

        # --- GOTO: command each agent to its target XY ---
        if self.stage == "GOTO":
            all_reached = True
            for agent_id in self.agent_ids:
                tx, ty = self.targets.get(agent_id, (0.0, 0.0))
                self.publish_setpoint(agent_id, tx, ty, self.target_altitude)

                pos = self.io[agent_id].position
                if pos is None:
                    all_reached = False
                    continue
                dx = float(tx) - float(pos.point.x)
                dy = float(ty) - float(pos.point.y)
                d = math.sqrt(dx * dx + dy * dy)
                if d > self.acceptance_radius:
                    all_reached = False

            if all_reached:
                self.transition("LANDING")
                return

            if self.elapsed_stage_sec() > self.goto_timeout_sec:
                self.get_logger().error("GOTO timeout; proceeding to LANDING")
                self.transition("LANDING")
            return

        # --- LANDING: command low Z; disarm once close to ground ---
        if self.stage == "LANDING":
            all_low = True
            for agent_id in self.agent_ids:
                pos = self.io[agent_id].position
                if pos is None:
                    all_low = False
                    continue
                self.publish_setpoint(agent_id, pos.point.x, pos.point.y, self.land_height)

                z = float(pos.point.z)
                if z > (self.land_height + self.landing_tolerance):
                    all_low = False

            if all_low:
                for agent_id in self.agent_ids:
                    if not self.io[agent_id].disarm_sent:
                        self.send_arm(agent_id, False)
                        self.io[agent_id].disarm_sent = True
                self.transition("DONE")
                return

            if self.elapsed_stage_sec() > self.landing_timeout_sec:
                self.get_logger().error("Landing timeout; sending DISARM anyway")
                for agent_id in self.agent_ids:
                    if not self.io[agent_id].disarm_sent:
                        self.send_arm(agent_id, False)
                        self.io[agent_id].disarm_sent = True
                self.transition("DONE")
            return

        # --- DONE ---
        if self.stage == "DONE":
            return

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

