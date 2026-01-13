import rclpy
import os
import sys
import math
import numpy as np
from rclpy.node import Node
from crazyflie_robotics_projects_msgs.srv import DarpPetition
from crazyflie_robotics_projects_msgs.msg import Trajectory2D
from vision_msgs.msg import Point2D

scripts_path = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(scripts_path))

from multiRobotPathPlanner import MultiRobotPathPlanner


class DarpNode(Node):
    def __init__(self):
        super().__init__("darp_node")

        # Parameters
        self.declare_parameter("cell_size", 2.0)
        self.cell_size = self.get_parameter("cell_size").get_parameter_value().double_value

        # Service to process DARP requests
        self.srv = self.create_service(
            DarpPetition, "darp_service", self.service_callback
        )

        self.get_logger().info("DARP node ready to receive requests.")

    def service_callback(self, request, response):
        self.get_logger().info("Request received")

        extent_x = request.max_x - request.min_x
        extent_y = request.max_y - request.min_y

        cols = int(round(extent_x / self.cell_size))
        rows = int(round(extent_y / self.cell_size))

        # Convert interface coordinates to DARP cell indices
        initial_positions, obstacles_positions = self.process_darp_input(
            request.initial_positions,
            request.obstacle_points,
            request.min_x,
            request.min_y,
            rows,
            cols,
        )
        
        # Run DARP algorithm
        darp = MultiRobotPathPlanner(
            nx=rows,
            ny=cols,
            notEqualPortions=False,
            initial_positions=initial_positions,
            portions=[],
            obs_pos=obstacles_positions,
            visualization=request.visualization,
        )

        if not darp.DARP_success:
            self.get_logger().error("DARP failed")
            return response

        # Convert DARP results to ROS 2 messages
        response.trajectories, response.zones = self.process_darp_output(
            darp,
            request.min_x,
            request.min_y,
            rows,
            cols
        )

        self.get_logger().info("Request processed successfully")
        return response

    """Convert Point2D points (world coordinates) to cell indices for DARP."""
    def process_darp_input(self, initial_positions_msg, obstacle_points_msg, min_x, min_y, rows, cols):
        # Convert initial robot positions
        initial_positions = []
        for point in initial_positions_msg:
            x = float(point.x)
            y = float(point.y)

            # Translate coordinates to the grid origin
            grid_x = int(math.floor((x - float(min_x)) / self.cell_size))
            grid_y = int(math.floor((y - float(min_y)) / self.cell_size))

            # Ensure coordinates are inside the grid
            grid_x = max(0, min(cols - 1, grid_x))
            grid_y = max(0, min(rows - 1, grid_y))

            # Convert to cell index
            cell = grid_y * cols + grid_x
            initial_positions.append(cell)

        # Convert obstacle positions
        obstacles_positions = []
        for point in obstacle_points_msg:
            x = float(point.x)
            y = float(point.y)

            # Translate coordinates to the grid origin
            grid_x = int(math.floor((x - float(min_x)) / self.cell_size))
            grid_y = int(math.floor((y - float(min_y)) / self.cell_size))

            # Ensure coordinates are inside the grid
            grid_x = max(0, min(cols - 1, grid_x))
            grid_y = max(0, min(rows - 1, grid_y))

            cell = grid_y * cols + grid_x
            obstacles_positions.append(cell)

        return initial_positions, obstacles_positions

    """Convert DARP results (trajectories and zones) to ROS 2 messages."""
    def process_darp_output(self, planner, min_x, min_y, rows, cols):
        # Paths come in sub-cells
        subcell_size = self.cell_size / 2.0

        trajectories = []

        # Process trajectories for each robot
        for robot_id in range(planner.darp_instance.droneNo):
            path = planner.best_case.paths[robot_id]

            traj = Trajectory2D()

            if len(path) > 0:
                # Initial point
                first_move = path[0]
                p = Point2D()
                # (row, col) sub-cell -> (x,y) meters (sub-cell center)
                p.x = float(float(min_x) + (float(first_move[1]) + 0.5) * subcell_size)
                p.y = float(float(min_y) + (float(first_move[0]) + 0.5) * subcell_size)
                traj.points.append(p)

                # Next points
                for move in path:
                    p = Point2D()
                    p.x = float(float(min_x) + (float(move[3]) + 0.5) * subcell_size)
                    p.y = float(float(min_y) + (float(move[2]) + 0.5) * subcell_size)
                    traj.points.append(p)

            trajectories.append(traj)

        # Process zones matrix
        assignment_matrix = planner.darp_instance.A
        zones_matrix = np.zeros((rows, cols), dtype=np.int32)

        for i in range(rows):
            for j in range(cols):
                cell_value = int(assignment_matrix[i, j])
                # If obstacle, assign -1; otherwise assign UAV id + 1
                if cell_value == planner.darp_instance.droneNo:
                    zones_matrix[i, j] = -1
                else:
                    zones_matrix[i, j] = cell_value + 1

        # Flatten matrix for the message
        zones = zones_matrix.flatten().tolist()

        return trajectories, zones


def main(args=None):
    rclpy.init(args=args)
    node = DarpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
