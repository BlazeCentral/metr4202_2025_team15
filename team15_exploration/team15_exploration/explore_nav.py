#!/usr/bin/env python3

# Copyright [2025] [METR4202 Team 15]
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
from collections import deque

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ==============================================================================
# Configuration constants (hardcoded)
# ------------------------------------------------------------------------------
# These replace external parameters/YAML so the node is self-contained. Adjust
# here to tune frontier selection and behavior without touching launch files.
# ==============================================================================
BLACKLIST_RADIUS_METERS = 0.5                 # Distance around failed goals to avoid revisiting
FRONTIER_RANKING_HEURISTIC = 'combined'       # Heuristic label for readability/logging

class ExploreNavNode(Node):
    """
    An autonomous exploration node.

    This node is responsible for finding unexplored frontiers in a map and
    commanding a robot to navigate to them using the Nav2 stack.
    """

    # ==============================================================================
    # 1.0 Class Initialization (__init__)
    # ==============================================================================
    def __init__(self):
        """Initializes the node, its state variables, parameters, and ROS interfaces."""
        super().__init__('explore_nav')

        # --- 1.1 Initialize State Variables ---
        self.map_data = None
        self.is_goal_active = False
        self.goal_blacklist = deque(maxlen=20)  # Store last 20 failed points
        self.robot_pose = None
        self.get_logger().info("✅ Initialized state variables.")

        # --- 1.2 Hardcoded configuration (no parameters/YAML required) ---
        self.blacklist_radius = BLACKLIST_RADIUS_METERS
        self.frontier_heuristic = FRONTIER_RANKING_HEURISTIC
        self.get_logger().info(f"Config: blacklist_radius={self.blacklist_radius}, heuristic='{self.frontier_heuristic}'")

        # --- 1.3 Initialize ROS2 Interfaces ---
        # Define a QoS profile for the map subscriber to ensure we get the latest map
        map_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscriber for the map
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos_profile
        )

        # Service server to trigger exploration
        self.explore_service = self.create_service(
            Trigger,
            '/get_next_waypoint',
            self._service_callback
        )

        # Action client for Nav2
        self._nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Debug publisher for the next waypoint
        self.waypoint_publisher = self.create_publisher(PoseStamped, '/next_waypoint_viz', 10)
        
        # Timer to periodically update robot's pose
        self.pose_update_timer = self.create_timer(0.5, self._update_robot_pose)
        
        self.get_logger().info("✅ ROS2 interfaces initialized. Exploration node is ready.")

    # ==============================================================================
    # 2.0 Core Logic Methods
    # ==============================================================================
    #HARVEER
   
    def _find_frontiers(self):
        """Processes the current map to find and cluster frontier cells."""
        # --- 2.1.1 Input Validation ---
        if self.map_data is None:
            self.get_logger().warn("Map data not yet available.")
            return []

        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        visited = np.zeros_like(map_array, dtype=bool)
        frontiers = []

        # --- 2.1.2 & 2.1.3 Identify and Cluster Frontier Cells using BFS ---
        # This logic is based on the interim project goal of finding unknown-free boundaries.
        for r in range(self.map_data.info.height):
            for c in range(self.map_data.info.width):
                if visited[r, c]:
                    continue
                
                # A frontier cell is an unknown cell (-1) adjacent to a known free cell (0).
                if map_array[r, c] == -1:
                    is_frontier_cell = False
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            if dr == 0 and dc == 0:
                                continue
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < self.map_data.info.height and 0 <= nc < self.map_data.info.width:
                                if map_array[nr, nc] == 0:
                                    is_frontier_cell = True
                                    break
                        if is_frontier_cell:
                            break
                    
                    if is_frontier_cell:
                        # --- 2.1.3 Start BFS to find the whole cluster ---
                        current_frontier = []
                        queue = deque([(r, c)])
                        visited[r, c] = True
                        
                        while queue:
                            row, col = queue.popleft()
                            current_frontier.append((row, col))
                            
                            for dr_bfs in [-1, 0, 1]:
                                for dc_bfs in [-1, 0, 1]:
                                    nr_bfs, nc_bfs = row + dr_bfs, col + dc_bfs
                                    if 0 <= nr_bfs < self.map_data.info.height and 0 <= nc_bfs < self.map_data.info.width and not visited[nr_bfs, nc_bfs]:
                                        # Check if neighbor is also part of a frontier region
                                        if map_array[nr_bfs, nc_bfs] == -1:
                                            visited[nr_bfs, nc_bfs] = True
                                            queue.append((nr_bfs, nc_bfs))
                        
                        # Filter out small, noisy frontiers
                        if len(current_frontier) > 5:
                            frontiers.append(current_frontier)
        
        self.get_logger().info(f"Found {len(frontiers)} frontier clusters.")
        return frontiers

    #===============================================================
    #
    #===============================================================
    def _select_best_goal(self, frontiers):
        """Ranks frontiers and selects the best one as a goal."""
        if not frontiers or self.robot_pose is None:
            return None

        ranked_goals = []
        map_info = self.map_data.info
        
        for frontier in frontiers:
            # --- 2.2.2 Process Clusters ---
            centroid_r = sum(p[0] for p in frontier) / len(frontier)
            centroid_c = sum(p[1] for p in frontier) / len(frontier)

            # Convert map coordinates to world coordinates
            goal_x = map_info.origin.position.x + centroid_c * map_info.resolution
            goal_y = map_info.origin.position.y + centroid_r * map_info.resolution

            # --- 2.2.3 Filter Blacklisted Goals ---
            is_blacklisted = False
            for bp in self.goal_blacklist:
                dist = math.hypot(goal_x - bp.x, goal_y - bp.y)
                if dist < self.blacklist_radius:
                    is_blacklisted = True
                    break
            if is_blacklisted:
                continue

            # --- 2.2.4 Rank Goals ---
            distance_from_robot = math.hypot(goal_x - self.robot_pose.x, goal_y - self.robot_pose.y)
            if distance_from_robot < 0.3: # Avoid goals that are too close
                continue
            
            size = len(frontier)
            # This heuristic is part of the project's 'search efficiency' goal.
            score = size / distance_from_robot if distance_from_robot > 0 else 0
            
            ranked_goals.append((score, goal_x, goal_y))

        if not ranked_goals:
            return None

        # --- 2.2.5 Return Best Goal ---
        ranked_goals.sort(key=lambda x: x[0], reverse=True)
        best_score, best_x, best_y = ranked_goals[0]
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = best_x
        goal_pose.pose.position.y = best_y
        goal_pose.pose.orientation.w = 1.0 # Neutral orientation

        self.get_logger().info(f"Selected best goal at ({best_x:.2f}, {best_y:.2f}) with score {best_score:.2f}")
        return goal_pose

    # ==============================================================================
    # 3.0 ROS2 Interface Callbacks
    # ==============================================================================
    def _map_callback(self, msg):
        """Stores the latest map data."""
        self.map_data = msg

    def _update_robot_pose(self):
        """Periodically looks up the robot's pose in the map frame."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_pose = transform.transform.translation #Changed this line to include .translation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform 'base_link' to 'map': {e}", throttle_duration_sec=5)

    def _service_callback(self, request, response):
        """Orchestrates the exploration cycle when the service is called."""
        if self.is_goal_active:
            response.success = False
            response.message = "A goal is already active. Please wait."
            self.get_logger().warn(response.message)
            return response
            
        if self.map_data is None:
            response.success = False
            response.message = "Map data not available yet."
            self.get_logger().error(response.message)
            return response

        frontiers = self._find_frontiers()
        goal_pose = self._select_best_goal(frontiers)

        if goal_pose:
            self._send_nav2_goal(goal_pose)
            self.waypoint_publisher.publish(goal_pose) # For debug
            response.success = True
            response.message = f"New goal sent to Nav2: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        else:
            response.success = False
            response.message = "No valid frontiers found."
            self.get_logger().info(response.message)
            # Potentially add a recovery behavior here, like rotating in place.
        
        return response

    # ==============================================================================
    # 4.0 Navigation Goal Management
    # ==============================================================================
    def _send_nav2_goal(self, pose):
        """Formats and sends a goal to the Nav2 action server."""
        self.is_goal_active = True
        self.get_logger().info("Waiting for Nav2 action server...")
        if not self._nav2_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available after 5s.")
            self.is_goal_active = False
            return
        
        # This logic is based on Prac 4's method of sending waypoints.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending goal request to Nav2...")
        send_goal_future = self._nav2_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handles the response from Nav2 after sending a goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            self.is_goal_active = False
            return
        
        self.get_logger().info("Goal accepted by Nav2. Awaiting result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """Handles the final result of the navigation action."""
        # The logic to check status is based on Prac 4's waypoint cycler behavior tree check.
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Navigation goal succeeded!")
        else:
            self.get_logger().error(f"❌ Navigation failed with status: {status}")
            if self.robot_pose: # Add the current location to blacklist on failure
                failed_point = Point()
                failed_point.x = self.robot_pose.x
                failed_point.y = self.robot_pose.y
                self.goal_blacklist.append(failed_point)
                self.get_logger().info(f"Added ({failed_point.x:.2f}, {failed_point.y:.2f}) to blacklist.")
        
        self.is_goal_active = False


# ==============================================================================
# 5.0 Main Execution Block
# ==============================================================================
def main(args=None):
    """The main function to run the node."""
    rclpy.init(args=args)
    explore_nav_node = ExploreNavNode()
    try:
        rclpy.spin(explore_nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        explore_nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()