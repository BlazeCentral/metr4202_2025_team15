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
from nav2_msgs.msg import BehaviorTreeLog
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
BLACKLIST_RADIUS_METERS = 0.2             # Distance around failed goals to avoid revisiting
FRONTIER_RANKING_HEURISTIC = 'combined'       # Heuristic label for readability/logging

# Linear scoring coefficients: score = A*cluster_size + B*distance + C*obstacle_count
SCORING_CLUSTER_SIZE_WEIGHT = 1.0        # A: Weight for frontier cluster size
SCORING_DISTANCE_WEIGHT = -0.5           # B: Weight for distance (negative = prefer closer)
SCORING_OBSTACLE_WEIGHT = -2.0           # C: Weight for nearby obstacles (negative = avoid obstacles)
OBSTACLE_DETECTION_RADIUS = 1.0          # Radius to check for obstacles around goal
MIN_WALL_DISTANCE = 0.4                  # Minimum distance from walls (meters)

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
        self.goal_blacklist = deque(maxlen=5)  # Store last 20 failed points
        self.robot_pose = None
        self.get_logger().info("Initialized state variables.")

        # self.subscription = self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10)

        # --- 1.2 Hardcoded configuration (no parameters/YAML required) ---
        self.blacklist_radius = BLACKLIST_RADIUS_METERS
        self.min_goal_distance = 0.5  # Minimum distance from robot for goal selection
        self.min_wall_distance = MIN_WALL_DISTANCE  # Minimum distance from walls
        self.frontier_heuristic = FRONTIER_RANKING_HEURISTIC
        self.get_logger().info(f"Config: blacklist_radius={self.blacklist_radius}, min_goal_distance={self.min_goal_distance}, min_wall_distance={self.min_wall_distance}, heuristic='{self.frontier_heuristic}'")

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
        # Timer to continuously check for new goals
        self.exploration_timer = self.create_timer(1.0, self.goal_cycler)
        # Timer to monitor path cost during navigation
        self.path_monitor_timer = self.create_timer(2.0, self._monitor_path_cost)

        
        self.get_logger().info("ROS2 interfaces initialized. Exploration node is ready.")

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
                        if len(current_frontier) > 8:
                            frontiers.append(current_frontier)
        
        self.get_logger().info(f"Found {len(frontiers)} frontier clusters.")
        return frontiers

    def _count_nearby_obstacles(self, goal_x, goal_y):
        """Count obstacles within a radius of the goal position."""
        if self.map_data is None:
            return 0
        
        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
        
        # Convert goal position to map coordinates
        goal_c = int((goal_x - map_info.origin.position.x) / map_info.resolution)
        goal_r = int((goal_y - map_info.origin.position.y) / map_info.resolution)
        
        # Calculate search radius in cells
        search_radius_cells = int(OBSTACLE_DETECTION_RADIUS / map_info.resolution)
        
        obstacle_count = 0
        
        # Check all cells within the search radius
        for dr in range(-search_radius_cells, search_radius_cells + 1):
            for dc in range(-search_radius_cells, search_radius_cells + 1):
                # Calculate distance from goal
                distance = math.sqrt(dr*dr + dc*dc) * map_info.resolution
                if distance <= OBSTACLE_DETECTION_RADIUS:
                    # Check if cell is within map bounds
                    check_r = goal_r + dr
                    check_c = goal_c + dc
                    if 0 <= check_r < map_info.height and 0 <= check_c < map_info.width:
                        # Count obstacles (value > 50 in occupancy grid)
                        if map_array[check_r, check_c] > 50:
                            obstacle_count += 1
        
        return obstacle_count

    def _check_wall_distance(self, goal_x, goal_y):
        """Check if the goal position maintains minimum distance from walls."""
        if self.map_data is None:
            return True  # If no map data, assume it's safe
        
        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
        
        # Convert goal position to map coordinates
        goal_c = int((goal_x - map_info.origin.position.x) / map_info.resolution)
        goal_r = int((goal_y - map_info.origin.position.y) / map_info.resolution)
        
        # Calculate search radius in cells
        search_radius_cells = int(self.min_wall_distance / map_info.resolution)
        
        # Check all cells within the minimum wall distance
        for dr in range(-search_radius_cells, search_radius_cells + 1):
            for dc in range(-search_radius_cells, search_radius_cells + 1):
                # Calculate distance from goal
                distance = math.sqrt(dr*dr + dc*dc) * map_info.resolution
                if distance <= self.min_wall_distance:
                    # Check if cell is within map bounds
                    check_r = goal_r + dr
                    check_c = goal_c + dc
                    if 0 <= check_r < map_info.height and 0 <= check_c < map_info.width:
                        # Check if there's a wall/obstacle within minimum distance
                        if map_array[check_r, check_c] > 50:  # Obstacle threshold
                            return False
        
        return True

    def _validate_path_cost(self, start_x, start_y, goal_x, goal_y, max_cost_threshold=30):
        """Validate that the path from start to goal doesn't go through high-cost regions."""
        if self.map_data is None:
            return True  # If no map data, assume path is valid
        
        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
        
        # Convert positions to map coordinates
        start_c = int((start_x - map_info.origin.position.x) / map_info.resolution)
        start_r = int((start_y - map_info.origin.position.y) / map_info.resolution)
        goal_c = int((goal_x - map_info.origin.position.x) / map_info.resolution)
        goal_r = int((goal_y - map_info.origin.position.y) / map_info.resolution)
        
        # Simple line-of-sight path validation
        # Sample points along the straight-line path
        distance = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        num_samples = max(10, int(distance / map_info.resolution))
        
        for i in range(num_samples + 1):
            # Interpolate along the path
            t = i / num_samples
            sample_x = start_x + t * (goal_x - start_x)
            sample_y = start_y + t * (goal_y - start_y)
            
            # Convert to map coordinates
            sample_c = int((sample_x - map_info.origin.position.x) / map_info.resolution)
            sample_r = int((sample_y - map_info.origin.position.y) / map_info.resolution)
            
            # Check if sample point is within map bounds
            if 0 <= sample_r < map_info.height and 0 <= sample_c < map_info.width:
                cell_value = map_array[sample_r, sample_c]
                
                # Check if path goes through high-cost regions
                if cell_value > max_cost_threshold:
                    self.get_logger().debug(f"Path validation failed: high cost cell at ({sample_x:.2f}, {sample_y:.2f}) with value {cell_value}")
                    return False
        
        return True

    def _find_alternative_path(self, start_x, start_y, goal_x, goal_y):
        """Find alternative paths when direct path has high cost."""
        if self.map_data is None:
            return None
        
        map_info = self.map_data.info
        
        # Try different waypoints to avoid obstacles
        # Strategy: Try points perpendicular to the direct line
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.1:  # Too close, no alternative needed
            return None
        
        # Normalize direction vector
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        # Perpendicular vectors (left and right)
        perp_left_x = -dy_norm
        perp_left_y = dx_norm
        perp_right_x = dy_norm
        perp_right_y = -dx_norm
        
        # Try alternative paths with different offsets
        offsets = [0.5, 1.0, 1.5, 2.0]  # meters
        
        for offset in offsets:
            # Try left side
            alt_x = start_x + offset * perp_left_x
            alt_y = start_y + offset * perp_left_y
            
            if self._validate_path_cost(start_x, start_y, alt_x, alt_y) and \
               self._validate_path_cost(alt_x, alt_y, goal_x, goal_y):
                self.get_logger().info(f"Found alternative path via ({alt_x:.2f}, {alt_y:.2f})")
                return (alt_x, alt_y)
            
            # Try right side
            alt_x = start_x + offset * perp_right_x
            alt_y = start_y + offset * perp_right_y
            
            if self._validate_path_cost(start_x, start_y, alt_x, alt_y) and \
               self._validate_path_cost(alt_x, alt_y, goal_x, goal_y):
                self.get_logger().info(f"Found alternative path via ({alt_x:.2f}, {alt_y:.2f})")
                return (alt_x, alt_y)
        
        return None

    def _monitor_path_cost(self):
        """Monitor path cost during navigation and replan if necessary."""
        if not self.is_goal_active or self.robot_pose is None:
            return
        
        # Get current goal from Nav2 (this would need to be stored when goal is sent)
        # For now, we'll use a simple approach: check if robot is stuck
        # In a full implementation, you'd store the current goal and check path to it
        
        # Check if robot has been at the same position for too long
        # This is a simple heuristic - in practice you'd want more sophisticated monitoring
        current_time = self.get_clock().now()
        
        # If we had stored the current goal, we could check:
        # if not self._validate_path_cost(self.robot_pose.x, self.robot_pose.y, current_goal_x, current_goal_y):
        #     self.get_logger().warn("Path to current goal has become high-cost, replanning...")
        #     # Cancel current goal and select new one
        #     self._cancel_current_goal()
        #     self.goal_cycler()
        
        pass  # Placeholder for now

    def _cancel_current_goal(self):
        """Cancel the current navigation goal."""
        if self.is_goal_active:
            self.get_logger().info("Canceling current goal due to path cost issues")
            # In a full implementation, you'd cancel the Nav2 goal here
            self.is_goal_active = False

    #===============================================================
    #
    #===============================================================
    def _select_best_goal(self, frontiers):
        """Ranks frontiers and selects the best one as a goal."""
        if not frontiers or self.robot_pose is None:
            return None
        
        # Log robot position for debugging
        self.get_logger().info(f"Robot position: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}), min_goal_distance: {self.min_goal_distance:.2f}m")

        ranked_goals = []
        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
        
        for frontier in frontiers:
            best_goal_candidate = None
            min_dist_to_robot = float('inf')

            # --- 2.2.2 Process Clusters and find the best free-space goal ---
            # Iterate through all frontier cells (r_f, c_f) in the cluster
            for r_f, c_f in frontier:
                # Check all 8 neighbors of the frontier cell (r_f, c_f)
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        
                        r_free, c_free = r_f + dr, c_f + dc
                        
                        # Check map bounds
                        if 0 <= r_free < map_info.height and 0 <= c_free < map_info.width:
                            
                            # Candidate must be a known free cell (0) to be a valid goal
                            if map_array[r_free, c_free] == 0:
                                # Convert to world coordinates (using center of cell)
                                candidate_x = map_info.origin.position.x + (c_free + 0.5) * map_info.resolution
                                candidate_y = map_info.origin.position.y + (r_free + 0.5) * map_info.resolution
                                
                                # --- 2.2.3 Filter Blacklisted Goals ---
                                is_blacklisted = False
                                for bp in self.goal_blacklist:
                                    dist = math.hypot(candidate_x - bp.x, candidate_y - bp.y)
                                    if dist < self.blacklist_radius:
                                        is_blacklisted = True
                                        break
                                if is_blacklisted:
                                    continue

                                # --- 2.2.4 Rank Goals ---
                                distance_from_robot = math.hypot(candidate_x - self.robot_pose.x, candidate_y - self.robot_pose.y)
                                
                                # Debug logging for distance calculations
                                self.get_logger().debug(f"Goal candidate at ({candidate_x:.2f}, {candidate_y:.2f}), robot at ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}), distance: {distance_from_robot:.2f}m, min_distance: {self.min_goal_distance:.2f}m")
                                
                                # Reject goals that are too close to the robot
                                if distance_from_robot < self.min_goal_distance:
                                    self.get_logger().debug(f"Rejecting goal at ({candidate_x:.2f}, {candidate_y:.2f}) - too close ({distance_from_robot:.2f}m < {self.min_goal_distance:.2f}m)")
                                    continue
                                
                                # Check minimum distance from walls
                                if not self._check_wall_distance(candidate_x, candidate_y):
                                    self.get_logger().debug(f"Rejecting goal at ({candidate_x:.2f}, {candidate_y:.2f}) - too close to walls (min distance: {self.min_wall_distance}m)")
                                    continue
                                
                                # Select the goal candidate on the free-space side with the best score 
                                # (closest to robot, but not too close, maximizing coverage/size)
                                size = len(frontier)
                                
                                # Count nearby obstacles
                                obstacle_count = self._count_nearby_obstacles(candidate_x, candidate_y)
                                
                                # Linear scoring: A*cluster_size + B*distance + C*obstacle_count
                                score = (SCORING_CLUSTER_SIZE_WEIGHT * size + 
                                        SCORING_DISTANCE_WEIGHT * distance_from_robot + 
                                        SCORING_OBSTACLE_WEIGHT * obstacle_count)
                                
                                # Debug logging for scoring components
                                self.get_logger().debug(f"Goal scoring: size={size}, distance={distance_from_robot:.2f}, obstacles={obstacle_count}, score={score:.2f}")
                                
                                # Validate path cost before considering this goal
                                if not self._validate_path_cost(self.robot_pose.x, self.robot_pose.y, candidate_x, candidate_y):
                                    self.get_logger().debug(f"Goal at ({candidate_x:.2f}, {candidate_y:.2f}) rejected due to high path cost")
                                    
                                    # Try to find alternative path
                                    alt_waypoint = self._find_alternative_path(self.robot_pose.x, self.robot_pose.y, candidate_x, candidate_y)
                                    if alt_waypoint:
                                        alt_x, alt_y = alt_waypoint
                                        # Recalculate score for alternative waypoint
                                        alt_distance = math.hypot(alt_x - self.robot_pose.x, alt_y - self.robot_pose.y)
                                        alt_obstacles = self._count_nearby_obstacles(alt_x, alt_y)
                                        alt_score = (SCORING_CLUSTER_SIZE_WEIGHT * size + 
                                                   SCORING_DISTANCE_WEIGHT * alt_distance + 
                                                   SCORING_OBSTACLE_WEIGHT * alt_obstacles)
                                        
                                        self.get_logger().info(f"Using alternative waypoint at ({alt_x:.2f}, {alt_y:.2f}) with score {alt_score:.2f}")
                                        
                                        # Track the alternative goal
                                        if not best_goal_candidate or alt_score > best_goal_candidate[0]:
                                            best_goal_candidate = (alt_score, alt_x, alt_y)
                                    continue
                                
                                # Track the best scoring goal for this frontier
                                if not best_goal_candidate or score > best_goal_candidate[0]:
                                    best_goal_candidate = (score, candidate_x, candidate_y)
            
            # If a valid goal was found for this frontier cluster, add it to the list
            if best_goal_candidate:
                ranked_goals.append(best_goal_candidate)


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
        
        # Orient the robot toward the frontier by calculating the angle from the robot to the goal
        angle = math.atan2(best_y - self.robot_pose.y, best_x - self.robot_pose.x)
        
        # Convert yaw angle to quaternion (qx, qy, qz, qw)
        goal_pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_pose.pose.orientation.w = math.cos(angle / 2.0)

        self.get_logger().info(f"Selected best goal at ({best_x:.2f}, {best_y:.2f}) with score {best_score:.2f}")
        return goal_pose
    
    def goal_cycler(self):
        if self.is_goal_active:
            return
        if self.map_data is None:
            return
        frontier=self._find_frontiers()
        goal_pose=self._select_best_goal(frontier)

        if goal_pose:
            self._send_nav2_goal(goal_pose)
            self.waypoint_publisher.publish(goal_pose)
        else:
            self.get_logger().info("All Frontiers explore.")


    # ==============================================================================
    # 3.0 ROS2 Interface Callbacks
    # ==============================================================================
    def _map_callback(self, msg):
        """Stores the latest map data."""
        self.map_data = msg

    def _update_robot_pose(self):
        """Periodically looks up the robot's pose in the map frame."""
        try:
            # Change 'base_link' to 'base_footprint' for consistency with SLAM/Nav2 configs
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_pose = transform.transform.translation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform 'base_footprint' to 'map': {e}", throttle_duration_sec=5)


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
            self.get_logger().info(" Navigation goal succeeded!")
        else:
            self.get_logger().error(f" Navigation failed with status: {status}")
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