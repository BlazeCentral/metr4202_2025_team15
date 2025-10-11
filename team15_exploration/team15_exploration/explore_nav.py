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
from nav2_msgs.msg import BehaviorTreeStatusChange


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
        self.blacklist_timers = {}  # Track when goals were blacklisted
        self.robot_pose = None
        self.last_robot_pose = None
        self.stationary_start_time = None
        self.last_goal_send_time = None
        self.recovery_mode = False
        self.recovery_start_time = None
        self.get_logger().info("Initialized state variables.")

        # Behavior tree logging subscriber
        self.bt_log_subscription = self.create_subscription(
            BehaviorTreeLog, 
            '/behavior_tree_log', 
            self.bt_log_callback, 
            10
        )
        
        # Alternative: Behavior tree status change subscriber (more reliable)
        self.bt_status_subscription = self.create_subscription(
            BehaviorTreeStatusChange,
            '/behavior_tree_status_change',
            self.bt_status_callback,
            10
        )

        # --- 1.2 Hardcoded configuration (no parameters/YAML required) ---
        self.blacklist_radius = BLACKLIST_RADIUS_METERS
        self.min_goal_distance = 1.25 # Minimum distance from robot for goal selection
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
        # Timer to manage blacklist timers and recovery mode
        self.blacklist_manager_timer = self.create_timer(2.0, self._manage_blacklist_timers)
        # Timer to monitor path cost during navigation
        self.path_monitor_timer = self.create_timer(2.0, self._monitor_path_cost)
        # Timer to show navigation status during active goals
        self.nav_status_timer = self.create_timer(3.0, self._log_navigation_status)
        # Timer to check if behavior tree topic is available
        self.bt_topic_checker = self.create_timer(5.0, self._check_bt_topic)
        # Timer to detect if robot is stuck/stationary
        self.stuck_detector_timer = self.create_timer(2.0, self._detect_stuck_robot)
        # Timer to check for goal timeouts
        self.goal_timeout_timer = self.create_timer(5.0, self._check_goal_timeout)

        
        self.get_logger().info("ROS2 interfaces initialized. Exploration node is ready.")
        self.get_logger().info("📊 Behavior Tree logging enabled - you'll see Nav2 behavior details in the terminal")

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

    def _check_unknown_goal_safety(self, goal_x, goal_y):
        """Check if unknown space goal is safe (maintains wall distance, path validation)."""
        if self.map_data is None:
            return True  # If no map data, assume it's safe
        
        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
        
        # Convert goal position to map coordinates
        goal_c = int((goal_x - map_info.origin.position.x) / map_info.resolution)
        goal_r = int((goal_y - map_info.origin.position.y) / map_info.resolution)
        
        # Check if goal is within map bounds
        if not (0 <= goal_r < map_info.height and 0 <= goal_c < map_info.width):
            return False
        
        # Check if goal is in unknown space (-1)
        if map_array[goal_r, goal_c] != -1:
            return False
        
        # Check wall distance (same as free space goals)
        if not self._check_wall_distance(goal_x, goal_y):
            return False
        
        # Check path validation from robot to goal
        if self.robot_pose:
            if not self._validate_path_cost(self.robot_pose.x, self.robot_pose.y, goal_x, goal_y):
                return False
        
        return True

    def _find_unknown_space_goals(self, frontiers):
        """Find goals in unknown space when free space goals are unavailable."""
        if not frontiers or self.robot_pose is None:
            return []
        
        ranked_goals = []
        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
        
        for frontier in frontiers:
            best_goal_candidate = None
            
            # Look for unknown space cells adjacent to frontier
            for r_f, c_f in frontier:
                # Check all 8 neighbors of the frontier cell
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        
                        r_unknown, c_unknown = r_f + dr, c_f + dc
                        
                        # Check map bounds
                        if 0 <= r_unknown < map_info.height and 0 <= c_unknown < map_info.width:
                            # Must be unknown space (-1)
                            if map_array[r_unknown, c_unknown] == -1:
                                # Convert to world coordinates
                                candidate_x = map_info.origin.position.x + (c_unknown + 0.5) * map_info.resolution
                                candidate_y = map_info.origin.position.y + (r_unknown + 0.5) * map_info.resolution
                                
                                # Check blacklist
                                is_blacklisted = False
                                goal_key = (candidate_x, candidate_y)
                                
                                for bp in self.goal_blacklist:
                                    dist = math.hypot(candidate_x - bp.x, candidate_y - bp.y)
                                    if dist < self.blacklist_radius:
                                        is_blacklisted = True
                                        break
                                
                                if not is_blacklisted and goal_key in self.blacklist_timers:
                                    current_time = self.get_clock().now()
                                    time_since_blacklist = (current_time - self.blacklist_timers[goal_key]).nanoseconds / 1e9
                                    if time_since_blacklist < 30:
                                        is_blacklisted = True
                                
                                if is_blacklisted:
                                    continue
                                
                                # Check distance constraints
                                distance_from_robot = math.hypot(candidate_x - self.robot_pose.x, candidate_y - self.robot_pose.y)
                                min_distance = 0.4 if self.recovery_mode else self.min_goal_distance
                                
                                if distance_from_robot < min_distance:
                                    continue
                                
                                # Check safety constraints
                                if not self._check_unknown_goal_safety(candidate_x, candidate_y):
                                    continue
                                
                                # Score the unknown space goal
                                size = len(frontier)
                                obstacle_count = self._count_nearby_obstacles(candidate_x, candidate_y)
                                
                                # Boost score for unknown space goals (exploration priority)
                                exploration_boost = 2.0  # Higher priority for unknown space
                                distance_boost = distance_from_robot * 0.5 if self.recovery_mode else 0
                                
                                score = (SCORING_CLUSTER_SIZE_WEIGHT * size + 
                                        SCORING_DISTANCE_WEIGHT * distance_from_robot + 
                                        SCORING_OBSTACLE_WEIGHT * obstacle_count + 
                                        exploration_boost + distance_boost)
                                
                                # Track best goal for this frontier
                                if not best_goal_candidate or score > best_goal_candidate[0]:
                                    best_goal_candidate = (score, candidate_x, candidate_y)
            
            # Add best goal for this frontier
            if best_goal_candidate:
                ranked_goals.append(best_goal_candidate)
        
        return ranked_goals

    def _manage_blacklist_timers(self):
        """Manage blacklist timers and recovery mode."""
        current_time = self.get_clock().now()
        
        # Check for expired blacklist entries (30 seconds)
        expired_entries = []
        for goal_key, blacklist_time in self.blacklist_timers.items():
            time_since_blacklist = (current_time - blacklist_time).nanoseconds / 1e9
            if time_since_blacklist > 30:  # 30 seconds
                expired_entries.append(goal_key)
        
        # Remove expired entries
        for goal_key in expired_entries:
            del self.blacklist_timers[goal_key]
            # Remove from blacklist deque
            for i, point in enumerate(self.goal_blacklist):
                if abs(point.x - goal_key[0]) < 0.1 and abs(point.y - goal_key[1]) < 0.1:
                    self.goal_blacklist.remove(point)
                    break
        
        if expired_entries:
            self.get_logger().info(f"🕒 Removed {len(expired_entries)} expired blacklist entries")
        
        # Check recovery mode timeout (30-40 seconds)
        if self.recovery_mode and self.recovery_start_time:
            recovery_duration = (current_time - self.recovery_start_time).nanoseconds / 1e9
            if recovery_duration > 40:  # 40 seconds max recovery time
                self.recovery_mode = False
                self.recovery_start_time = None
                self.get_logger().info("🔄 Recovery mode ended - returning to normal exploration")

    def _trigger_recovery_mode(self):
        """Trigger recovery mode to help robot escape stuck situations."""
        if not self.recovery_mode:
            self.recovery_mode = True
            self.recovery_start_time = self.get_clock().now()
            self.get_logger().warn("🚨 RECOVERY MODE ACTIVATED!")
            self.get_logger().warn("🔧 Allowing goals as close as 0.4m for 30-40 seconds")
            self.get_logger().warn("🎯 Robot will be pushed to move away from current position")

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

    def _log_navigation_status(self):
        """Log current navigation status when goal is active."""
        if not self.is_goal_active or self.robot_pose is None:
            return
            
        # Log robot position and status
        self.get_logger().info(f"🤖 Robot at: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}) - Navigation in progress...")
        self.get_logger().info("🌳 Nav2 Behavior Tree is actively planning and executing path")

    def _check_bt_topic(self):
        """Check if behavior tree topic is available and being published."""
        try:
            # Check if the topic exists
            topic_info = self.get_topic_names_and_types()
            bt_log_found = False
            bt_status_found = False
            
            for topic_name, topic_types in topic_info:
                if '/behavior_tree_log' in topic_name:
                    bt_log_found = True
                    self.get_logger().info(f"📊 Found BT log topic: {topic_name} with types: {topic_types}")
                elif '/behavior_tree_status_change' in topic_name:
                    bt_status_found = True
                    self.get_logger().info(f"📊 Found BT status topic: {topic_name} with types: {topic_types}")
            
            if not bt_log_found and not bt_status_found:
                self.get_logger().warn("⚠️ No behavior tree topics found!")
                self.get_logger().warn("💡 Make sure Nav2 is running with behavior tree logging enabled")
                self.get_logger().warn("💡 Check your Nav2 launch file for 'enable_bt_logging' parameter")
                self.get_logger().warn("💡 Try: ros2 launch nav2_bringup navigation_launch.py enable_bt_logging:=True")
            elif bt_log_found or bt_status_found:
                self.get_logger().info("✅ Behavior tree logging topics found - logging should work!")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error checking BT topic: {e}")

    def _detect_stuck_robot(self):
        """Detect if robot is stuck/stationary and provide diagnostic information."""
        if not self.is_goal_active or self.robot_pose is None:
            return
            
        current_time = self.get_clock().now()
        current_pose = (self.robot_pose.x, self.robot_pose.y)
        
        # Check if robot has moved
        if self.last_robot_pose is not None:
            distance_moved = math.sqrt(
                (current_pose[0] - self.last_robot_pose[0])**2 + 
                (current_pose[1] - self.last_robot_pose[1])**2
            )
            
            # If robot hasn't moved much (less than 0.1m)
            if distance_moved < 0.1:
                if self.stationary_start_time is None:
                    self.stationary_start_time = current_time
                    self.get_logger().warn("⚠️ Robot appears to be stationary - starting stuck detection timer")
                else:
                    # Calculate how long robot has been stationary
                    stationary_duration = (current_time - self.stationary_start_time).nanoseconds / 1e9
                    
                    if stationary_duration > 10:  # 10 seconds
                        self.get_logger().error(f"🚨 ROBOT STUCK DETECTED! Stationary for {stationary_duration:.1f} seconds")
                        self.get_logger().error(f"📍 Current position: ({current_pose[0]:.2f}, {current_pose[1]:.2f})")
                        
                        # Provide diagnostic information
                        self._diagnose_stuck_robot(stationary_duration)
                        
                    elif stationary_duration > 5:  # 5 seconds
                        self.get_logger().warn(f"⚠️ Robot stationary for {stationary_duration:.1f}s - monitoring...")
            else:
                # Robot is moving, reset stationary timer
                if self.stationary_start_time is not None:
                    self.get_logger().info("✅ Robot movement detected - resetting stuck detection")
                    self.stationary_start_time = None
        
        # Update last known position
        self.last_robot_pose = current_pose

    def _diagnose_stuck_robot(self, stationary_duration):
        """Provide detailed diagnostic information when robot is stuck."""
        self.get_logger().error("🔍 DIAGNOSTIC INFORMATION:")
        self.get_logger().error(f"   • Stationary duration: {stationary_duration:.1f} seconds")
        self.get_logger().error(f"   • Goal active: {self.is_goal_active}")
        self.get_logger().error(f"   • Robot position: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})")
        
        if self.last_goal_send_time:
            time_since_goal = (self.get_clock().now() - self.last_goal_send_time).nanoseconds / 1e9
            self.get_logger().error(f"   • Time since last goal sent: {time_since_goal:.1f} seconds")
        
        # Check if we have map data
        if self.map_data is None:
            self.get_logger().error("   ❌ NO MAP DATA - This could cause navigation issues!")
        else:
            self.get_logger().error("   ✅ Map data available")
            
        # Check for nearby obstacles
        if self.robot_pose:
            obstacle_count = self._count_nearby_obstacles(self.robot_pose.x, self.robot_pose.y)
            self.get_logger().error(f"   • Nearby obstacles: {obstacle_count}")
            
            if obstacle_count > 5:
                self.get_logger().error("   ⚠️ High obstacle density - robot may be trapped!")
        
        # Provide recovery suggestions
        self.get_logger().error("🛠️ RECOVERY SUGGESTIONS:")
        self.get_logger().error("   1. Check if Nav2 is still running: ros2 node list | grep nav2")
        self.get_logger().error("   2. Check Nav2 status: ros2 topic echo /navigate_to_pose/_action/status")
        self.get_logger().error("   3. Check for obstacles around robot")
        self.get_logger().error("   4. Try canceling current goal and selecting new one")
        self.get_logger().error("   5. Check robot's laser scan: ros2 topic echo /scan")

    def _check_goal_timeout(self):
        """Check if current goal has been active for too long."""
        if not self.is_goal_active or not self.last_goal_send_time:
            return
            
        current_time = self.get_clock().now()
        goal_duration = (current_time - self.last_goal_send_time).nanoseconds / 1e9
        
        # If goal has been active for more than 60 seconds
        if goal_duration > 60:
            self.get_logger().error(f"⏰ GOAL TIMEOUT! Goal has been active for {goal_duration:.1f} seconds")
            self.get_logger().error("🔍 DIAGNOSTIC: Goal may be unreachable or Nav2 is stuck")
            self.get_logger().error("🛠️ RECOVERY ACTIONS:")
            self.get_logger().error("   1. Goal will be automatically canceled")
            self.get_logger().error("   2. New goal will be selected")
            self.get_logger().error("   3. Check Nav2 logs for detailed error information")
            
            # Cancel the current goal
            self._cancel_current_goal()
            
        elif goal_duration > 30:
            self.get_logger().warn(f"⏰ Goal has been active for {goal_duration:.1f} seconds - monitoring...")

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
                                goal_key = (candidate_x, candidate_y)
                                
                                # Check if goal is in current blacklist
                                for bp in self.goal_blacklist:
                                    dist = math.hypot(candidate_x - bp.x, candidate_y - bp.y)
                                    if dist < self.blacklist_radius:
                                        is_blacklisted = True
                                        break
                                
                                # Check if goal is in timed blacklist
                                if not is_blacklisted and goal_key in self.blacklist_timers:
                                    current_time = self.get_clock().now()
                                    time_since_blacklist = (current_time - self.blacklist_timers[goal_key]).nanoseconds / 1e9
                                    if time_since_blacklist < 30:  # Still within 30 second blacklist period
                                        is_blacklisted = True
                                
                                if is_blacklisted:
                                    continue

                                # --- 2.2.4 Rank Goals ---
                                distance_from_robot = math.hypot(candidate_x - self.robot_pose.x, candidate_y - self.robot_pose.y)
                                
                                # In recovery mode, prefer goals that are further away to push robot out
                                if self.recovery_mode:
                                    # Boost score for goals that are further from robot's current position
                                    distance_boost = distance_from_robot * 0.5  # Boost score by distance
                                else:
                                    distance_boost = 0
                                
                                # Debug logging for distance calculations
                                self.get_logger().debug(f"Goal candidate at ({candidate_x:.2f}, {candidate_y:.2f}), robot at ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}), distance: {distance_from_robot:.2f}m, min_distance: {self.min_goal_distance:.2f}m")
                                
                                # Reject goals that are too close to the robot
                                # Use recovery mode distance if in recovery mode
                                min_distance = 0.4 if self.recovery_mode else self.min_goal_distance
                                if distance_from_robot < min_distance:
                                    self.get_logger().debug(f"Rejecting goal at ({candidate_x:.2f}, {candidate_y:.2f}) - too close ({distance_from_robot:.2f}m < {min_distance:.2f}m)")
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
                                
                                # Linear scoring: A*cluster_size + B*distance + C*obstacle_count + distance_boost
                                score = (SCORING_CLUSTER_SIZE_WEIGHT * size + 
                                        SCORING_DISTANCE_WEIGHT * distance_from_robot + 
                                        SCORING_OBSTACLE_WEIGHT * obstacle_count + 
                                        distance_boost)
                                
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


        # If no free space goals found, try unknown space goals
        if not ranked_goals:
            self.get_logger().warn("⚠️ No valid free space goals found, trying unknown space goals...")
            ranked_goals = self._find_unknown_space_goals(frontiers)
            
            if not ranked_goals:
                self.get_logger().warn("⚠️ No valid unknown space goals found either")
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
            if self.recovery_mode:
                self.get_logger().info("🚨 RECOVERY MODE: Selected closer goal to help robot escape")
            else:
                # Check if this was an unknown space goal
                goal_x, goal_y = goal_pose.pose.position.x, goal_pose.pose.position.y
                if self.map_data:
                    map_info = self.map_data.info
                    goal_c = int((goal_x - map_info.origin.position.x) / map_info.resolution)
                    goal_r = int((goal_y - map_info.origin.position.y) / map_info.resolution)
                    map_array = np.array(self.map_data.data).reshape((map_info.height, map_info.width))
                    if 0 <= goal_r < map_info.height and 0 <= goal_c < map_info.width:
                        if map_array[goal_r, goal_c] == -1:
                            self.get_logger().info("🔍 EXPLORATION: Selected unknown space goal for exploration")
        else:
            self.get_logger().info("All Frontiers explore.")
            if self.recovery_mode:
                self.get_logger().warn("🚨 RECOVERY MODE: No valid goals found - robot may be truly trapped")


    # ==============================================================================
    # 3.0 ROS2 Interface Callbacks
    # ==============================================================================
    def bt_log_callback(self, msg):
        """Processes behavior tree log messages to show Nav2 behavior."""
        # Debug: Always log when we receive BT messages
        self.get_logger().info(f"📊 Received BT log with {len(msg.event_log)} events")
        
        if not self.is_goal_active:
            self.get_logger().debug("📊 BT log received but no active goal - skipping detailed logging")
            return  # Only log when we have an active goal
            
        # Log behavior tree status changes
        for event in msg.event_log:
            if hasattr(event, 'current_status') and hasattr(event, 'previous_status'):
                if event.current_status != event.previous_status:
                    status_names = {
                        1: "IDLE", 2: "RUNNING", 3: "SUCCESS", 4: "FAILURE", 5: "CANCELLED", 6: "SKIPPED"
                    }
                    prev_status = status_names.get(event.previous_status, f"UNKNOWN({event.previous_status})")
                    curr_status = status_names.get(event.current_status, f"UNKNOWN({event.current_status})")
                    
                    # Only log significant status changes
                    if event.current_status in [3, 4, 5]:  # SUCCESS, FAILURE, CANCELLED
                        self.get_logger().info(f"🌳 BT Node '{event.node_name}': {prev_status} → {curr_status}")
                    elif event.current_status == 2:  # RUNNING
                        self.get_logger().debug(f"🌳 BT Node '{event.node_name}': {prev_status} → {curr_status}")

    def bt_status_callback(self, msg):
        """Processes behavior tree status change messages (alternative to BT log)."""
        if not self.is_goal_active:
            return
            
        # Log status changes from the status change topic
        status_names = {
            1: "IDLE", 2: "RUNNING", 3: "SUCCESS", 4: "FAILURE", 5: "CANCELLED", 6: "SKIPPED"
        }
        
        prev_status = status_names.get(msg.previous_status, f"UNKNOWN({msg.previous_status})")
        curr_status = status_names.get(msg.current_status, f"UNKNOWN({msg.current_status})")
        
        # Log significant status changes
        if msg.current_status in [3, 4, 5]:  # SUCCESS, FAILURE, CANCELLED
            self.get_logger().info(f"🌳 BT Status '{msg.node_name}': {prev_status} → {curr_status}")
        elif msg.current_status == 2:  # RUNNING
            self.get_logger().debug(f"🌳 BT Status '{msg.node_name}': {prev_status} → {curr_status}")

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
        self.last_goal_send_time = self.get_clock().now()
        self.stationary_start_time = None  # Reset stuck detection
        
        self.get_logger().info("🎯 Waiting for Nav2 action server...")
        if not self._nav2_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 action server not available after 5s.")
            self.get_logger().error("🔍 DIAGNOSTIC: Nav2 may not be running or not responding")
            self.get_logger().error("🛠️ SOLUTION: Check if Nav2 is launched: ros2 node list | grep nav2")
            self.is_goal_active = False
            return
        
        # This logic is based on Prac 4's method of sending waypoints.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"🚀 Sending goal to Nav2: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.get_logger().info("📊 Nav2 Behavior Tree will start processing navigation...")
        self.get_logger().info("⏱️ Goal sent at: " + str(self.last_goal_send_time.seconds_nanoseconds()))
        send_goal_future = self._nav2_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handles the response from Nav2 after sending a goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal was rejected by Nav2.")
            self.get_logger().error("🔍 DIAGNOSTIC: Goal may be unreachable or Nav2 is overloaded")
            self.get_logger().error("🛠️ SOLUTIONS:")
            self.get_logger().error("   • Check if goal is in free space")
            self.get_logger().error("   • Check Nav2 planner status: ros2 topic echo /plan")
            self.get_logger().error("   • Check for obstacles blocking the path")
            self.get_logger().error("   • Try a different goal location")
            self.is_goal_active = False
            return
        
        self.get_logger().info("✅ Goal accepted by Nav2!")
        self.get_logger().info("🌳 Behavior Tree is now active - monitoring navigation progress...")
        self.get_logger().info("⏱️ Starting navigation timer - will detect if robot gets stuck")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """Handles the final result of the navigation action."""
        # The logic to check status is based on Prac 4's waypoint cycler behavior tree check.
        result = future.result().result
        status = future.result().status

        # Detailed status logging
        status_messages = {
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING", 
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED"
        }
        
        status_name = status_messages.get(status, f"UNKNOWN({status})")
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Navigation goal succeeded!")
            self.get_logger().info("🌳 Behavior Tree completed successfully")
            self.stationary_start_time = None  # Reset stuck detection
        else:
            self.get_logger().error(f"❌ Navigation failed with status: {status_name}")
            self.get_logger().error("🌳 Behavior Tree encountered an issue")
            
            # Enhanced failure analysis
            if status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("💥 Navigation was aborted")
                self.get_logger().error("🔍 POSSIBLE CAUSES:")
                self.get_logger().error("   • Robot stuck in obstacle")
                self.get_logger().error("   • Path planning timeout")
                self.get_logger().error("   • Controller failed to follow path")
                self.get_logger().error("🛠️ DEBUGGING COMMANDS:")
                self.get_logger().error("   • Check laser scan: ros2 topic echo /scan")
                self.get_logger().error("   • Check path: ros2 topic echo /plan")
                self.get_logger().error("   • Check Nav2 status: ros2 topic echo /navigate_to_pose/_action/status")
                
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn("⏹️ Navigation was canceled")
                self.get_logger().warn("🔍 POSSIBLE CAUSES:")
                self.get_logger().warn("   • New goal was sent before previous completed")
                self.get_logger().warn("   • Manual cancellation")
                
            # Calculate time spent on this goal
            if self.last_goal_send_time:
                goal_duration = (self.get_clock().now() - self.last_goal_send_time).nanoseconds / 1e9
                self.get_logger().error(f"⏱️ Goal was active for {goal_duration:.1f} seconds before failing")
                
            if self.robot_pose: # Add the current location to timed blacklist on failure
                failed_point = Point()
                failed_point.x = self.robot_pose.x
                failed_point.y = self.robot_pose.y
                
                # Add to timed blacklist (30 seconds)
                goal_key = (failed_point.x, failed_point.y)
                self.blacklist_timers[goal_key] = self.get_clock().now()
                self.goal_blacklist.append(failed_point)
                
                self.get_logger().info(f"📍 Added ({failed_point.x:.2f}, {failed_point.y:.2f}) to 30-second blacklist.")
                self.get_logger().info(f"📊 Blacklist now contains {len(self.goal_blacklist)} failed locations")
                
                # Trigger recovery mode if multiple failures
                if len(self.goal_blacklist) >= 3:
                    self._trigger_recovery_mode()
        
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