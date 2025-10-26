import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
import numpy as np
from collections import deque
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from scipy.ndimage import convolve
import time

# filter constants and active filters
SIZE_FILTER = True
MIN_CLUSTER_SIZE = 4 # number of cells
VALUE_RADIUS = 1.0  # radius for calculating a waypoints exploration value
SIZE_WEIGHT = 2.3
DIST_WEIGHT = -2.0 # negative value prefers close waypoints
ANGLE_WEIGHT = 5.0  

EXTEND_DIST_THRES = 2.5  # dist from which waypoints begin to be extended
EXTEND_DIST = 0.4  
RETRACT_DIST = 0.0


# --- STUCK / RECOVERY TUNING ---
# --- STUCK / RECOVERY TUNING ---
# --- STUCK / RECOVERY TUNING ---
STUCK_WINDOW_SEC = 10        # look-back window for displacement (s)
STUCK_MIN_DISP  = 0.30         # m; below this over window => "stuck"
RECOVERY_COOLDOWN_SEC = 15.0   # min time between nudges (s)
RECOVERY_MIN_R  = 0.8          # m, min radius for recovery goal
RECOVERY_MAX_R  = 4.3          # m, max radius for recovery goal
RECOVERY_OPEN_PATCH = 2        # half-size (cells) of free patch check (=> (2k+1)^2)
FREE_MAX_COST = 60             # treat < 40 as free/safe in costmap

# --- STUCK / RECOVERY TUNING ---
# --- STUCK / RECOVERY TUNING ---
# --- STUCK / RECOVERY TUNING ---


class ExploreNavNode(Node):
    def __init__(self):
        super().__init__('explore_nav')

        # asynchronous variables
        self.robot_pose = None
        self.robot_yaw = None
        self.costmap = None
        self.frontier_points = []
        self.current_goal = None
        self.no_frontier_counter = 0
        self.max_no_frontier_attempts = 10  # stop after 10 tries

                # --- STUCK/RECOVERY STATE ---
        self.pose_history = deque(maxlen=600)  # (t_sec, x, y)
        self.last_recovery_sec = -1e9          # so first nudge is allowed
        # Periodic stuck check
        self.create_timer(2.0, self.stuck_watchdog)

        # Define a QoS profile for the map subscriber to ensure we get the latest map
        map_qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        
        # Subscriber for the map
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', 
                                                       self.map_callback, map_qos_profile)

        # frontier publisher for visualisation
        self.frontier_publisher = self.create_publisher(OccupancyGrid, '/frontiers', 10)
        self.frontier_point_publisher = self.create_publisher(Marker, '/frontier_points', 10)

        # Action client for Nav2
        self._nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF2 buffer and listener for robot pose updates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self.update_robot_pose)  # 1 Hz
        self.create_timer(3.0, self.set_goal) # set new goal periodically

    def map_callback(self, msg):
        self.costmap = msg

        # compute frontiers
        frontier_map = msg
        frontier_map.data = self.find_frontiers(msg)

        # compute frontier points and markers
        frontier_points = self.find_frontier_points(frontier_map)  # list
        self.frontier_points = frontier_points
        markers = self.mark_points_viz(frontier_points, frontier_map.header) # markers

        # publish (for visualisation only)
        self.frontier_publisher.publish(frontier_map)
        self.frontier_point_publisher.publish(markers)

                # Check if frontiers exist
        if len(frontier_points) == 0:
            self.no_frontier_counter += 1
            self.get_logger().warn(
                f"No frontiers found. Attempt {self.no_frontier_counter}/{self.max_no_frontier_attempts}"
            )
            if self.no_frontier_counter >= self.max_no_frontier_attempts:
                self.get_logger().info("Mapping complete — no new frontiers detected.")
                print("Mapping complete — no new frontiers detected.")
                rclpy.shutdown()
                return
                
        else:
            # reset counter if new frontiers appear
            self.no_frontier_counter = 0
        

        self.get_logger().info('Published frontier map and points')



    def update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_pose = (t.transform.translation.x, t.transform.translation.y)

            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_yaw = yaw

            # NEW: record time-stamped pose for stuck detection
            now_sec = self.get_clock().now().seconds_nanoseconds()[0] \
                    + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
            self.pose_history.append((now_sec, self.robot_pose[0], self.robot_pose[1]))

            self.get_logger().info(
                f"Robot position: x={self.robot_pose[0]:.2f}, y={self.robot_pose[1]:.2f}, yaw={math.degrees(yaw):.1f}°")

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.robot_pose = None
            self.robot_yaw = None

    def stuck_watchdog(self):
        """Periodically check displacement. If "stuck", send a nudge Nav2 goal to a safe open patch."""
        if self.robot_pose is None or self.costmap is None:
            return

        now_sec = self.get_clock().now().seconds_nanoseconds()[0] \
                + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        # Need at least two points to compare
        if len(self.pose_history) < 2:
            return

        # Find an older point within the look-back window
        oldest_recent = None
        for t_sec, x, y in reversed(self.pose_history):  # from newest to older
            if now_sec - t_sec >= STUCK_WINDOW_SEC:
                oldest_recent = (t_sec, x, y)
                break
        if oldest_recent is None:
            # history doesn’t go back far enough yet
            return

        # Displacement over the window
        x0, y0 = oldest_recent[1], oldest_recent[2]
        x1, y1 = self.robot_pose
        disp = math.hypot(x1 - x0, y1 - y0)

        # Respect cooldown
        if now_sec - self.last_recovery_sec < RECOVERY_COOLDOWN_SEC:
            return

        if disp < STUCK_MIN_DISP:
            self.get_logger().warn(
                f"Stuck detected (disp={disp:.2f} m over {STUCK_WINDOW_SEC:.0f}s). Attempting recovery nudge...")
            if self.try_recovery_nudge():
                self.last_recovery_sec = now_sec
            else:
                self.get_logger().warn("Recovery nudge failed to find a safe open patch.")


    def try_recovery_nudge(self) -> bool:
        """Pick a safe open-space patch on the known map and send a Nav2 goal there."""
        if self.costmap is None or self.robot_pose is None:
            return False

        # Candidate ring(s)
        radii = [RECOVERY_MIN_R,
                0.5 * (RECOVERY_MIN_R + RECOVERY_MAX_R),
                RECOVERY_MAX_R]
        num_angles = 16  # 22.5° steps
        rx, ry = self.robot_pose

        for r in radii:
            for k in range(num_angles):
                theta = (2 * math.pi) * (k / num_angles)
                wx = rx + r * math.cos(theta)
                wy = ry + r * math.sin(theta)

                if not self._world_point_is_safe(wx, wy, RECOVERY_OPEN_PATCH):
                    continue

                # Send a one-off Nav2 goal to this safe open patch
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x = wx
                goal.pose.position.y = wy
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0  # neutral orientation

                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = goal

                if self._nav2_action_client.server_is_ready():
                    self._nav2_action_client.send_goal_async(goal_msg)
                    self.get_logger().info(
                        f"Recovery nudge goal sent to open space at x={wx:.2f}, y={wy:.2f}")
                    time.sleep(4)
                    return True

        return False


    def _world_to_map(self, wx: float, wy: float):
        """Convert world coords to map indices, returning (mx, my) or None if OOB."""
        info = self.costmap.info
        width, height = info.width, info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None
        return mx, my


    def _world_point_is_safe(self, wx: float, wy: float, half_patch_cells: int) -> bool:
        """Check a (2k+1)x(2k+1) patch around (wx, wy) is free/known."""
        idx = self._world_to_map(wx, wy)
        if idx is None:
            return False

        info = self.costmap.info
        width, height = info.width, info.height
        data = np.array(self.costmap.data, dtype=np.int16).reshape((height, width))

        mx, my = idx
        k = half_patch_cells
        x_min = max(mx - k, 0)
        x_max = min(mx + k, width - 1)
        y_min = max(my - k, 0)
        y_max = min(my + k, height - 1)

        patch = data[y_min:y_max + 1, x_min:x_max + 1]

        # Reject if any unknown (-1) or any cost >= FREE_MAX_COST
        if np.any(patch == -1):
            return False
        if np.any(patch >= FREE_MAX_COST):
            return False
        return True
            
    def find_frontiers(self, costmap_msg):
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        data = np.array(costmap_msg.data, dtype=np.int8).reshape((height, width))

        # Binary masks
        unknown = (data == -1)
        free = (data >= 0) & (data < 60)  # traversable area

        # 8-connected neighborhood kernel
        kernel = np.ones((3, 3), dtype=int)

        # Count free cells adjacent to each unknown cell
        free_neighbors = convolve(free.astype(int), kernel, mode='constant', cval=0)

        # Frontier = unknown cell with at least 1 free neighbor
        frontier_mask = unknown & (free_neighbors > 0)

        frontier = np.zeros_like(data, dtype=np.int8)
        frontier[frontier_mask] = 100

        return frontier.flatten().tolist()


    def find_frontier_points(self, frontier_map):
        width = frontier_map.info.width
        height = frontier_map.info.height
        resolution = frontier_map.info.resolution
        origin = frontier_map.info.origin.position

        # Convert map to 2D array
        data = np.array(frontier_map.data, dtype=np.int8).reshape((height, width))
        frontier_mask = (data == 100)
        visited = np.zeros_like(frontier_mask, dtype=bool)

        # 8-connected neighbours
        neighbors = [(-1, -1), (-1, 0), (-1, 1),
                    (0, -1),          (0, 1),
                    (1, -1),  (1, 0), (1, 1)]

        def bfs(start_y, start_x):
            q = deque([(start_y, start_x)])
            cluster = []
            visited[start_y, start_x] = True
            while q:
                y, x = q.popleft()
                cluster.append((y, x))
                for dy, dx in neighbors:
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        if frontier_mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            return cluster

        # Collect all clusters
        clusters = []
        for y in range(height):
            for x in range(width):
                if frontier_mask[y, x] and not visited[y, x]:
                    cluster = bfs(y, x)
                    if SIZE_FILTER and len(cluster) < MIN_CLUSTER_SIZE:
                        continue
                    clusters.append(cluster)

        self.get_logger().info(f"Total clusters: {len(clusters)}")
        frontier_count = len(clusters)
        if frontier_count < 10:
            clusters = sorted(clusters, key=len, reverse=True)[:5]
        else:
            clusters = sorted(clusters, key=len, reverse=True)[:10]

        # Compute midpoints
        points = []
        for cluster in clusters:
            mean_y = int(np.mean([p[0] for p in cluster]))
            mean_x = int(np.mean([p[1] for p in cluster]))
            world_x = origin.x + (mean_x + 0.5) * resolution
            world_y = origin.y + (mean_y + 0.5) * resolution

            # Extend away from robot
            if self.robot_pose is not None:
                rx, ry = self.robot_pose
                dx = world_x - rx
                dy = world_y - ry
                ryaw = self.robot_yaw

                length = math.hypot(dx, dy)
                goal_angle = math.atan2(dy, dx)
                angle_diff = abs((goal_angle - ryaw + math.pi) % (2 * math.pi) - math.pi)  # normalize to [-pi, pi]

                if length > 0 and length < EXTEND_DIST_THRES and angle_diff < math.radians(90): # extend if close and in front
                    extend_dist = EXTEND_DIST  # meters
                    world_x += dx / length * extend_dist
                    world_y += dy / length * extend_dist

            points.append(Point(x=world_x, y=world_y, z=0.0))

        return points


    def select_goal(self):
        if not self.frontier_points or self.robot_pose is None or self.costmap is None or self.robot_yaw is None:  # <<<
            return None
        
        costmap = self.costmap  # Costmap
        points = self.frontier_points  # list of Point() s
        rx, ry = self.robot_pose 
        ryaw = self.robot_yaw  

        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution
        origin = costmap.info.origin.position

        costmap_data = np.array(costmap.data, dtype=np.int8).reshape((height, width))

        scored_points = []
        radius_cells = int(VALUE_RADIUS / resolution)

        for pt in points:
            # Convert world coords to map indices
            mx = int((pt.x - origin.x) / resolution)
            my = int((pt.y - origin.y) / resolution)

            # neighborhood bounds
            x_min = max(mx - radius_cells, 0)
            x_max = min(mx + radius_cells + 1, width)
            y_min = max(my - radius_cells, 0)
            y_max = min(my + radius_cells + 1, height)

            local_region = costmap_data[y_min:y_max, x_min:x_max]
            unknown_count = np.sum(local_region == -1)

            # distance to robot
            dx = pt.x - rx
            dy = pt.y - ry
            distance = math.hypot(dx, dy)

            #Angular bias: in front of robot within 90 deg
            goal_angle = math.atan2(dy, dx)
            angle_diff = abs((goal_angle - ryaw + math.pi) % (2 * math.pi) - math.pi)  # normalize to [-pi, pi]
            
            if angle_diff < math.radians(90):
                angle_bias = max(0.0, math.cos(angle_diff)) 
            else:
                angle_bias = 0.0  # behind robot

            score = SIZE_WEIGHT * unknown_count + DIST_WEIGHT * distance + ANGLE_WEIGHT * angle_bias  # <<<
            scored_points.append((score, pt))

        if not scored_points:
            return None

        # sort by score descending (largest score = best)
        scored_points.sort(key=lambda x: x[0], reverse=True)

        self.get_logger().info(f"1st place score: {scored_points[0]}")

        best_point = scored_points[0][1]

        self.current_goal = best_point
        
        return best_point


    def set_goal(self):
        goal_pos = self.select_goal()
        if goal_pos:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position = goal_pos
            goal.pose.orientation.w = 1.0  # facing forward

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal

            if self._nav2_action_client.server_is_ready():
                self._nav2_action_client.send_goal_async(goal_msg)
                self.get_logger().info(f"Sent goal: x={goal_pos.x:.2f}, y={goal_pos.y:.2f}")


    def mark_points_viz(self, points, header):
        # Create Marker for RViz
        marker = Marker()
        marker.header = header
        marker.ns = "frontier_midpoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = points

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
