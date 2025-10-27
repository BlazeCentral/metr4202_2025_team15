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

# filter constants and active filters
SIZE_FILTER = True
MIN_CLUSTER_SIZE = 3 # number of cells
VALUE_RADIUS = 1.0  # radius for calculating a waypoints exploration value
SIZE_WEIGHT = 3.0
DIST_WEIGHT = -5.0 # negative value prefers close waypoints
ANGLE_WEIGHT = 6.0  

EXTEND_DIST_THRES = 1.5  # dist from which waypoints begin to be extended
EXTEND_DIST = 0.5  
RETRACT_DIST = 0.0


class ExploreNavNode(Node):
     '''
    This node handles map processing to identify frontiers, 
    grouping frontiers into a list of candidate waypoints, 
    and decision making on next goal waypoint.
    '''
    def __init__(self):
        super().__init__('explore_nav')

        # asynchronous variables
        self.robot_pose = None
        self.robot_yaw = None
        self.costmap = None
        self.frontier_points = []
        self.current_goal = None

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
        self.create_timer(2.5, self.set_goal) # set new goal periodically

    def map_callback(self, msg):
        '''
        Retrieves a global costmap via the /global_costmap/costmap topic.
        Calls the required processing functions to find, group, and publish frontiers.
        '''
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

        self.get_logger().info('Published frontier map and points')

    def update_robot_pose(self):
         '''
        Performs tf operations to determine robot pose. Asynchronous, updates every second.
        '''
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_pose = (t.transform.translation.x, t.transform.translation.y)

            q = t.transform.rotation
            # convert quaternion to yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_yaw = yaw  
            # Log
            self.get_logger().info(
                f"Robot position: x={self.robot_pose[0]:.2f}, y={self.robot_pose[1]:.2f}, yaw={math.degrees(yaw):.1f}°") 

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.robot_pose = None
            self.robot_yaw = None 

    def find_frontiers(self, costmap_msg):
        '''
        Groups frontiers using BFS. Frontier cells touching are considered to be connected nodes, 
        search is finished when no more connected nodes.
        Includes filtering small / noisy frontiers.
        Points are considered midpoint of list of points.
        Returns list of midpoints indicating candidate waypoints.
        '''
        
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        data = np.array(costmap_msg.data, dtype=np.int8).reshape((height, width))

        # Binary masks
        unknown = (data == -1)
        free = (data >= 0) & (data < 75)  # traversable area

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
        '''
        Identify and cluster frontier cells in the costmap, then compute representative
        world-coordinate midpoints for navigation.

        Uses a breadth-first search (BFS) to group 8-connected frontier cells into clusters,
        filters small clusters, and converts their mean map indices to world coordinates.
        Nearby frontiers in front of the robot may be slightly extended outward to improve
        exploration goal placement.
        '''
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
            '''
            Perform a breadth-first search (BFS) from a starting frontier cell to
            collect all 8-connected neighboring frontier cells into a single cluster.
            '''
            # Init queue
            q = deque([(start_y, start_x)])
            cluster = []
            visited[start_y, start_x] = True

            # Expand search until no more frontiers
            while q:
                y, x = q.popleft()
                cluster.append((y, x))
               # Explore all connected 8 nieighbours
                for dy, dx in neighbors:
                    ny, nx = y + dy, x + dx
                    # Check bounds (dont access outside map)
                    if 0 <= ny < height and 0 <= nx < width:
                         # Add neighbour to cluster if frontier/not visited
                        if frontier_mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            return cluster

        # Collect all clusters
        clusters = []
        for y in range(height):
            for x in range(width):
                # If valid cluster
                if frontier_mask[y, x] and not visited[y, x]:
                    # Perform BFS
                    cluster = bfs(y, x)
                    # Size Filter
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
            # Midpoints
            mean_y = int(np.mean([p[0] for p in cluster]))
            mean_x = int(np.mean([p[1] for p in cluster]))
            world_x = origin.x + (mean_x + 0.5) * resolution
            world_y = origin.y + (mean_y + 0.5) * resolution

            # Extend away from robot
            if self.robot_pose is not None:
                #relative pose
                rx, ry = self.robot_pose
                dx = world_x - rx
                dy = world_y - ry
                ryaw = self.robot_yaw

                #Find displacment and angular delta
                length = math.hypot(dx, dy)
                goal_angle = math.atan2(dy, dx)
                angle_diff = abs((goal_angle - ryaw + math.pi) % (2 * math.pi) - math.pi)  # normalize to [-pi, pi]

               # Extend the point away from the robot
                if length > 0 and length < EXTEND_DIST_THRES and angle_diff < math.radians(90): # extend if close and in front
                    extend_dist = EXTEND_DIST  # meters
                    world_x += dx / length * extend_dist
                    world_y += dy / length * extend_dist

            points.append(Point(x=world_x, y=world_y, z=0.0))

        return points


    def select_goal(self):
        '''
        Considers a list of candidate waypoints to determine the ideal target. 
        Accounts for proximity, direction and exploration value. 
        Returns goal point.
        '''
        if not self.frontier_points or self.robot_pose is None or self.costmap is None or self.robot_yaw is None:  # <<<
            return None
        
        costmap = self.costmap  # Costmap
        points = self.frontier_points  # list of Point() s
        rx, ry = self.robot_pose 
        ryaw = self.robot_yaw  

        # cost map processing
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

        # Select best goal
        self.get_logger().info(f"1st place score: {scored_points[0]}")
        best_point = scored_points[0][1]
        self.current_goal = best_point
        
        return best_point


    def set_goal(self):
         '''
        Sends selected goal to nav2 stack.
        '''
        goal_pos = self.select_goal()
        if goal_pos:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position = goal_pos
            goal.pose.orientation.w = 1.0  # facing forward

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal
             
            #Goal sending/msg comms
            if self._nav2_action_client.server_is_ready():
                self._nav2_action_client.send_goal_async(goal_msg)
                self.get_logger().info(f"Sent goal: x={goal_pos.x:.2f}, y={goal_pos.y:.2f}")


    def mark_points_viz(self, points, header):
        '''
        Visualises waypoint candidates in RVIz using markers.
        '''
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
