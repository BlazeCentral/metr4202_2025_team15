import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped


class WaypointCycler(Node):

    def __init__(self):
        super().__init__('waypoint_cycler')  # This defines the node name

        # Create a subscriber to the behavior_tree_log topic
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # Keep track of the number of waypoints sent
        self.waypoint_counter = 0

        # Manually input the two waypoints
        p0 = PoseStamped()
        p0.header.frame_id = 'map'
        p0.pose.position.x = 1.7
        p0.pose.position.y = -0.5
        p0.pose.orientation.w = 1.0

        p1 = PoseStamped()
        p1.header.frame_id = 'map'
        p1.pose.position.x = -0.6
        p1.pose.position.y = 1.8
        p1.pose.orientation.w = 1.0

        # Store waypoints in a list
        self.waypoints = [p0, p1]

    def bt_log_callback(self, msg: BehaviorTreeLog):
        # Check the current state of the behaviour tree
        # When the robot has reached its waypoint, the top-level node state
        # is 'NavigateRecovery' and the event status is 'IDLE'
        # If the behaviour tree is in this state, send the next waypoint
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.send_waypoint()

    def send_waypoint(self):
        # Keep track of the number of waypoints sent
        self.waypoint_counter += 1

        # Mod operation to determine if waypoint count is odd or even
        if self.waypoint_counter % 2:
            self.publisher_.publish(self.waypoints[1])
        else:
            self.publisher_.publish(self.waypoints[0])


def main(args=None):
    rclpy.init(args=args)

    # Create a new WaypointCycler node
    waypoint_cmder = WaypointCycler()

    # Execute the node
    rclpy.spin(waypoint_cmder)

    # Shutdown
    waypoint_cmder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
