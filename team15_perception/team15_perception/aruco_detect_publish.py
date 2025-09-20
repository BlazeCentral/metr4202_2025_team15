#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from collections import deque, defaultdict
from typing import Dict, Deque, Tuple
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
import math

class ArucoDetectPublishNode(Node):
    """
    A ROS2 node to detect ArUco markers, estimate their pose, and publish them.
    This version is refactored into a class to properly handle ROS2 parameters and lifecycle.
    """

    def __init__(self):
        """Initializes the node, its parameters, and ROS interfaces."""
        super().__init__('aruco_detect_publish')

        # --- 1. Declare and Get Parameters ---
        # This allows the node to be configured from a YAML file.
        self.declare_parameter("dictionary", "DICT_6X6_250")
        self.declare_parameter("marker_size_m", 0.10)
        self.declare_parameter("reproj_error_thresh_px", 3.0)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("min_hits_for_publish", 2)
        self.declare_parameter("max_track_age_sec", 2.0)

        # Retrieve the parameter values
        dict_name = self.get_parameter("dictionary").get_parameter_value().string_value
        marker_size_m = self.get_parameter("marker_size_m").get_parameter_value().double_value
        self.reproj_px_max = self.get_parameter("reproj_error_thresh_px").get_parameter_value().double_value
        window_N = self.get_parameter("smoothing_window").get_parameter_value().integer_value
        self.min_hits = self.get_parameter("min_hits_for_publish").get_parameter_value().integer_value
        self.max_age_s = self.get_parameter("max_track_age_sec").get_parameter_value().double_value

        self.get_logger().info(f"Using ArUco dictionary: {dict_name}, Marker size: {marker_size_m}m")

        # --- 2. Initialize ROS and CV Components ---
        self.bridge = CvBridge()
        self.camera_K = None  # Intrinsic matrix
        self.camera_D = None  # Distortion coefficients
        self.image_frame_id = "camera_rgb_optical_frame"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers for the detected targets and their visualization
        self.targets_pub = self.create_publisher(PoseArray, "/targets", 10)
        self.viz_pub = self.create_publisher(MarkerArray, "/targets_viz", 10)

        # A dictionary to store recent detections for each marker ID for smoothing
        self.tracks: Dict[int, Deque[Tuple[np.ndarray, float, Time]]] = defaultdict(lambda: deque(maxlen=window_N))

        # Setup ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Pre-compute the 3D coordinates of the marker corners in its own frame
        half = marker_size_m / 2.0
        self.obj_pts = np.array([
            [-half,  half, 0.0], [ half,  half, 0.0],
            [ half, -half, 0.0], [-half, -half, 0.0]
        ], dtype=np.float32)

        # --- 3. Subscriptions ---
        # These trigger the main logic of the node.
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_camera_info, 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        self.get_logger().info("✅ ArUco detector node is ready and waiting for camera data.")

    def _on_camera_info(self, msg: CameraInfo):
        """Callback to receive and store camera calibration parameters."""
        if self.camera_K is None:
            self.camera_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.camera_D = np.array(msg.d, dtype=np.float64).reshape(-1,)
            self.image_frame_id = msg.header.frame_id
            self.get_logger().info("Camera calibration received successfully.")

    def _on_image(self, msg: Image):
        """Main callback to process each incoming camera image."""
        if self.camera_K is None:
            self.get_logger().warn("Waiting for camera calibration...", throttle_duration_sec=5)
            return

        stamp = Time.from_msg(msg.header.stamp)
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # Detect markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # If no markers are found, publish an empty array and exit
        if ids is None or len(ids) == 0:
            empty_pose_array = PoseArray()
            empty_pose_array.header.stamp = msg.header.stamp
            empty_pose_array.header.frame_id = "map"
            self.targets_pub.publish(empty_pose_array)
            return

        ids = ids.flatten().tolist()
        confident_ids = []

        # Process each detected marker
        for i, marker_id in enumerate(ids):
            image_pts = corners[i].reshape(-1, 2).astype(np.float32)

            # Estimate the pose of the marker relative to the camera
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, image_pts, self.camera_K, self.camera_D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: continue

            # Quality check: filter out detections with high reprojection error
            proj, _ = cv2.projectPoints(self.obj_pts, rvec, tvec, self.camera_K, self.camera_D)
            reproj_err = float(np.linalg.norm(proj.reshape(-1, 2) - image_pts, axis=1).mean())
            if reproj_err > self.reproj_px_max: continue

            # Convert rotation vector to a simplified yaw quaternion
            R, _ = cv2.Rodrigues(rvec)
            yaw = math.atan2(R[1, 0], R[0, 0])
            qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

            # Create a PoseStamped message in the camera's frame
            ps_cam = PoseStamped()
            ps_cam.header.stamp = msg.header.stamp
            ps_cam.header.frame_id = self.image_frame_id
            ps_cam.pose.position.x = float(tvec[0])
            ps_cam.pose.position.y = float(tvec[1])
            ps_cam.pose.position.z = float(tvec[2])
            ps_cam.pose.orientation.z = qz
            ps_cam.pose.orientation.w = qw
            
            # Transform the pose from the camera frame to the map frame
            try:
                transform = self.tf_buffer.lookup_transform("map", self.image_frame_id, stamp, timeout=Duration(seconds=0.2))
                ps_map = do_transform_pose(ps_cam.pose, transform)

                # Store the transformed pose for smoothing
                px, py, pz = ps_map.position.x, ps_map.position.y, ps_map.position.z
                qx, qy, qz, qw = ps_map.orientation.x, ps_map.orientation.y, ps_map.orientation.z, ps_map.orientation.w
                yaw_map = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                
                self.tracks[marker_id].append((np.array([px, py, pz]), yaw_map, stamp))
                confident_ids.append(marker_id)
            except TransformException as e:
                self.get_logger().debug(f"TF lookup failed from '{self.image_frame_id}' to 'map': {e}")
                continue

        # --- Build and Publish Smoothed Outputs ---
        self._publish_smoothed_targets(msg.header.stamp)

    def _publish_smoothed_targets(self, stamp):
        """Filters, smooths, and publishes confident marker poses."""
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = "map"
        marker_array = MarkerArray()
        marker_seq = 0
        now = self.get_clock().now()

        for marker_id in list(self.tracks.keys()):
            buf = self.tracks[marker_id]
            
            # Prune old detections from the buffer
            pruned = deque(maxlen=buf.maxlen)
            for (xyz, yaw_m, detection_time) in buf:
                if (now - detection_time).nanoseconds * 1e-9 < self.max_age_s:
                    pruned.append((xyz, yaw_m, detection_time))
            self.tracks[marker_id] = pruned
            
            # Check if the track is still confident enough to publish
            if len(pruned) < self.min_hits:
                continue

            # Calculate the smoothed pose using a moving average
            positions = np.array([e[0] for e in pruned])
            yaws = np.array([e[1] for e in pruned])
            pos_mean = positions.mean(axis=0)
            # Average angles correctly using atan2 of mean sin/cos
            yaw_mean = math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean())

            # Create the final smoothed Pose message
            smoothed_pose = Pose()
            smoothed_pose.position.x, smoothed_pose.position.y, smoothed_pose.position.z = float(pos_mean[0]), float(pos_mean[1]), float(pos_mean[2])
            qz, qw = math.sin(yaw_mean * 0.5), math.cos(yaw_mean * 0.5)
            smoothed_pose.orientation.z, smoothed_pose.orientation.w = qz, qw
            pose_array.poses.append(smoothed_pose)

            # Create a visualization marker for RViz
            m = Marker()
            m.header = pose_array.header
            m.ns = "aruco_targets"
            m.id = marker_id # Use marker ID for consistent visualization
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = smoothed_pose
            m.scale.x, m.scale.y, m.scale.z = 0.2, 0.05, 0.05
            m.color.r, m.color.g, m.color.a = 0.0, 1.0, 1.0
            marker_array.markers.append(m)

        self.targets_pub.publish(pose_array)
        if len(marker_array.markers) > 0:
            self.viz_pub.publish(marker_array)
            self.get_logger().info(f"Published {len(pose_array.poses)} smoothed ArUco targets.")


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = ArucoDetectPublishNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

