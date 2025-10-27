#!/usr/bin/env python3
#REFERENCE: https://github.com/Rishit-katiyar/ArUcoMarkerDetector/tree/main
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import defaultdict
import time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
import math

# Simple configuration
ARUCO_DICTIONARY_NAME = "DICT_6X6_250"
MARKER_SIZE_METERS = 0.10
MIN_DETECTIONS_FOR_CONFIRMATION = 2  # Minimum detections to consider marker confirmed

class ArucoDetectPublishNode(Node):
    """Simplified ROS2 node to detect ArUco markers and publish their poses."""

    def __init__(self):
        super().__init__('aruco_detect_publish')
        
        # Initialize components
        self.bridge = CvBridge()
        self.camera_K = None
        self.camera_D = None
        self.calibration_received = False
        
        # Marker tracking system - single source of truth
        self.tracked_markers = {}  # {marker_id: {'pose': Pose, 'detection_count': int, 'last_seen': timestamp, 'confirmed': bool, 'pose_history': []}}
        self.smoothing_factor = 0.3  # How much to weight new detections vs existing pose (0.0 = no change, 1.0 = full replacement)
        
        # TF for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.targets_pub = self.create_publisher(PoseArray, "/targets", 10)
        self.viz_pub = self.create_publisher(MarkerArray, "/targets_viz", 10)
        
        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICTIONARY_NAME))
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Marker 3D points
        half = MARKER_SIZE_METERS / 2.0
        self.obj_pts = np.array([
            [-half,  half, 0.0], [ half,  half, 0.0],
            [ half, -half, 0.0], [-half, -half, 0.0]
        ], dtype=np.float32)
        
        # Subscriptions
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_camera_info, 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        self.get_logger().info("ArUco detector ready")

    def _on_camera_info(self, msg: CameraInfo):
        """Store camera calibration parameters."""
        if not self.calibration_received:
            self.camera_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.camera_D = np.array(msg.d, dtype=np.float64).reshape(-1,)
            self.image_frame_id = msg.header.frame_id
            self.calibration_received = True
            self.get_logger().info("Camera calibration received")

    def _on_image(self, msg: Image):
        """Process camera image and detect ArUco markers."""
        # Check calibration rx
        if not self.calibration_received:
            return
        # Convert img to cv format
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Debug messaage
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
            
        # Convert to grayscale and detect markers
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None:
            # No markers detected in current frame, but publish confirmed markers if any
            self._publish_persistent_results(msg.header.stamp)
            return

        # Process detected markers
        current_timestamp = time.time()
        current_frame_detections = {}
        
        for i, marker_id in enumerate(ids.flatten()):
            image_pts = corners[i].reshape(-1, 2).astype(np.float32)
            
            # Estimate pose
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, image_pts, self.camera_K, self.camera_D)
            if not ok:
                continue

            # Convert to quaternion
            R, _ = cv2.Rodrigues(rvec)
            yaw = math.atan2(R[1, 0], R[0, 0])
            qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

            # Create pose in camera frame
            ps_cam = PoseStamped()
            ps_cam.header.stamp = msg.header.stamp
            ps_cam.header.frame_id = self.image_frame_id
            ps_cam.pose.position.x = float(tvec[0])
            ps_cam.pose.position.y = float(tvec[1])
            ps_cam.pose.position.z = float(tvec[2])
            ps_cam.pose.orientation.z = qz
            ps_cam.pose.orientation.w = qw
            
            # Transform to map frame
            try:
                transform = self.tf_buffer.lookup_transform("map", self.image_frame_id, msg.header.stamp)
                ps_map = do_transform_pose(ps_cam.pose, transform)
                
                # Store current frame detection
                current_frame_detections[marker_id] = {
                    'pose': ps_map,
                    'timestamp': current_timestamp
                }
                
            except TransformException:
                continue

        # Update tracking system
        self._update_marker_tracking(current_frame_detections, current_timestamp)
        
        # Publish all confirmed markers (persistent)
        self._publish_persistent_results(msg.header.stamp)

    def _update_marker_tracking(self, current_detections, timestamp):
        """Update marker tracking system with current frame detections."""
        # Process currently detected markers
        for marker_id, detection_data in current_detections.items():
            if marker_id in self.tracked_markers:
                # Marker already exists - smooth pose update and increment count
                old_pose = self.tracked_markers[marker_id]['pose']
                new_pose = detection_data['pose']
                
                # Smooth pose update (weighted average)
                smoothed_pose = self._smooth_pose(old_pose, new_pose)
                
                self.tracked_markers[marker_id]['pose'] = smoothed_pose
                self.tracked_markers[marker_id]['detection_count'] += 1
                self.tracked_markers[marker_id]['last_seen'] = timestamp
                
                # Check if marker should be confirmed (only log once)
                if (not self.tracked_markers[marker_id]['confirmed'] and 
                    self.tracked_markers[marker_id]['detection_count'] >= MIN_DETECTIONS_FOR_CONFIRMATION):
                    self.tracked_markers[marker_id]['confirmed'] = True
                    self.get_logger().info(f"Marker {marker_id} confirmed after {self.tracked_markers[marker_id]['detection_count']} detections")
            else:
                # First detection of this marker
                self.tracked_markers[marker_id] = {
                    'pose': detection_data['pose'],
                    'detection_count': 1,
                    'last_seen': timestamp,
                    'confirmed': False
                }
                self.get_logger().info(f"New marker {marker_id} detected")
        
        # NEVER remove markers - they stay forever once detected!
        # Only update poses and detection counts

    def _smooth_pose(self, old_pose, new_pose):
        """Smooth pose update using weighted average."""
        smoothed = Pose()
        
        # Smooth position (weighted average)
        smoothed.position.x = (1 - self.smoothing_factor) * old_pose.position.x + self.smoothing_factor * new_pose.position.x
        smoothed.position.y = (1 - self.smoothing_factor) * old_pose.position.y + self.smoothing_factor * new_pose.position.y
        smoothed.position.z = (1 - self.smoothing_factor) * old_pose.position.z + self.smoothing_factor * new_pose.position.z
        
        # Smooth orientation (simple average for quaternion components)
        smoothed.orientation.x = (1 - self.smoothing_factor) * old_pose.orientation.x + self.smoothing_factor * new_pose.orientation.x
        smoothed.orientation.y = (1 - self.smoothing_factor) * old_pose.orientation.y + self.smoothing_factor * new_pose.orientation.y
        smoothed.orientation.z = (1 - self.smoothing_factor) * old_pose.orientation.z + self.smoothing_factor * new_pose.orientation.z
        smoothed.orientation.w = (1 - self.smoothing_factor) * old_pose.orientation.w + self.smoothing_factor * new_pose.orientation.w
        
        return smoothed

    def _publish_persistent_results(self, stamp):
        """Publish all tracked markers persistently."""
        if not self.tracked_markers:
            # No tracked markers, publish empty results
            self._publish_empty_results(stamp)
            return
        
        # Create poses and markers for ALL tracked markers (both confirmed and unconfirmed)
        poses = []
        markers = []
        
        for marker_id, marker_data in self.tracked_markers.items():
            poses.append(marker_data['pose'])
            marker = self._create_visualization_marker(marker_id, marker_data['pose'], stamp)
            markers.append(marker)
        
        # Publish results
        self._publish_results(poses, markers, stamp)
        
        # Log status
        confirmed_count = sum(1 for m in self.tracked_markers.values() if m['confirmed'])
        self.get_logger().info(f"Published {len(poses)} ArUco targets ({confirmed_count} confirmed, {len(poses)-confirmed_count} unconfirmed)")
        self.get_logger().debug(f"All tracked markers: {list(self.tracked_markers.keys())}")

    def _publish_empty_results(self, stamp):
        """Publish empty results when no markers detected."""
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = "map"
        self.targets_pub.publish(pose_array)

    def _create_visualization_marker(self, marker_id, pose, stamp):
        """Create RViz visualization marker."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "map"
        marker.ns = "aruco_targets"
        marker.id = int(marker_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.05, 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
        return marker

    def _publish_results(self, poses, markers, stamp):
        """Publish detected poses and visualization markers."""
        # Publish poses
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = "map"
        pose_array.poses = poses
        self.targets_pub.publish(pose_array)
        
        # Publish visualization to rviz
        if markers:
            marker_array = MarkerArray()
            marker_array.markers = markers
            self.viz_pub.publish(marker_array)
            self.get_logger().info(f"Published {len(poses)} ArUco targets")


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
