#!/usr/bin/env python3

#blaise version 2.1
#Changes: Subscribe to raw camera info
#         Publish debug image
#         Simplified ArUco detection with ArUco.detectMarkers()
#         Simplified pose estimation with ArUco.estimatePoseSingleMarkers()



#REFERENCE: https://github.com/Rishit-katiyar/ArUcoMarkerDetector/tree/main
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import defaultdict
import time

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
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
        
        # Status update system
        self.first_aruco_found = False
        self.last_status_time = time.time()
        
        # TF for coordinate transforms - with proper buffer duration
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Track TF buffer initialization
        self.tf_buffer_ready = False
        self.tf_init_start_time = time.time()
        
        # Publishers
        self.targets_pub = self.create_publisher(PoseArray, "/targets", 10)
        self.viz_pub = self.create_publisher(MarkerArray, "/targets_viz", 10)
        self.debug_pub = self.create_publisher(Image, "/camera/debug_image", 10)
        
        # ArUco setup - simplified
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICTIONARY_NAME))
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Subscriptions - Updated for actual robot
        self.create_subscription(CameraInfo, "/camera_info", self._on_camera_info, 10)
        self.create_subscription(CompressedImage, "/image_raw/compressed", self._on_compressed_image, 10)
        self.get_logger().info("ArUco detector ready")

    def _on_camera_info(self, msg: CameraInfo):
        """Store camera calibration parameters."""
        if not self.calibration_received:
            self.camera_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.camera_D = np.array(msg.d, dtype=np.float64).reshape(-1,)
            self.image_frame_id = msg.header.frame_id
            
            # Check if frame_id is valid
            if not self.image_frame_id or self.image_frame_id == "":
                self.image_frame_id = "camera_rgb_optical_frame"  # Default frame
                self.get_logger().warn("Empty frame_id, using default: camera_rgb_optical_frame")
            
            self.calibration_received = True
            self.get_logger().info(f"Camera calibration received, frame_id: {self.image_frame_id}")

    def _on_compressed_image(self, msg: CompressedImage):
        """Process compressed camera image and detect ArUco markers."""
        if not self.calibration_received:
            return

        try:
            # Convert compressed image to OpenCV format using numpy
            import numpy as np
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().error("Failed to decode compressed image")
                return
        except Exception as e:
            self.get_logger().error(f"Compressed image conversion failed: {e}")
            return
            
        # Process the image
        self._process_image(cv_img, msg.header.stamp)

    def _on_image(self, msg: Image):
        """Process regular camera image and detect ArUco markers."""
        if not self.calibration_received:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
            
        # Process the image
        self._process_image(cv_img, msg.header.stamp)

    def _process_image(self, cv_img, stamp):
        """Common image processing logic for both compressed and regular images."""
        
        # Wait for TF buffer to be ready (minimum 2 seconds)
        if not self.tf_buffer_ready:
            if time.time() - self.tf_init_start_time < 2.0:
                self.get_logger().info("Waiting for TF buffer to initialize...")
                return
            else:
                self.tf_buffer_ready = True
                self.get_logger().info("TF buffer ready, starting marker detection")
        
        # Check TF availability and diagnose available frames
        self._diagnose_tf_tree(stamp)
        
        # Check for status updates every 5 seconds
        current_time = time.time()
        if not self.first_aruco_found and (current_time - self.last_status_time) >= 5.0:
            self.get_logger().info("No ArUco markers detected yet...")
            self.last_status_time = current_time
            
        # Convert to grayscale and detect markers
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None:
            # No markers detected in current frame, but publish confirmed markers if any
            self._publish_persistent_results(stamp)
            return

        # Mark that we found our first ArUco marker
        if not self.first_aruco_found:
            self.first_aruco_found = True
            self.get_logger().info("First ArUco marker detected! Status updates will stop.")
        
        # Use OpenCV's built-in pose estimation - much simpler!
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_METERS, self.camera_K, self.camera_D)
        
        # Draw debug visualization
        debug_img = self._draw_marker_debug(cv_img.copy(), corners, ids, rvecs, tvecs)
        self._publish_debug_image(debug_img, stamp)

        # Process detected markers
        current_timestamp = time.time()
        current_frame_detections = {}
        
        for i, marker_id in enumerate(ids.flatten()):
            # Extract pose for this marker - handle array shape correctly
            try:
                rvec = rvecs[i].flatten()
                tvec = tvecs[i].flatten()
            except (IndexError, AttributeError) as e:
                self.get_logger().error(f"Error extracting pose for marker {i}: {e}")
                continue

            # Convert to quaternion
            R, _ = cv2.Rodrigues(rvec)
            yaw = math.atan2(R[1, 0], R[0, 0])
            qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

            # Create pose in camera frame
            ps_cam = PoseStamped()
            ps_cam.header.stamp = stamp
            ps_cam.header.frame_id = self.image_frame_id
            ps_cam.pose.position.x = float(tvec[0])
            ps_cam.pose.position.y = float(tvec[1])
            ps_cam.pose.position.z = float(tvec[2])
            ps_cam.pose.orientation.z = qz
            ps_cam.pose.orientation.w = qw
            
            # Transform to map frame with multiple fallback options
            transform_success = False
            ps_map = None
            
            # Try multiple transform options in order of preference
            # FIXED: Use proper timeout and correct transform directions
            transform_options = [
                ("map", self.image_frame_id),
                ("base_link", self.image_frame_id), 
                ("base_footprint", self.image_frame_id),
                ("odom", self.image_frame_id)
            ]
            
            for target_frame, source_frame in transform_options:
                try:
                    self.get_logger().info(f"Trying transform from {source_frame} to {target_frame}")
                    # FIXED: Use rclpy.time.Duration instead of rclpy.duration.Duration
                    transform = self.tf_buffer.lookup_transform(
                        target_frame, 
                        source_frame, 
                        stamp, 
                        timeout=rclpy.time.Duration(seconds=0.5)
                    )
                    
                    # FIXED: Direct transform from camera to target frame
                    ps_map = do_transform_pose(ps_cam.pose, transform)
                    
                    self.get_logger().info(f"Transform successful: {source_frame} -> {target_frame}")
                    transform_success = True
                    break
                    
                except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
                    self.get_logger().debug(f"Transform failed {source_frame} -> {target_frame}: {e}")
                    continue
            
            if transform_success and ps_map is not None:
                # Store current frame detection
                current_frame_detections[marker_id] = {
                    'pose': ps_map,
                    'timestamp': current_timestamp
                }
                self.get_logger().info(f"Successfully processed marker {marker_id}")
            else:
                # Last resort: use camera frame directly (no transform)
                self.get_logger().warn(f"All transforms failed for marker {marker_id}, using camera frame directly")
                # FIXED: Keep original frame_id instead of changing to non-existent frame
                current_frame_detections[marker_id] = {
                    'pose': ps_cam.pose,
                    'timestamp': current_timestamp
                }
                self.get_logger().info(f"Using camera frame directly for marker {marker_id}")

        # Update tracking system
        self.get_logger().info(f"Updating tracking system with {len(current_frame_detections)} detections")
        self._update_marker_tracking(current_frame_detections, current_timestamp)
        
        # Publish all confirmed markers (persistent)
        self.get_logger().info("About to publish persistent results")
        self._publish_persistent_results(stamp)

    def _update_marker_tracking(self, current_detections, timestamp):
        """Update marker tracking system with current frame detections."""
        self.get_logger().info(f"Processing {len(current_detections)} current detections")
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

    def _diagnose_tf_tree(self, stamp):
        """Diagnose available TF frames and their connectivity."""
        # Only run this diagnostic occasionally to avoid spam
        if not hasattr(self, '_last_tf_diagnostic') or (time.time() - self._last_tf_diagnostic) > 10.0:
            self._last_tf_diagnostic = time.time()
            
            # Test common frame combinations
            test_frames = ["map", "odom", "base_link", "base_footprint", "camera_link", "camera_rgb_optical_frame"]
            available_frames = []
            
            for frame in test_frames:
                try:
                    # Try to get transform from this frame to itself (tests if frame exists)
                    # FIXED: Use rclpy.time.Duration instead of rclpy.duration.Duration
                    self.tf_buffer.lookup_transform(frame, frame, stamp, timeout=rclpy.time.Duration(seconds=0.1))
                    available_frames.append(frame)
                except:
                    pass
            
            self.get_logger().info(f"Available TF frames: {available_frames}")
            
            # Test specific transforms
            if self.image_frame_id in available_frames:
                for target in ["map", "base_link", "odom"]:
                    if target in available_frames:
                        try:
                            # FIXED: Use rclpy.time.Duration instead of rclpy.duration.Duration
                            self.tf_buffer.lookup_transform(target, self.image_frame_id, stamp, timeout=rclpy.time.Duration(seconds=0.1))
                            self.get_logger().info(f"✓ Transform available: {self.image_frame_id} -> {target}")
                        except:
                            self.get_logger().warn(f"✗ Transform failed: {self.image_frame_id} -> {target}")

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

    def _draw_marker_debug(self, img, corners, ids, rvecs, tvecs):
        """Draw red squares around detected markers with ID numbers."""
        if ids is None:
            return img
            
        # Draw marker corners and IDs
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        
        # Draw pose axes for each marker
        for i in range(len(ids)):
            cv2.drawFrameAxes(img, self.camera_K, self.camera_D, rvecs[i], tvecs[i], 0.05)
            
            # Draw red square around marker
            marker_corners = corners[i][0].astype(int)
            cv2.polylines(img, [marker_corners], True, (0, 0, 255), 3)  # Red square
            
            # Add marker ID text
            marker_id = ids[i][0]
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            
            # Draw white background for text
            cv2.rectangle(img, (center_x-20, center_y-15), (center_x+20, center_y+5), (255, 255, 255), -1)
            
            # Draw marker ID
            cv2.putText(img, f"ID:{marker_id}", (center_x-15, center_y+2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # Draw distance info
            distance = np.linalg.norm(tvecs[i])
            cv2.putText(img, f"d:{distance:.2f}m", (center_x-25, center_y+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        return img

    def _publish_debug_image(self, debug_img, stamp):
        """Publish debug image with marker annotations."""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header.stamp = stamp
            debug_msg.header.frame_id = self.image_frame_id
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publishing failed: {e}")

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
        
        # Publish visualization
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
