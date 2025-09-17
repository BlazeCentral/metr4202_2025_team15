#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# One public entrypoint, ROS2 node

def run_aruco_detector():
    # ===========================
    # Step 0: Imports & setup
    # ===========================
    import math  # math utilities (atan2 etc)
    from collections import deque, defaultdict  # ring buffers, dicts
    from typing import Dict, Deque, Tuple  # type annotations

    import rclpy  # ROS2 Python client
    from rclpy.node import Node  # base Node class
    from rclpy.time import Time  # ROS time wrapper
    from rclpy.duration import Duration  # ROS duration wrapper

    import numpy as np  # arrays and matrix math
    import cv2  # OpenCV and ArUco tools
    from cv_bridge import CvBridge  # ROS Image ↔ OpenCV bridge

    from sensor_msgs.msg import Image, CameraInfo  # camera topics
    from geometry_msgs.msg import Pose, PoseStamped, PoseArray  # pose messages
    from visualization_msgs.msg import Marker, MarkerArray  # RViz markers

    import tf2_ros  # TF buffer and listener
    from tf2_ros import TransformException  # TF exceptions
    from tf2_geometry_msgs import do_transform_pose  # pose transform helper
    from geometry_msgs.msg import TransformStamped  # transform message type

    # ===========================
    # Step 0.1: Node + parameters
    # ===========================
    rclpy.init()  # initialise ROS2 client
    node = Node("aruco_detect_publish")  # create node instance

    # Declare parameters with defaults
    dict_name = node.declare_parameter("dictionary", "DICT_6X6_250").get_parameter_value().string_value  # ArUco set
    marker_size_m = node.declare_parameter("marker_size_m", 0.10).get_parameter_value().double_value  # 100 mm size
    reproj_px_max = node.declare_parameter("reproj_error_thresh_px", 3.0).get_parameter_value().double_value  # px gate
    window_N = node.declare_parameter("smoothing_window", 5).get_parameter_value().integer_value  # track length
    min_hits = node.declare_parameter("min_hits_for_publish", 2).get_parameter_value().integer_value  # confidence gate
    max_age_s = node.declare_parameter("max_track_age_sec", 2.0).get_parameter_value().double_value  # staleness gate

    # ===========================
    # Step 1: Inputs (Image/Info/TF)
    # ===========================
    bridge = CvBridge()  # convert ROS↔OpenCV frames
    camera_K = None  # intrinsic matrix placeholder
    camera_D = None  # distortion coefficients placeholder
    image_frame_id = None  # camera frame id cache

    tf_buffer = tf2_ros.Buffer(node.get_clock())  # TF query buffer
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)  # TF subscriber

    targets_pub = node.create_publisher(PoseArray, "/targets", 10)  # map-frame poses
    viz_pub = node.create_publisher(MarkerArray, "/targets_viz", 10)  # optional markers

    tracks: Dict[int, Deque[Tuple[np.ndarray, float, Time]]] = defaultdict(lambda: deque(maxlen=window_N))  # per-ID history

    # Build ArUco dictionary and params
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))  # dictionary select
    aruco_params = cv2.aruco.DetectorParameters_create()  # default detector params

    # Precompute square’s object points (3D)
    half = marker_size_m / 2.0  # half side length (meters)
    obj_pts = np.array([[-half,  half, 0.0],  # top-left corner (3D)
                        [ half,  half, 0.0],  # top-right corner (3D)
                        [ half, -half, 0.0],  # bottom-right corner (3D)
                        [-half, -half, 0.0]], # bottom-left corner (3D)
                       dtype=np.float32)  # use float32 for OpenCV

    # ===========================
    # Step 1.1: /camera/camera_info sub
    # ===========================
    def on_camera_info(msg: CameraInfo):
        nonlocal camera_K, camera_D, image_frame_id  # capture outer vars
        camera_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)  # set intrinsics matrix
        camera_D = np.array(msg.d, dtype=np.float64).reshape(-1,)  # set distortion coefficients
        image_frame_id = msg.header.frame_id or "camera_link"  # cache frame id
    node.create_subscription(CameraInfo, "/camera/camera_info", on_camera_info, 10)  # subscribe to intrinsics

    # ===========================
    # Step 1.2: /camera/image_raw sub
    # ===========================
    def on_image(msg: Image):
        # ------------------------------------------
        # #### STEP 2: Preprocess (grayscale) ####
        # ------------------------------------------
        stamp = Time.from_msg(msg.header.stamp)  # capture image timestamp
        cam_frame = msg.header.frame_id or image_frame_id or "camera_link"  # resolve camera frame
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # convert ROS→OpenCV
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)  # convert to grayscale

        # ------------------------------------------
        # #### STEP 3: Detect ArUco markers ####
        # ------------------------------------------
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)  # run detector
        if ids is None or len(ids) == 0:  # check if any detected
            # Publish explicit empty PoseArray
            pa = PoseArray()  # create PoseArray message
            pa.header.stamp = msg.header.stamp  # copy timestamp
            pa.header.frame_id = "map"  # still in map frame
            targets_pub.publish(pa)  # publish empty array
            return  # nothing more this frame

        ids = ids.flatten().tolist()  # flatten id array to list

        # Prepare per-frame bookkeeping
        confident_ids = []  # list of IDs considered confident
        for i, marker_id in enumerate(ids):  # iterate detections
            image_pts = corners[i].reshape(-1, 2).astype(np.float32)  # 4×2 pixel corners

            # ------------------------------------------
            # #### STEP 4: Pose in camera frame ####
            # ------------------------------------------
            if camera_K is None:  # ensure intrinsics available
                continue  # skip without intrinsics
            # Use IPPE_SQUARE for planar square
            ok, rvec, tvec = cv2.solvePnP(obj_pts, image_pts, camera_K,
                                          camera_D if camera_D is not None else None,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)  # estimate pose
            if not ok:  # check if PnP succeeded
                continue  # skip failed estimate

            # ------------------------------------------
            # #### STEP 5: Reprojection quality gate ####
            # ------------------------------------------
            proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_K,
                                        camera_D if camera_D is not None else None)  # reproject corners
            proj = proj.reshape(-1, 2)  # Nx2 pixel points
            reproj_err = float(np.linalg.norm(proj - image_pts, axis=1).mean())  # mean pixel error
            if reproj_err > reproj_px_max:  # compare to threshold
                continue  # discard poor estimate

            # ------------------------------------------
            # #### STEP 6: Transform to map frame ####
            # ------------------------------------------
            # Build PoseStamped in camera frame
            # Convert rvec (Rodrigues) to rotation matrix
            R, _ = cv2.Rodrigues(rvec)  # rotation 3×3 matrix
            yaw = math.atan2(R[1, 0], R[0, 0])  # approximate yaw angle
            # Convert yaw to quaternion (Z-rotation)
            qz = math.sin(yaw * 0.5)  # quaternion z component
            qw = math.cos(yaw * 0.5)  # quaternion w component

            ps_cam = PoseStamped()  # create camera-frame pose
            ps_cam.header.stamp = msg.header.stamp  # copy timestamp
            ps_cam.header.frame_id = cam_frame  # source frame id
            ps_cam.pose.position.x = float(tvec[0])  # camera X position
            ps_cam.pose.position.y = float(tvec[1])  # camera Y position
            ps_cam.pose.position.z = float(tvec[2])  # camera Z position
            ps_cam.pose.orientation.x = 0.0  # only yaw used
            ps_cam.pose.orientation.y = 0.0  # only yaw used
            ps_cam.pose.orientation.z = qz  # yaw quaternion z
            ps_cam.pose.orientation.w = qw  # yaw quaternion w

            try:
                # Lookup camera->map transform at time
                tf_map_from_cam: TransformStamped = tf_buffer.lookup_transform(
                    "map", cam_frame, stamp, timeout=Duration(seconds=0.2)
                )  # get transform chain
            except TransformException as e:
                node.get_logger().debug(f"TF lookup failed: {e}")  # log transform failure
                continue  # skip this detection

            ps_map = do_transform_pose(ps_cam, tf_map_from_cam)  # apply transform to map

            # ------------------------------------------
            # #### STEP 7: Update per-ID tracks ####
            # ------------------------------------------
            # Extract position vector and yaw from pose
            px = ps_map.pose.position.x  # map X position
            py = ps_map.pose.position.y  # map Y position
            pz = ps_map.pose.position.z  # map Z position
            # Recover yaw from quaternion
            qx = ps_map.pose.orientation.x  # quat x component
            qy = ps_map.pose.orientation.y  # quat y component
            qz = ps_map.pose.orientation.z  # quat z component
            qw = ps_map.pose.orientation.w  # quat w component
            # Standard yaw extraction formula
            yaw_map = math.atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz))  # compute yaw

            # Append to ring buffer for this ID
            tracks[marker_id].append((np.array([px, py, pz], dtype=np.float64),
                                      yaw_map, stamp))  # push track sample
            confident_ids.append(marker_id)  # tentatively confident

        # ===========================
        # Step 8: Build outputs
        # ===========================
        # #### STEP 8.1: PoseArray (map) ####
        pose_array = PoseArray()  # create PoseArray message
        pose_array.header.stamp = msg.header.stamp  # frame timestamp
        pose_array.header.frame_id = "map"  # global map frame

        # #### STEP 8.2: MarkerArray (RViz) ####
        marker_array = MarkerArray()  # create marker container
        marker_seq = 0  # incremental marker ID

        now = node.get_clock().now()  # current ROS time
        # Smooth and output only confident, fresh tracks
        for marker_id in list(set(confident_ids)):  # unique confident ids
            buf = tracks.get(marker_id, None)  # retrieve ring buffer
            if not buf:  # handle missing buffer
                continue  # skip if no data
            # Prune stale entries by age
            # ===========================
            # Step 10.1: Prune stale tracks
            # ===========================
            pruned = deque(maxlen=buf.maxlen)  # new pruned buffer
            for (xyz, yaw_m, stmp) in buf:  # iterate samples
                if (now - stmp).nanoseconds * 1e-9 < max_age_s:  # age check
                    pruned.append((xyz, yaw_m, stmp))  # keep sample
            tracks[marker_id] = pruned  # overwrite with pruned
            if len(pruned) < min_hits:  # require minimum hits
                continue  # not confident yet

            # ===========================
            # Step 8.0: Temporal smoothing
            # ===========================
            xs = np.array([e[0] for e in pruned])  # collect positions
            yaws = np.array([e[1] for e in pruned])  # collect yaws
            pos_mean = xs.mean(axis=0)  # average position
            yaw_mean = math.atan2(np.sin(yaws).mean(),
                                  np.cos(yaws).mean())  # average yaw

            # Build smoothed Pose in map
            smoothed = Pose()  # create Pose object
            smoothed.position.x = float(pos_mean[0])  # set X
            smoothed.position.y = float(pos_mean[1])  # set Y
            smoothed.position.z = float(pos_mean[2])  # set Z
            # Yaw to quaternion (Z-only)
            qz = math.sin(yaw_mean * 0.5)  # compute quat z
            qw = math.cos(yaw_mean * 0.5)  # compute quat w
            smoothed.orientation.x = 0.0  # no roll/pitch here
            smoothed.orientation.y = 0.0  # no roll/pitch here
            smoothed.orientation.z = qz  # set quaternion z
            smoothed.orientation.w = qw  # set quaternion w

            pose_array.poses.append(smoothed)  # append to outputs

            # Build arrow marker (nice-to-have)
            m = Marker()  # RViz marker object
            m.header = pose_array.header  # reuse header
            m.ns = "aruco_targets"  # marker namespace
            m.id = marker_seq  # unique marker id
            marker_seq += 1  # increment for next
            m.type = Marker.ARROW  # draw arrow glyph
            m.action = Marker.ADD  # add to scene
            m.pose = smoothed  # place at smoothed pose
            m.scale.x = 0.12  # arrow length (m)
            m.scale.y = 0.025  # arrow width (m)
            m.scale.z = 0.025  # arrow height (m)
            m.color.r = 0.0  # green arrow red=0
            m.color.g = 1.0  # green arrow green=1
            m.color.b = 0.0  # green arrow blue=0
            m.color.a = 1.0  # fully opaque arrow
            marker_array.markers.append(m)  # store marker

            # Add text label with marker ID
            t = Marker()  # text marker object
            t.header = pose_array.header  # reuse header
            t.ns = "aruco_labels"  # text namespace
            t.id = marker_seq  # unique marker id
            marker_seq += 1  # increment for next
            t.type = Marker.TEXT_VIEW_FACING  # billboard text
            t.action = Marker.ADD  # add to scene
            t.pose = smoothed  # same position pose
            t.pose.position.z += 0.10  # lift label up
            t.scale.z = 0.12  # text height (m)
            t.color.r = 1.0  # white text red=1
            t.color.g = 1.0  # white text green=1
            t.color.b = 1.0  # white text blue=1
            t.color.a = 0.9  # slightly transparent
            t.text = f"ID {marker_id}"  # display marker ID
            marker_array.markers.append(t)  # store label

        # ===========================
        # Step 9: Publish results
        # ===========================
        if len(pose_array.poses) == 0:  # check empty result
            # 9.3 Publish empty PoseArray
            empty = PoseArray()  # create empty array
            empty.header = pose_array.header  # copy header
            targets_pub.publish(empty)  # publish empty result
        else:
            # 9.1 Publish PoseArray (map)
            targets_pub.publish(pose_array)  # publish all targets
            # 9.2 Publish MarkerArray (viz)
            viz_pub.publish(marker_array)  # publish RViz markers

        # ===========================
        # Step 10.2: Log frame stats
        # ===========================
        node.get_logger().info(
            f"targets={len(pose_array.poses)} tracks={len(tracks)}"  # brief status line
        )  # simple per-frame feedback

    node.create_subscription(Image, "/camera/image_raw", on_image, 10)  # subscribe to images

    # ===========================
    # Step 11: Spin the node
    # ===========================
    rclpy.spin(node)  # run callbacks until shutdown
    node.destroy_node()  # cleanly destroy node
    rclpy.shutdown()  # shutdown ROS2 client


# Allow running as a script
if __name__ == "__main__":
    run_aruco_detector()  # start ArUco detector
