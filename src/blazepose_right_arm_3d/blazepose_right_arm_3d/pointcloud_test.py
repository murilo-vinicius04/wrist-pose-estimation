#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import geometry_msgs.msg
import mediapipe as mp

class ArmPoseEstimator(Node):
    def __init__(self):
        super().__init__('arm_pose_estimator')
        self.get_logger().info("Initializing Arm Pose Estimator...")

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Calibration flag and transformation from marker to camera frame
        self.calibrated = False
        self.calib_transform = None

        # Camera intrinsics (to be set from CameraInfo)
        self.camera_info_received = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Variables for temporal smoothing and jump filtering of wrist position
        self.filtered_point = None    # last valid wrist position (in marker frame)
        self.alpha = 0.5              # smoothing factor for exponential moving average
        self.change_threshold = 0.15  # maximum allowed jump (in meters) between frames

        # Subscribe to the camera info topic to get the intrinsics (for the aligned depth-to-color stream)
        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10)

        # Use message_filters to synchronize the color and depth images
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ats = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],
                                               queue_size=10, slop=0.1)
        self.ats.registerCallback(self.image_callback)

        # Initialize MediaPipe Pose for wrist detection
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5,
                                      min_tracking_confidence=0.5)
        self.get_logger().info("Arm Pose Estimator Initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        # Only update (and log) intrinsics once.
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics received.")

    def image_callback(self, color_msg: Image, depth_msg: Image):
        if not self.camera_info_received:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error("CV Bridge error: %s" % str(e))
            return

        # Run calibration using ArUco until a marker is detected
        if not self.calibrated:
            self.calibrate(color_image)
        else:
            self.process_pose(color_image, depth_image, color_msg.header)

    def calibrate(self, color_image):
        # Detect an ArUco marker to define the reference (calibration) frame.
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()  # use available constructor
        corners, ids, _ = aruco.detectMarkers(color_image, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            # Draw detected markers for visualization
            aruco.drawDetectedMarkers(color_image, corners, ids)

            # Define the marker size (in meters) – adjust as needed.
            marker_length = 0.2

            # Use the camera intrinsics obtained from CameraInfo
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length,
                                                               self.camera_matrix,
                                                               self.dist_coeffs)
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            R, _ = cv2.Rodrigues(rvec)
            transform = np.eye(4)
            transform[0:3, 0:3] = R
            transform[0:3, 3] = tvec
            self.calib_transform = transform
            self.calibrated = True
            self.get_logger().info("Calibration successful using ArUco marker.")
        else:
            self.get_logger().info("Calibration: No ArUco marker detected. Please hold the marker in view.")

        cv2.imshow("Calibration", color_image)
        cv2.waitKey(1)

    def process_pose(self, color_image, depth_image, header):
        # Convert the BGR image to RGB for MediaPipe processing.
        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks:
            # Use the right wrist landmark (MediaPipe index 16) as an example.
            landmark = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]

            # Check landmark visibility to filter out uncertain detections.
            if landmark.visibility < 0.6:
                self.get_logger().warn("Wrist landmark visibility low (%.2f). Skipping frame." % landmark.visibility)
                return

            h, w, _ = color_image.shape
            pixel_x = int(landmark.x * w)
            pixel_y = int(landmark.y * h)

            # Get the depth value at the wrist pixel.
            depth_mm = depth_image[pixel_y, pixel_x]
            if depth_mm == 0:
                self.get_logger().warn("Depth value at wrist is zero. Skipping this frame.")
                return
            depth = depth_mm / 1000.0  # Convert depth from mm to meters

            # Back-project the 2D pixel to a 3D point using the intrinsics.
            X = (pixel_x - self.cx) * depth / self.fx
            Y = (pixel_y - self.cy) * depth / self.fy
            Z = depth
            point_cam = np.array([X, Y, Z, 1]).reshape(4, 1)

            # Transform the 3D point from the camera frame to the calibration (marker) frame.
            T_inv = np.linalg.inv(self.calib_transform)
            point_marker = T_inv.dot(point_cam)
            new_point = point_marker[:3, 0]

            # Sanity check: if the new wrist position is absurdly far, skip the update.
            MAX_DISTANCE = 3.0  # adjust based on your expected scene scale (meters)
            if np.linalg.norm(new_point) > MAX_DISTANCE:
                self.get_logger().warn("New wrist position is too far (%.2f m). Skipping update." % np.linalg.norm(new_point))
                return

            # Filter out sudden jumps (likely misclassifications).
            if self.filtered_point is not None:
                diff = np.linalg.norm(new_point - self.filtered_point)
                if diff > self.change_threshold:
                    self.get_logger().warn("Large jump in wrist position detected (%.3f m). Ignoring update." % diff)
                    return

            # Apply exponential moving average filtering for smoothing.
            if self.filtered_point is None:
                self.filtered_point = new_point
            else:
                self.filtered_point = self.alpha * new_point + (1 - self.alpha) * self.filtered_point

            # Rebuild homogeneous coordinates for the filtered point.
            filtered_point_hom = np.array([self.filtered_point[0],
                                             self.filtered_point[1],
                                             self.filtered_point[2], 1]).reshape(4, 1)
            self.publish_tf(filtered_point_hom, header)

            # Draw a circle on the wrist in the visualization.
            cv2.circle(color_image, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
        else:
            self.get_logger().info("No pose landmarks detected.")

        cv2.imshow("Arm Pose", color_image)
        cv2.waitKey(1)

    def publish_tf(self, point_marker, header):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = header.stamp  # ROS2 stamps are already of type Time
        t.header.frame_id = "calibration_frame"
        t.child_frame_id = "wrist"
        t.transform.translation.x = float(point_marker[0, 0])
        t.transform.translation.y = float(point_marker[1, 0])
        t.transform.translation.z = float(point_marker[2, 0])
        # Publish an identity quaternion since orientation is not estimated here.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published wrist transform: [%.3f, %.3f, %.3f]" %
                               (point_marker[0, 0], point_marker[1, 0], point_marker[2, 0]))

def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down arm_pose_estimator node.")
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
