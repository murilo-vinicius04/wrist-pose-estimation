#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D, PointStamped

from cv_bridge import CvBridge
import cv2
import mediapipe as mp

import pyrealsense2 as rs
import numpy as np

from message_filters import ApproximateTimeSynchronizer, Subscriber
from collections import deque


class BlazePoseRightArm3DNode(Node):
    def __init__(self):
        super().__init__('blazepose_right_arm_3d_node')

        # --- 1. Initialize RealSense pipeline for depth ---
        # --- REMOVE THIS SECTION ---
        # self.pipeline = rs.pipeline()
        # self.config = rs.config()
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # self.get_logger().info("Starting RealSense pipeline...")
        # self.profile = self.pipeline.start(self.config)
        # self.depth_intrinsics = (
        #     self.profile.get_stream(rs.stream.depth)
        #     .as_video_stream_profile()
        #     .get_intrinsics()
        # )
        # self.get_logger().info("RealSense pipeline started with depth intrinsics.")

        # --- 2. Subscribers for both color and depth images ---
        self.sub_color = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')  # Aligned depth

        # Subscribe to the camera intrinsics
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',  # Intrínsecos da câmera de cor!
            self.camera_info_callback,
            10
        )
        self.color_intrinsics = None  # Renomeie a variável

        # Use an Approximate Time Synchronizer to match frames
        self.ts = ApproximateTimeSynchronizer([self.sub_color, self.sub_depth], queue_size=10, slop=0.03)  # Mais rigoroso!
        self.ts.registerCallback(self.image_depth_callback)

        # --- 3. Publishers: (A) 2D Pose2D for the wrist, (B) 3D PointStamped for the wrist ---
        self.pub_right_wrist_2d = self.create_publisher(
            Pose2D,
            '/blazepose/right_wrist_2d',
            10
        )

        self.pub_right_wrist_3d = self.create_publisher(
            PointStamped,
            '/blazepose/right_wrist_3d',
            10
        )

        # --- 4. Setup for OpenCV <-> ROS image conversion ---
        self.bridge = CvBridge()

        # --- 5. Initialize MediaPipe Pose (BlazePose) ---
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # BlazePose index for the right wrist landmark
        self.idx_wrist = self.mp_pose.PoseLandmark.RIGHT_WRIST.value

        self.wrist_history = deque(maxlen=7)  # Increased from 3 to 7
        
        # Add a buffer to store depth history
        self.depth_history = deque(maxlen=10)
        
        # Constants for outlier filtering
        self.MAX_DEPTH_CHANGE = 0.3  # meters - maximum allowed change between frames

        self.get_logger().info("BlazePoseRightArm3DNode initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        """Receives the color camera intrinsics from /camera/camera/color/camera_info"""
        self.color_intrinsics = rs.intrinsics()
        self.color_intrinsics.width = msg.width
        self.color_intrinsics.height = msg.height
        self.color_intrinsics.ppx = msg.k[2]  # Principal point X (cx)
        self.color_intrinsics.ppy = msg.k[5]  # Principal point Y (cy)
        self.color_intrinsics.fx = msg.k[0]   # Focal length X (fx)
        self.color_intrinsics.fy = msg.k[4]   # Focal length Y (fy)
        self.color_intrinsics.model = rs.distortion.none
        self.color_intrinsics.coeffs = list(msg.d)  # Usa os coeficientes reais da câmera

        self.get_logger().info("Color intrinsics received.")

    def image_depth_callback(self, color_msg: Image, depth_msg: Image):
        """Receives synchronized color and depth images, runs MediaPipe to find wrist, and then deprojects to 3D."""
        # Verifique se os intrínsecos já foram recebidos
        if self.color_intrinsics is None:
            self.get_logger().warn("Intrínsecos não inicializados. Ignorando frame.")
            return

        # Convert ROS -> OpenCV BGR
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")
            return

        # Convert BGR to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Process with MediaPipe Pose
        results = self.pose.process(frame_rgb)
        if not results.pose_landmarks:
            self.get_logger().warn("Nenhuma pose detectada! Verifique a câmera e o posicionamento.")
            return

        # Extract wrist coordinates in pixel space
        landmarks = results.pose_landmarks.landmark
        color_h, color_w, _ = frame_bgr.shape  # Para a imagem de cor
        wrist_landmark = landmarks[self.idx_wrist]

        # 2D pixel coordinates (u, v)
        u = int(wrist_landmark.x * color_w)
        v = int(wrist_landmark.y * color_h)

        self.get_logger().info(f"Pulso detectado: u={u}, v={v}")

        # Publish 2D wrist coordinates (Pose2D)
        wrist_pose_2d = Pose2D()
        wrist_pose_2d.x = float(u)
        wrist_pose_2d.y = float(v)
        wrist_pose_2d.theta = 0.0
        self.pub_right_wrist_2d.publish(wrist_pose_2d)

        # Convert ROS -> OpenCV depth image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
            return

        # Clamp u, v within image dimensions
        depth_h, depth_w = depth_image.shape  # Para a imagem de profundidade
        u = int(np.clip(u, 0, depth_w - 1))
        v = int(np.clip(v, 0, depth_h - 1))

        # Increase the search window to 7x7 pixels
        window_size = 7
        half_window = window_size // 2
        
        u_min = max(0, u - half_window)
        u_max = min(depth_w, u + half_window + 1)
        v_min = max(0, v - half_window)
        v_max = min(depth_h, v + half_window + 1)
        
        depth_window = depth_image[v_min:v_max, u_min:u_max]

        # Filter valid values (within practical operating range for wrist tracking)
        valid_pixels = depth_window[(depth_window >= 300) & (depth_window <= 1500)]
        
        if valid_pixels.size == 0:
            self.get_logger().warn(f"No valid depth found at ({u},{v}), skipping frame...")
            return

        # Use median instead of mean for robustness to outliers
        if depth_msg.encoding == '16UC1':
            depth_value = np.median(valid_pixels) * 0.001  # mm to meters
        elif depth_msg.encoding == '32FC1':
            depth_value = np.median(valid_pixels)  # already in meters
        else:
            self.get_logger().error(f"Unsupported format: {depth_msg.encoding}")
            return

        # Log the raw depth value
        self.get_logger().info(f"Raw Depth (m): {depth_value:.3f}")
        
        # Check if depth is within reasonable limits
        if depth_value <= 0.05 or depth_value > 3.0:
            self.get_logger().warn(f"Depth out of bounds ({depth_value:.2f}m), skipping...")
            return
        
        # Temporal filter: check for large jumps in depth
        if len(self.depth_history) > 0:
            last_depth = self.depth_history[-1]
            if abs(depth_value - last_depth) > self.MAX_DEPTH_CHANGE:
                self.get_logger().warn(f"Depth jump detected: {last_depth:.2f}m -> {depth_value:.2f}m")
                
                # Option 2: Limit the maximum variation (more gradual)
                if depth_value > last_depth:
                    depth_value = last_depth + self.MAX_DEPTH_CHANGE
                else:
                    depth_value = last_depth - self.MAX_DEPTH_CHANGE
                    
                self.get_logger().info(f"Depth adjusted to: {depth_value:.3f}m")
        
        # Store the depth value in the history buffer
        self.depth_history.append(depth_value)
        
        # Deproject the 2D pixel + depth to 3D coordinates
        wrist_3d = rs.rs2_deproject_pixel_to_point(
            self.color_intrinsics,
            [u, v],
            depth_value
        )

        # Add the new position to the history buffer
        self.wrist_history.append(wrist_3d)

        # Compute the median of the last positions for each coordinate X, Y, Z
        wrist_array = np.array(self.wrist_history)
        wrist_smoothed = np.median(wrist_array, axis=0)
        
        # Publish the 3D coordinate as PointStamped
        wrist_point_msg = PointStamped()
        wrist_point_msg.header.stamp = self.get_clock().now().to_msg()
        wrist_point_msg.header.frame_id = "camera_color_optical_frame"  # Correto após alinhamento
        wrist_point_msg.point.x = float(wrist_smoothed[0])
        wrist_point_msg.point.y = float(wrist_smoaothed[1])
        wrist_point_msg.point.z = float(wrist_smoothed[2])

        self.pub_right_wrist_3d.publish(wrist_point_msg)

        # Log info
        self.get_logger().info(
            f"Right wrist 2D: (u={u}, v={v}), depth={depth_value:.3f} m -> 3D: "
            f"X={wrist_point_msg.point.x:.3f}, "
            f"Y={wrist_point_msg.point.y:.3f}, "
            f"Z={wrist_point_msg.point.z:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BlazePoseRightArm3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
