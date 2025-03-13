#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

from collections import deque

class MediapipeWrist3DNode(Node):
    def __init__(self):
        super().__init__('mediapipe_wrist_3d_node')

        # Inscrição na imagem de cor da RealSense
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription  # evita aviso de variável não utilizada

        # Publicador para a posição 3D do wrist
        self.pub_wrist_3d = self.create_publisher(PointStamped, '/mediapipe/wrist_3d', 10)

        # Inicializa a conversão entre ROS e OpenCV
        self.bridge = CvBridge()

        # Inicializa o MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # Índice do landmark do punho direito
        self.idx_wrist = self.mp_pose.PoseLandmark.RIGHT_WRIST.value

        # Buffer para suavização (opcional)
        self.wrist_history = deque(maxlen=5)

        self.get_logger().info("MediapipeWrist3DNode inicializado.")

    def image_callback(self, msg: Image):
        try:
            # Converte a imagem ROS para OpenCV (BGR)
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter a imagem: {e}")
            return

        # Converte BGR para RGB para o MediaPipe
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Processa a imagem com o MediaPipe Pose
        results = self.pose.process(frame_rgb)

        if not results.pose_landmarks:
            self.get_logger().info("Nenhuma pose detectada.")
            return

        # Tenta usar os world landmarks (mais precisos para coordenadas 3D)
        if results.pose_world_landmarks:
            wrist_landmark = results.pose_world_landmarks.landmark[self.idx_wrist]
            # As coordenadas já estão em metros
            wrist_3d = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])
        else:
            # Fallback: usa os landmarks normalizados (não ideal para 3D)
            self.get_logger().warn("World landmarks não disponíveis, usando landmarks normalizados!")
            wrist_landmark = results.pose_landmarks.landmark[self.idx_wrist]
            wrist_3d = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])

        # Suaviza a posição usando um histórico (opcional)
        self.wrist_history.append(wrist_3d)
        smoothed_wrist = np.mean(self.wrist_history, axis=0)

        # Prepara a mensagem PointStamped com a posição 3D do punho (na frame da câmera)
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera_color_optical_frame"
        point_msg.point.x = float(smoothed_wrist[0])
        point_msg.point.y = float(smoothed_wrist[1])
        point_msg.point.z = float(smoothed_wrist[2])

        self.pub_wrist_3d.publish(point_msg)

        self.get_logger().info(
            f"Wrist 3D publicado: x={smoothed_wrist[0]:.3f}, y={smoothed_wrist[1]:.3f}, z={smoothed_wrist[2]:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MediapipeWrist3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
