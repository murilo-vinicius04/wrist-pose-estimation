#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Mensagens ROS 2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

# Para converter imagens entre ROS <-> OpenCV
from cv_bridge import CvBridge

import cv2
import mediapipe as mp

class BlazePoseRightArm2DNode(Node):
    def __init__(self):
        super().__init__('blazepose_right_arm_2d_node')

        # Subscreve à imagem RGB
        self.sub_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publica Pose2D com o punho (wrist) do braço direito
        self.pub_right_wrist_2d = self.create_publisher(
            Pose2D,
            '/blazepose/right_wrist_2d',
            10
        )

        # Ferramenta de conversão Imagem <-> OpenCV
        self.bridge = CvBridge()

        # Inicializa MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # Índice do landmark de interesse (modelo BlazePose / MediaPipe)
        self.idx_wrist = mp.solutions.pose.PoseLandmark.RIGHT_WRIST.value

        self.get_logger().info("BlazePoseRightArm2DNode inicializado!")

    def image_callback(self, msg: Image):
        """Callback da imagem."""
        self.get_logger().debug("Callback da imagem acionado.")

        # Converte imagem ROS -> OpenCV
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Processa pose
        self.get_logger().debug("Processando pose com MediaPipe.")
        results = self.pose.process(frame_rgb)
        if not results.pose_landmarks:
            self.get_logger().info("Nenhuma pose detectada nesta imagem.")
            return

        self.get_logger().debug("Pose detectada, extraindo coordenadas 2D do pulso.")
        lm = results.pose_landmarks.landmark
        h, w, _ = frame_bgr.shape

        # Extrai as coordenadas 2D do pulso
        wrist_landmark = lm[self.idx_wrist]
        u = int(wrist_landmark.x * w)
        v = int(wrist_landmark.y * h)
        self.get_logger().debug(f"Coordenadas 2D do pulso: (x={u}, y={v})")

        # Cria a mensagem Pose2D
        wrist_pose_2d = Pose2D()
        wrist_pose_2d.x = float(u)
        wrist_pose_2d.y = float(v)
        wrist_pose_2d.theta = 0.0  # Como não há orientação 3D, definimos como 0

        # Publica as coordenadas 2D do pulso
        self.pub_right_wrist_2d.publish(wrist_pose_2d)
        self.get_logger().info(f"Publicando coordenadas 2D do pulso: x={wrist_pose_2d.x}, y={wrist_pose_2d.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BlazePoseRightArm2DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
