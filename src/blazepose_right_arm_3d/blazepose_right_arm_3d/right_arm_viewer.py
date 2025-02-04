#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import mediapipe as mp

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RightArmViewer(Node):
    def __init__(self):
        super().__init__('right_arm_viewer_node')

        # Subscrição ao tópico de imagem
        self.sub_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Ferramenta para converter Imagem ROS <-> OpenCV
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

        # Índices dos landmarks do braço direito (BlazePose/MediaPipe)
        self.idx_shoulder = self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value  # 12
        self.idx_elbow = self.mp_pose.PoseLandmark.RIGHT_ELBOW.value        # 14
        self.idx_wrist = self.mp_pose.PoseLandmark.RIGHT_WRIST.value       # 16

        # Nome da janela para exibir resultados
        self.window_name = "Right Arm Landmarks"

        self.get_logger().info("Nó RightArmViewer inicializado!")

    def image_callback(self, msg: Image):
        """Callback chamado sempre que uma nova imagem chega no tópico /camera/camera/color/image_raw."""
        # Converte de ROS Image para OpenCV (BGR)
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        # Converte para RGB (MediaPipe trabalha melhor em RGB)
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Processa a pose usando MediaPipe
        results = self.pose.process(frame_rgb)

        # Se não encontrou landmarks, apenas mostra a imagem sem marcações
        if not results.pose_landmarks:
            cv2.imshow(self.window_name, frame_bgr)
            cv2.waitKey(1)
            return

        # Obtem o shape da imagem
        h, w, _ = frame_bgr.shape

        # Converte landmarks normalizados (0-1) para coordenadas em pixels
        lm = results.pose_landmarks.landmark
        shoulder = lm[self.idx_shoulder]
        elbow = lm[self.idx_elbow]
        wrist = lm[self.idx_wrist]

        # Calcula coordenadas em pixel
        shoulder_px = (int(shoulder.x * w), int(shoulder.y * h))
        elbow_px = (int(elbow.x * w), int(elbow.y * h))
        wrist_px = (int(wrist.x * w), int(wrist.y * h))

        # Desenha círculos nos pontos
        cv2.circle(frame_bgr, shoulder_px, 5, (0, 255, 0), -1)  # Verde
        cv2.circle(frame_bgr, elbow_px, 5, (0, 255, 0), -1)     # Verde
        cv2.circle(frame_bgr, wrist_px, 5, (0, 255, 0), -1)     # Verde

        # Desenha linhas conectando ombro->cotovelo->punho
        cv2.line(frame_bgr, shoulder_px, elbow_px, (0, 255, 0), 2)
        cv2.line(frame_bgr, elbow_px, wrist_px, (0, 255, 0), 2)

        # Exibe a imagem com os landmarks do braço direito
        cv2.imshow(self.window_name, frame_bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RightArmViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
