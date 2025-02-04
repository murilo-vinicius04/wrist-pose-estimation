#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D, Point
from cv_bridge import CvBridge
import threading
import time

class DepthWristSubscriber(Node):
    def __init__(self):
        super().__init__('depth_wrist_subscriber')
        self.bridge = CvBridge()

        # Subscrição ao tópico de imagem de profundidade
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Subscrição ao tópico de coordenadas 2D do pulso
        self.wrist_subscription = self.create_subscription(
            Pose2D,
            '/blazepose/right_wrist_2d',
            self.wrist_callback,
            10
        )

        # Variáveis para armazenar a última imagem de profundidade e seu encoding
        self.latest_depth_image = None
        self.depth_encoding = None
        self.depth_lock = threading.Lock()

        # Variável para armazenar a última posição válida do pulso
        self.last_valid_point = None
        self.point_lock = threading.Lock()

        # Publicador para publicar a posição 3D do pulso
        self.pub_wrist_3d = self.create_publisher(
            Point,
            '/blazepose/right_wrist_3d',
            10
        )

        # Timer para publicar periodicamente a última posição válida
        timer_period = 0.1  # segundos (10 Hz)
        self.timer = self.create_timer(timer_period, self.publish_last_point)

        # Variável para controlar a frequência dos logs
        self.last_log_time = time.time()
        self.log_interval = 1.0  # Segundos entre os logs de coordenadas

        self.get_logger().info("DepthWristSubscriber inicializado!")

    def depth_callback(self, msg):
        """Callback para a imagem de profundidade."""
        try:
            # Converte a mensagem ROS Image em um array NumPy
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Protege o acesso à imagem com uma trava
            with self.depth_lock:
                self.latest_depth_image = depth_image
                self.depth_encoding = msg.encoding

            self.get_logger().debug("Imagem de profundidade atualizada.")
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem de profundidade: {e}")

    def wrist_callback(self, msg):
        """Callback para as coordenadas 2D do pulso."""
        try:
            # Obtém as coordenadas 2D do pulso em milímetros
            u_mm = float(msg.x)  # Em milímetros
            v_mm = float(msg.y)  # Em milímetros
            self.get_logger().debug(f"Coordenadas 2D do pulso recebidas: (x={u_mm} mm, y={v_mm} mm)")

            # Converte as coordenadas de milímetros para metros
            u_m = u_mm / 1000.0
            v_m = v_mm / 1000.0

            # Acessa a última imagem de profundidade de forma segura
            with self.depth_lock:
                if self.latest_depth_image is None:
                    self.get_logger().warn("Imagem de profundidade ainda não recebida.")
                    return
                depth_image = self.latest_depth_image.copy()
                encoding = self.depth_encoding

            height, width = depth_image.shape[:2]

            # Verifica se as coordenadas estão dentro dos limites da imagem
            if u_mm < 0 or u_mm >= width or v_mm < 0 or v_mm >= height:
                # Removido o log de aviso
                return

            # Lê o valor de profundidade na posição (u, v)
            depth_value = depth_image[int(v_mm), int(u_mm)]

            # Ignora profundidade inválida (0.0)
            if depth_value == 0.0:
                self.get_logger().warn(f"Valor de profundidade inválido detectado em (x={u_mm} mm, y={v_mm} mm). Mantendo última posição válida.")
                return

            # Converte a profundidade para metros
            if encoding == '16UC1':
                depth_in_m = float(depth_value) / 1000.0  # Converte de milímetros para metros
            elif encoding == '32FC1':
                depth_in_m = float(depth_value)  # Assumindo que já está em metros
            else:
                self.get_logger().warn(f"Encoding de profundidade desconhecido: {encoding}")
                return

            # Atualiza os logs a cada intervalo definido
            current_time = time.time()
            if current_time - self.last_log_time >= self.log_interval:
                self.get_logger().info(f"Coordenadas 3D do pulso atualizadas: x={u_m:.3f} m, y={v_m:.3f} m, z={depth_in_m:.3f} m")
                self.last_log_time = current_time

            # Cria a mensagem Point com as coordenadas 3D do pulso em metros
            wrist_pose_3d = Point()
            wrist_pose_3d.x = u_m
            wrist_pose_3d.y = v_m
            wrist_pose_3d.z = depth_in_m

            # Atualiza a última posição válida
            with self.point_lock:
                self.last_valid_point = wrist_pose_3d

            self.get_logger().debug(f"Atualizada última posição válida do pulso: x={wrist_pose_3d.x:.3f} m, y={wrist_pose_3d.y:.3f} m, z={wrist_pose_3d.z:.3f} m")
        except Exception as e:
            self.get_logger().error(f"Erro ao processar as coordenadas do pulso: {e}")

    def publish_last_point(self):
        """Publica a última posição válida do pulso."""
        with self.point_lock:
            if self.last_valid_point is not None:
                self.pub_wrist_3d.publish(self.last_valid_point)
                self.get_logger().info(
                    f"Publicando última posição válida do pulso: "
                    f"x={self.last_valid_point.x:.3f} m, y={self.last_valid_point.y:.3f} m, z={self.last_valid_point.z:.3f} m"
                )
            else:
                self.get_logger().debug("Nenhuma posição válida para publicar ainda.")

def main(args=None):
    rclpy.init(args=args)
    node = DepthWristSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
