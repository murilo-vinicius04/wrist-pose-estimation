import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
import numpy as np
import tf_transformations

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # Inscreve-se no tópico que envia a posição 3D do pulso
        self.sub_wrist = self.create_subscription(
            PointStamped,
            '/blazepose/right_wrist_3d',
            self.wrist_callback,
            10
        )
        self.latest_wrist = None

        # Variáveis de estado da calibração
        self.calib_count = 0  # Indica em qual etapa da calibração estamos (0: origem, 1: eixo X, 2: eixo Y, 3: eixo Z)
        self.origin_cam = None
        self.x_axis_cam = None
        self.y_axis_cam = None
        self.z_axis_cam = None

        # Objeto para publicação de transformada estática
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.get_logger().info("Nó de calibração inicializado.")

        # Intervalo (em segundos) entre cada etapa da calibração
        self.step_interval = 5.0

        # Inicia o procedimento de calibração: o usuário deverá posicionar o pulso conforme a instrução
        self.get_logger().info(
            f"[PASSO 0] Posicione o pulso na posição ORIGEM. A calibração será executada automaticamente em {self.step_interval} segundos."
        )
        self.schedule_next_step()

    def wrist_callback(self, msg: PointStamped):
        # Atualiza a posição mais recente do pulso
        self.latest_wrist = msg.point

    def schedule_next_step(self):
        """
        Agenda a próxima etapa da calibração utilizando um timer que dispara apenas uma vez.
        """
        # Cria um timer que chamará o callback após 'step_interval' segundos
        self.timer = self.create_timer(self.step_interval, self.calibration_timer_callback)

    def calibration_timer_callback(self):
        # Cancela o timer para garantir que ele seja executado apenas uma vez (one-shot)
        self.timer.cancel()

        if self.latest_wrist is None:
            self.get_logger().warn("Nenhum dado do pulso recebido. Verifique o sensor e posicione o pulso corretamente.")
            # Reagenda a mesma etapa
            self.schedule_next_step()
            return

        # Executa a etapa de calibração conforme o contador
        if self.calib_count == 0:
            self.origin_cam = self.latest_wrist
            self.get_logger().info(
                f"[PASSO 1] Origem armazenada. Agora, mova a mão para a posição FRONTAL (eixo X). A calibração será executada em {self.step_interval} segundos."
            )
        elif self.calib_count == 1:
            self.x_axis_cam = self.latest_wrist
            self.get_logger().info(
                f"[PASSO 2] Eixo X armazenado. Agora, mova a mão para a posição LATERAL (eixo Y). A calibração será executada em {self.step_interval} segundos."
            )
        elif self.calib_count == 2:
            self.y_axis_cam = self.latest_wrist
            self.get_logger().info(
                f"[PASSO 3] Eixo Y armazenado. Agora, mova a mão para a posição VERTICAL (eixo Z). A calibração será executada em {self.step_interval} segundos."
            )
        elif self.calib_count == 3:
            self.z_axis_cam = self.latest_wrist
            # Após a última etapa, calcula e publica a transformação
            self.compute_and_broadcast_transform()
            self.get_logger().info("Calibração completa!")
            self.calib_count = 0  # Se desejar, pode reiniciar o processo aqui
            return
        else:
            self.get_logger().error("Estado de calibração inesperado.")
            self.calib_count = 0
            return

        # Incrementa o contador e agenda a próxima etapa
        self.calib_count += 1
        self.schedule_next_step()

    def compute_and_broadcast_transform(self):
        """
        Calcula a matriz de rotação e a translação a partir dos pontos de calibração
        e publica a transformação.
        """
        # Calcula os vetores base a partir das diferenças dos pontos
        u = np.array([
            self.x_axis_cam.x - self.origin_cam.x,
            self.x_axis_cam.y - self.origin_cam.y,
            self.x_axis_cam.z - self.origin_cam.z
        ])
        v = np.array([
            self.y_axis_cam.x - self.origin_cam.x,
            self.y_axis_cam.y - self.origin_cam.y,
            self.y_axis_cam.z - self.origin_cam.z
        ])
        w = np.array([
            self.z_axis_cam.x - self.origin_cam.x,
            self.z_axis_cam.y - self.origin_cam.y,
            self.z_axis_cam.z - self.origin_cam.z
        ])

        # Orthogonaliza os vetores utilizando o processo de Gram-Schmidt
        u = u / np.linalg.norm(u)
        v = v - np.dot(v, u) * u
        v = v / np.linalg.norm(v)
        w = w - np.dot(w, u) * u - np.dot(w, v) * v
        w = w / np.linalg.norm(w)

        # Cria a matriz de rotação com as colunas correspondentes aos vetores ortonormais
        R = np.column_stack((u, v, w))

        # Converte a matriz de rotação para um quaternion
        quat = tf_transformations.quaternion_from_matrix([
            [R[0, 0], R[0, 1], R[0, 2], 0],
            [R[1, 0], R[1, 1], R[1, 2], 0],
            [R[2, 0], R[2, 1], R[2, 2], 0],
            [0,       0,       0,       1]
        ])

        # Calcula a translação: a origem do sistema no referencial do robô
        rotated_origin = R.T @ np.array([self.origin_cam.x, self.origin_cam.y, self.origin_cam.z])
        trans = -rotated_origin

        # Prepara a mensagem de transformada
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = "base"
        transform.transform.translation.x = float(trans[0])
        transform.transform.translation.y = float(trans[1])
        transform.transform.translation.z = float(trans[2])
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        # Publica a transformada
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
