import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
import time

class HumanToRobotTF(Node):
    def __init__(self):
        super().__init__('human_to_robot_tf')

        self.get_logger().info("Iniciando nó HumanToRobotTF!")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Point,
            '/blazepose/right_wrist_3d',  # Tópico com as coordenadas do pulso humano
            self.wrist_callback,
            10
        )

        # Armazena a posição inicial do pulso no frame final (já transformado)
        self.initial_wrist_position_final = None  
        self.calibrated = False

        # Rotação de +50 graus (em radianos) para compensar a inclinação de -50°
        self.yaw_correction = np.radians(50.0)

        # Matriz de rotação em torno de Y (no sistema estilo ROS)
        self.rotation_y_matrix = np.array([
            [ np.cos(self.yaw_correction), 0, np.sin(self.yaw_correction) ],
            [ 0,                          1, 0                          ],
            [-np.sin(self.yaw_correction), 0, np.cos(self.yaw_correction)]
        ])

        # Matriz de reordenação (camera -> ROS)
        # Exemplo: se Xc=dir, Yc=cima, Zc=frente da câmera
        # e queremos Xr=frente, Yr=esquerda, Zr=cima no frame ROS.
        self.camera_to_ros = np.array([
            [0,  0,  1],   # Xr = +Zc
            [-1, 0,  0],   # Yr = -Xc
            [0,  1,  0]    # Zr = +Yc
        ])

        self.get_logger().info("Aguardando usuário se posicionar...")
        time.sleep(3)  # Tempo para o usuário se posicionar
        self.get_logger().info("Iniciando calibração do pulso!")

    def wrist_callback(self, msg):
        """Recebe a posição do pulso (em metros), transforma e publica a TF."""
        # 1) Convertemos a posição no frame da Câmera para numpy
        p_camera = np.array([msg.x, msg.y, msg.z])

        # 2) Reordenamos para o frame ROS
        p_ros_reorder = self.camera_to_ros.dot(p_camera)

        # 3) Aplicamos a rotação de +50° em torno de Y
        p_ros_corrected = self.rotation_y_matrix.dot(p_ros_reorder)

        # --- Calibração ---
        if not self.calibrated:
            # Guardamos a posição inicial já no frame final (reordenado + rotacionado)
            self.initial_wrist_position_final = p_ros_corrected
            self.calibrated = True
            self.get_logger().info(
                f"Posição inicial registrada (já no frame ROS+rot): "
                f"x={p_ros_corrected[0]:.3f}, y={p_ros_corrected[1]:.3f}, z={p_ros_corrected[2]:.3f}"
            )
            return

        # 4) Calculamos o delta no frame ROS final
        delta = p_ros_corrected - self.initial_wrist_position_final

        # Log do delta (para debug)
        self.get_logger().debug(
            f"Delta (ROS-frame): dx={delta[0]:.6f}, dy={delta[1]:.6f}, dz={delta[2]:.6f}"
        )

        # 5) Publicar a transformação TF usando o delta
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # Frame pai e filho
        t.header.frame_id = "human_wrist"      # Nome do frame "pai"
        t.child_frame_id = "arm0_link_wr0"     # Nome do frame "filho"

        t.transform.translation.x = delta[0]
        t.transform.translation.y = delta[1]
        t.transform.translation.z = delta[2]

        # Rotação identidade (apenas exemplo)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # Log final da TF
        self.get_logger().info(
            f"TF Publicado (ROS-frame): x={delta[0]:.3f}, y={delta[1]:.3f}, z={delta[2]:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = HumanToRobotTF()

    # Se quiser ver todos os logs de debug:
    # node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
