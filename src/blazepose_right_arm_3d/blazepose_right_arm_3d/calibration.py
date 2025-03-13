#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointStamped

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.get_logger().info("Nó de calibração inicializado!")
        # Inscrição no tópico do pulso
        self.latest_point = None
        self.subscription = self.create_subscription(
            PointStamped, '/mediapipe/wrist_3d', self.wrist_callback, 10)
        # Sequência de movimentos e pontos esperados (em cm)
        self.movements = [
            ("Origem", (0, 0, 0)),
            ("Direita", (10, 0, 0)),
            ("Cima", (0, 10, 0)),
            ("FRENTE", (0, 0, 10))
        ]
        self.calibration_index = 0
        self.calibration_data = []
        # Timer para coleta automática (a cada 3 segundos)
        self.timer = self.create_timer(3.0, self.capture_point_callback)

    def wrist_callback(self, msg):
        # Captura o último ponto recebido do pulso
        self.latest_point = msg

    def capture_point_callback(self):
        if self.calibration_index >= len(self.movements):
            self.get_logger().info("Coleta de calibração completa!")
            self.timer.cancel()
            # ...opcional: processamento adicional dos dados...
            return
        
        move_name, robot_point = self.movements[self.calibration_index]
        self.get_logger().info(f"Capturando ponto para: {move_name}")
        if self.latest_point is not None:
            camera_point = (
                self.latest_point.point.x,
                self.latest_point.point.y,
                self.latest_point.point.z
            )
            self.calibration_data.append({
                "move": move_name,
                "camera_point": camera_point,
                "robot_point": robot_point
            })
            self.get_logger().info(f"Ponto capturado: {camera_point}")
        else:
            self.get_logger().warn("Nenhum dado recebido do pulso.")
        self.calibration_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
