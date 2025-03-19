#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# Mensagens e transformações
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

class WristTransformer(Node):
    def __init__(self):
        super().__init__('wrist_transformer')
        
        # Configurações do tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscrição para a posição do pulso no frame da câmera
        self.wrist_sub = self.create_subscription(
            PointStamped,
            '/blazepose/right_wrist_3d',
            self.wrist_callback,
            10
        )
        
        # Publicador para a posição do pulso no frame do ArUco
        self.wrist_pub = self.create_publisher(
            PointStamped,
            '/mediapipe/wrist_3d_aruco',
            10
        )
        
        self.get_logger().info('Nó WristTransformer inicializado')
    
    def wrist_callback(self, msg):
        try:
            # Verifica se o frame do ArUco está disponível
            target_frame = 'aruco_marker_0'  # O ID do seu marcador
            
            # Usar o timestamp atual em vez do timestamp da mensagem
            # Isso evita o problema de extrapolação para o futuro
            transform_time = rclpy.time.Time()
            
            # Transforma a posição do pulso para o frame do ArUco
            # Criamos uma cópia da mensagem com o timestamp modificado
            msg_copy = PointStamped()
            msg_copy.header.frame_id = msg.header.frame_id
            msg_copy.header.stamp = transform_time.to_msg()
            msg_copy.point = msg.point
            
            wrist_aruco = self.tf_buffer.transform(
                msg_copy,
                target_frame,
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # Publica a nova posição transformada
            self.wrist_pub.publish(wrist_aruco)
            
            # Log para debug (agora como INFO para ser exibido)
            self.get_logger().info(
                f'Pulso transformado: x={wrist_aruco.point.x:.3f}, y={wrist_aruco.point.y:.3f}, z={wrist_aruco.point.z:.3f}'
            )
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f'Erro na transformação: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WristTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()