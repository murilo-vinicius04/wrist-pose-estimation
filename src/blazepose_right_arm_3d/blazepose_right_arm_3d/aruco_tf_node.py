#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ArucoTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        
        # Criar o TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Criar o CvBridge para converter imagens
        self.bridge = CvBridge()
        
        # Parâmetros para ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.marker_size = 0.20  # tamanho do marcador em metros
        
        # Variáveis para armazenar informações da câmera
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Subscrever ao tópico de informações da câmera
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Subscrever ao tópico de imagem
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Timer para executar a detecção periodicamente
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        self.get_logger().info('Nó ArucoTFBroadcaster inicializado')
    
    def camera_info_callback(self, msg):
        # Extrair a matriz de câmera
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        
        # Extrair os coeficientes de distorção
        self.dist_coeffs = np.array(msg.d)
        
        # Informar que recebemos os parâmetros da câmera
        self.get_logger().info('Parâmetros da câmera recebidos')
        
        # Em vez de destruir, vamos simplesmente cancelar a inscrição
        self.camera_info_sub = None
    
    def image_callback(self, msg):
        # Verificar se temos os parâmetros da câmera
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Parâmetros da câmera ainda não recebidos')
            return
            
        try:
            # Converter a imagem ROS para OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Converter para escala de cinza
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detectar os marcadores ArUco
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            # Se algum marcador for detectado
            if ids is not None:
                # Estimar a pose para cada marcador
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, 
                                                                  self.camera_matrix, self.dist_coeffs)
                
                # Para cada marcador detectado
                for i in range(len(ids)):
                    # Publicar a transformação
                    self.publish_transform(ids[i][0], rvecs[i], tvecs[i], msg.header.stamp)
                    
        except Exception as e:
            self.get_logger().error(f'Erro no processamento da imagem: {e}')
    
    def publish_transform(self, marker_id, rvec, tvec, timestamp):
        # Criar mensagem de transformação
        transform_msg = TransformStamped()
        
        # Preencher o cabeçalho
        transform_msg.header.stamp = timestamp
        transform_msg.header.frame_id = 'camera_color_optical_frame'  # Frame pai (câmera)
        transform_msg.child_frame_id = f'aruco_marker_{marker_id}'    # Frame filho (ArUco)
        
        # Preencher translação (posição)
        transform_msg.transform.translation.x = float(tvec[0][0])
        transform_msg.transform.translation.y = float(tvec[0][1])
        transform_msg.transform.translation.z = float(tvec[0][2])
        
        # Converter vetor de Rodrigues para quaternion
        rot = R.from_rotvec(rvec[0])
        quat = rot.as_quat()  # Formato [x, y, z, w]
        
        # Preencher rotação (orientação)
        transform_msg.transform.rotation.x = float(quat[0])
        transform_msg.transform.rotation.y = float(quat[1])
        transform_msg.transform.rotation.z = float(quat[2])
        transform_msg.transform.rotation.w = float(quat[3])
        
        # Publicar a transformação
        self.tf_broadcaster.sendTransform(transform_msg)
        
        # Log para debug
        self.get_logger().info(f'Publicada transform para aruco_marker_{marker_id}')
    
    def timer_callback(self):
        pass  # O timer_callback não é mais necessário

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()