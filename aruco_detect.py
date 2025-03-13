import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

# Seleciona o dicionário de ArUco (exemplo: DICT_4X4_50)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Definir o tamanho real do marcador ArUco em metros
marker_size = 0.20

# Configurar RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Iniciar o pipeline
profile = pipeline.start(config)

# Obter os parâmetros intrínsecos da câmera
color_stream = profile.get_stream(rs.stream.color)
color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

# Converter os parâmetros intrínsecos para o formato da matriz de câmera do OpenCV
camera_matrix = np.array([
    [color_intrinsics.fx, 0, color_intrinsics.ppx],
    [0, color_intrinsics.fy, color_intrinsics.ppy],
    [0, 0, 1]
])

# Obter coeficientes de distorção da câmera
dist_coeffs = np.array([
    color_intrinsics.coeffs[0],  # k1
    color_intrinsics.coeffs[1],  # k2
    color_intrinsics.coeffs[2],  # p1
    color_intrinsics.coeffs[3],  # p2
    color_intrinsics.coeffs[4]   # k3
]).reshape(5, 1)

try:
    while True:
        # Aguarda por um conjunto de frames
        frames = pipeline.wait_for_frames()
        
        # Obter o frame de cor
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
            
        # Converter para numpy array (formato que o OpenCV usa)
        frame = np.asanyarray(color_frame.get_data())

        # Converte o frame para escala de cinza
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detecta os marcadores ArUco no frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Se algum marcador for detectado
        if ids is not None:
            # Desenha o contorno e o ID do marcador
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estima a pose para cada marcador
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
            
            # Desenha os eixos para cada marcador detectado
            for i in range(len(ids)):
                # Desenha o sistema de coordenadas 3D do marcador
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_size/2)
                
                # Converter vetor de Rodrigues para matriz de rotação e depois para ângulos de Euler
                rot = R.from_rotvec(rvecs[i][0])
                euler_angles = rot.as_euler('xyz', degrees=True)  # Converte para graus
                
                # Imprime os valores de posição e rotação
                print(f"Marcador ID {ids[i][0]}")
                print(f"  Posição (x,y,z): {tvecs[i][0]}")
                print(f"  Rotação (roll, pitch, yaw) em graus: {euler_angles}")
                print("------------------------")

        # Exibe o frame com os marcadores detectados e os eixos
        cv2.imshow("Deteccao de ArUco", frame)

        # Encerra o loop se a tecla 'q' for pressionada
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Parar o pipeline quando terminar
    pipeline.stop()
    cv2.destroyAllWindows()
