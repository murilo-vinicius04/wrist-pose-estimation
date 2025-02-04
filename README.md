
---

# ROS 2 Simulation Workspace - SPWAR_ws

Este repositório contém um workspace para simulação e desenvolvimento de um robô utilizando ROS 2 Humble, Gazebo e RViz. Agora com ambiente Docker otimizado.

## Estrutura do Repositório

- **Dockerfile**: Imagem Docker com as dependências necessárias e compilação otimizada do *librealsense*.
- **ros_run.sh**: Script para executar o container com suporte a GPU, X11 e dispositivos USB.
- **Workspace ROS**:
  - URDF/Xacro
  - Meshes
  - Launch Files

## Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

RUN apt-get update && apt-get install -y \
    sudo \
    git \
    wget \
    vim \
    python3-colcon-common-extensions \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-ros \
    ros-humble-gazebo-ros-pkgs \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    build-essential \
    libopencv-dev \
    libssl-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    cmake \
    v4l-utils \
    linux-headers-generic \
    bc \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install --no-cache-dir mediapipe numpy opencv-python

RUN mkdir -p /home/developer
WORKDIR /home/developer

RUN if [ ! -d "librealsense" ]; then \
        echo "Clonando e compilando librealsense..."; \
        git clone https://github.com/IntelRealSense/librealsense.git && \
        cd librealsense && mkdir build && cd build && \
        cmake .. -DBUILD_EXAMPLES=true && \
        make -j$(nproc) && \
        make install; \
    else \
        echo "librealsense já instalado, pulando compilação."; \
    fi

RUN pip3 install pyrealsense2

WORKDIR /home/developer/SPWAR_ws
COPY ./src /home/developer/SPWAR_ws/src

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash && \
    colcon build && \
    echo "Build do workspace concluído com sucesso!"

ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

CMD ["/bin/bash"]
```

## Script `ros_run.sh`

```bash
#!/bin/bash

# Permitir acesso ao servidor X11
echo "Permitindo acesso ao servidor X11..."
xhost +local:docker

# Nome do container e imagem
CONTAINER_NAME="ros_arm_sim"
IMAGE_NAME="ros_spot:novo"

# Verificar se a GPU está disponível
if ! command -v nvidia-smi &>/dev/null; then
    echo "NVIDIA GPU não detectada. Certifique-se de que os drivers estão instalados."
    exit 1
fi

# Lista de dispositivos de câmera esperados (por exemplo: /dev/video2 a /dev/video7)
camera_devices=()
for video in {2..7}; do
    device="/dev/video${video}"
    if [ -e "$device" ]; then
        camera_devices+=(--device="${device}:${device}")
    else
        echo "Aviso: dispositivo ${device} não encontrado. O nó da câmera pode não funcionar corretamente."
    fi
done

# Executar o container com mapeamento de GPU, display, volumes e dispositivos USB/câmera (se existirem)
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PWD/../SPWAR_ws:/home/developer/SPWAR_ws" \
    -v ~/.ssh:/home/developer/.ssh:ro \
    "${camera_devices[@]}" \
    "$IMAGE_NAME"

# Revogar acesso ao servidor X11 após sair do container
echo "Revogando acesso ao servidor X11..."
xhost -local:docker
```

## Como Usar

1. **Construir a imagem Docker:**
   ```bash
   docker build -t ros_spot:novo .
   ```

2. **Executar o container:**
   ```bash
   ./ros_run.sh
   ```

3. **Executar os arquivos de lançamento:**
   - **RViz (visualização do robô):**
     ```bash
     ros2 launch spot_description display.launch.py
     ```
   - **Gazebo (simulação):**
     ```bash
     ros2 launch spot_description gazebo.launch.py
     ```
   - **BlazePose e Visualização do braço direito 3D:**  
     Este lançamento inicia o driver da câmera RealSense (com os parâmetros configurados) juntamente com os nós responsáveis por processar a pose e visualizar o braço direito.
     ```bash
     ros2 launch blazepose_right_arm_3d blazepose_right_arm_launch.py
     ```

4. **Desenvolver e compilar:**
   ```bash
   colcon build --symlink-install
   ```

---