# Base completa com ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop-full

# Define o ambiente como não interativo
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

# Instalação de dependências básicas
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

# Instalação de bibliotecas Python
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install --no-cache-dir mediapipe numpy opencv-python

# Configuração do diretório de trabalho
WORKDIR /home/developer/SPWAR_ws

# Copia o conteúdo do workspace
COPY ./src /home/developer/SPWAR_ws/src

# Instalação do librealsense apenas se necessário
RUN if [ ! -d "/home/developer/librealsense" ]; then \
        git clone https://github.com/IntelRealSense/librealsense.git /home/developer/librealsense && \
        cd /home/developer/librealsense && \
        mkdir build && cd build && \
        cmake .. -DBUILD_EXAMPLES=true && \
        make -j$(nproc) && make install; \
    else \
        echo "librealsense já instalado, pulando compilação."; \
    fi
    
RUN pip3 install pyrealsense2

# Ambiente do ROS
ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

# Troca o shell padrão para bash
SHELL ["/bin/bash", "-c"]

# Compila o workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build && \
    echo "Build do workspace concluído com sucesso!"

# Comando padrão
CMD ["/bin/bash"]
