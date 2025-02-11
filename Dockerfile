# Base com ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop-full

# Define ambiente não interativo e timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

# Instala dependências do sistema
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

# Instala pip e dependências Python
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install --no-cache-dir mediapipe numpy opencv-python

# Cria o diretório do usuário e configura o ambiente
RUN mkdir -p /home/developer
WORKDIR /home/developer

# Clone e compilação do librealsense (essa camada será cacheada após a primeira execução)
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

# Instala a binding do pyrealsense2
RUN pip3 install pyrealsense2

RUN pip3 install transforms3d

# Configura o workspace do ROS
WORKDIR /home/developer/SPWAR_ws
# Copia o conteúdo do diretório "src" para o workspace
COPY ./src /home/developer/SPWAR_ws/src

# Troca o shell padrão para bash (para assegurar que o sourcing funcione corretamente)
SHELL ["/bin/bash", "-c"]

# Compila o workspace. Como essa etapa depende do conteúdo em "src", ela será reexecutada apenas se houver alterações.
RUN source /opt/ros/humble/setup.bash && \
    colcon build && \
    echo "Build do workspace concluído com sucesso!"

# Variáveis de ambiente do ROS e configurações gráficas
ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

# Comando padrão ao iniciar o container
CMD ["/bin/bash"]
