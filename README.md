# ROS 2 Simulation Workspace - SPWAR_ws

Este repositório contém um workspace para simulação e desenvolvimento de um robô utilizando o ROS 2 Humble e ferramentas como Gazebo e RViz. Inclui um ambiente Docker configurado para facilitar a replicação e execução.

## Estrutura do Repositório

- **`Dockerfile`**: Define a imagem Docker com todas as dependências necessárias.
- **`ros_run.sh`**: Script para executar o container Docker com suporte a GPU e servidor X11.
- **Workspace ROS**:
  - **URDF/Xacro**: Arquivos de descrição do robô.
  - **Meshes**: Modelos 3D do robô.
  - **Launch Files**: Arquivos de lançamento para o Gazebo e RViz.

## Dockerfile

```dockerfile
# Base completa com ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop-full

# Define o ambiente como não interativo
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

# Configuração de fuso horário e pacotes necessários
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    vim \
    python3-colcon-common-extensions \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-ros \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Configuração do workspace
RUN mkdir -p /home/developer/SPWAR_ws/src
WORKDIR /home/developer/SPWAR_ws

# Clona o repositório do GitHub, se necessário
RUN [ ! -d "/home/developer/SPWAR_ws/.git" ] && git clone https://github.com/MHC-CodeSmith/SPWAR_ws.git /home/developer/SPWAR_ws || echo "Repositório já clonado."

# Configuração do ambiente
ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

# Troca o shell padrão para bash
SHELL ["/bin/bash", "-c"]

# Constrói o workspace e verifica pacotes
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build && \
    dpkg -l | grep gazebo-ros && \
    echo "Verificação concluída com sucesso!"

# Comando padrão
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

# Executar o container com mapeamento de GPU, display e volumes
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PWD/../SPWAR_ws:/home/developer/SPWAR_ws" \
    -v ~/.ssh:/home/developer/.ssh:ro \
    "$IMAGE_NAME"

# Revogar acesso ao servidor X11 após sair do container
echo "Revogando acesso ao servidor X11..."
xhost -local:docker

# Torna o script executável
chmod +x ./ros_run.sh
```

## O que foi feito

1. **Configuração do Dockerfile**:
   - Criação de uma imagem baseada no `osrf/ros:humble-desktop-full`.
   - Instalação de pacotes essenciais, incluindo `rviz2`, `gazebo_ros`, `colcon`, e outros necessários para simulação e desenvolvimento no ROS 2 Humble.
   - Configuração do workspace `/home/developer/SPWAR_ws`.
   - Clonagem automática do repositório do GitHub durante o build da imagem.

2. **Script `ros_run.sh`**:
   - Configuração para iniciar o container Docker com suporte a GPU e integração com o servidor X11.
   - Mapeamento de volumes locais para persistência dos dados do workspace e acesso ao SSH.

3. **Workspace ROS**:
   - Criação de arquivos URDF para descrição do robô.
   - Configuração de arquivos de lançamento para simulação no Gazebo e visualização no RViz.

## Como usar

1. **Construir a imagem Docker**:
   ```bash
   docker build -t ros_spot:novo .
   ```

2. **Executar o container**:
   ```bash
   ./ros_run.sh
   ```

3. **Executar os arquivos de lançamento**:
   - Para visualizar no RViz:
     ```bash
     ros2 launch spot_description display.launch.py
     ```
   - Para simular no Gazebo:
     ```bash
     ros2 launch spot_description gazebo.launch.py
     ```

4. **Editar e desenvolver**:
   - Edite os arquivos no workspace `/home/developer/SPWAR_ws` e use `colcon build` para recompilar.
