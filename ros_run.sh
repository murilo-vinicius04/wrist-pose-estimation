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

# Executar o container com mapeamento de GPU, display, volumes e dispositivos USB
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PWD/../SPWAR_ws:/home/developer/SPWAR_ws" \
    -v ~/.ssh:/home/developer/.ssh:ro \
    --device=/dev/video2:/dev/video2 \
    --device=/dev/video3:/dev/video3 \
    --device=/dev/video4:/dev/video4 \
    --device=/dev/video5:/dev/video5 \
    --device=/dev/video6:/dev/video6 \
    --device=/dev/video7:/dev/video7 \
    "$IMAGE_NAME"


# Revogar acesso ao servidor X11 após sair do container
echo "Revogando acesso ao servidor X11..."
xhost -local:docker
