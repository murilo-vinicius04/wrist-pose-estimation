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
