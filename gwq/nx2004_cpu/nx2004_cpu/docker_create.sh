#!/bin/bash

CONTAINER_NAME="ros_noetic_oset"
IMAGE_NAME="orin-nx-noetic-cpu:v1"
HOST_DIR="/home/gwq/gwq"
CONTAINER_DIR="/gwq"

echo "🚀 准备启动容器: $CONTAINER_NAME ..."

# 1. 配置显示权限
xhost +local:docker > /dev/null

# 2. 准备 Xauthority (增强显示兼容性)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# 3. 清理旧容器
docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

# 4. 启动容器
# 注意：即使不要求 GPU 计算，--runtime nvidia 也能帮助映射显示驱动
docker run -d \
    -it \
    --name "${CONTAINER_NAME}" \
    --net=host \
    --runtime nvidia \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="XAUTHORITY=$XAUTH" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --volume="/lib/modules:/lib/modules:ro" \
    --volume="${HOST_DIR}:${CONTAINER_DIR}" \
    --workdir="${CONTAINER_DIR}" \
    "${IMAGE_NAME}" \
    /bin/bash

echo "✅ 容器已在后台运行！"
