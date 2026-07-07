#!/bin/bash

# ================= 配置项 =================
# 容器名称 (方便后续管理)
# CONTAINER_NAME="ros_noetic"
CONTAINER_NAME="ros_noetic_uss_nav"
# 镜像名称 (你刚才 build 的名字)
IMAGE_NAME="ros-noetic-gwq:v2"
# 宿主机代码路径 (默认为当前目录)
HOST_DIR="/home/gwq/gwq"
# 容器内挂载路径
CONTAINER_DIR="/gwq"
# =========================================

echo "🚀 准备启动容器: $CONTAINER_NAME ..."

# 1. 允许 X11 转发 (解决 Rviz 权限问题)
echo "   -> 配置显示权限 (xhost)..."
xhost +local:root > /dev/null

# 2. 清理旧容器 (防止名称冲突)
if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    echo "   -> 发现同名旧容器，正在清理..."
    docker rm -f ${CONTAINER_NAME} > /dev/null
fi

# 3. 后台启动容器 (-d 参数)
# 注意：最后我们执行 /bin/bash 并配合 -it，保证容器不会启动后立即退出
docker run -d \
    -it \
    --name "${CONTAINER_NAME}" \
    --gpus all \
    --net=host \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOST_DIR}:${CONTAINER_DIR}" \
    --workdir="${CONTAINER_DIR}" \
    "${IMAGE_NAME}" \
    /bin/bash

echo "✅ 容器已在后台运行！"
echo "👉 使用 ./into.sh 进入容器"
