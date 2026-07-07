#!/bin/bash

# CONTAINER_NAME="ros_noetic"
CONTAINER_NAME="ros_noetic_uss_nav"


# 检查容器是否在运行
if [ ! "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
    echo "❌ 错误：容器 ${CONTAINER_NAME} 未运行！"
    echo "请先执行 ./run.sh 启动容器。"
    exit 1
fi

# 进入容器
# 这里的 "bash" 也可以换成 "zsh" (如果你镜像里装了的话)
docker exec -it "${CONTAINER_NAME}" bash