# syntax=docker/dockerfile:1.4
FROM ubuntu:20.04

LABEL maintainer="Weiqi Gai <gaiwq@buaa.edu.cn>"
ENV DEBIAN_FRONTEND=noninteractive

ENV http_proxy=http://127.0.0.1:7890 \
    https_proxy=http://127.0.0.1:7890 \
    HTTP_PROXY=http://127.0.0.1:7890 \
    HTTPS_PROXY=http://127.0.0.1:7890 \
    no_proxy=localhost,127.0.0.1,::1 \
    NO_PROXY=localhost,127.0.0.1,::1

# 注意：这里假设你的宿主机代理端口是 7890
# 如果是 Linux 宿主机，通常使用 172.17.0.1；如果是 Win/Mac，建议用 host.docker.internal
ARG PROXY_URL=http://127.0.0.1:7890

# 1. 切换国内镜像源 (解决网络慢的问题，并适配 ARM 架构的 ports 源) 
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/ports.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list

# 2. 系统基础环境初始化 [cite: 1, 2]
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
    tzdata \
    ca-certificates \
    gnupg2 \
    lsb-release \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# 3. 安装开发常用工具 (仅保留原生 python3 基础组件) [cite: 2, 3]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-pip \
    build-essential \
    git \
    git-lfs \
    curl \
    vim \
    tmux \
    wget \
    htop \
    net-tools \
    unzip \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libglfw3-dev \
    libglm-dev \
    libx11-dev \
    libomp-dev \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libcgal-dev \
    libompl-dev \
    libzmqpp-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. 安装 CMake (修改为适配 Orin NX 的 ARM64 架构版本) 
RUN wget https://github.com/Kitware/CMake/releases/download/v3.23.5/cmake-3.23.5-linux-aarch64.sh \
    && mkdir -p /opt/cmake \
    && sh /cmake-3.23.5-linux-aarch64.sh --prefix=/opt/cmake --skip-license \
    && ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake \
    && rm /cmake-3.23.5-linux-aarch64.sh

# 5. 安装 igraph 0.10.16 (从源码编译) [cite: 4, 5]
RUN wget https://github.com/igraph/igraph/releases/download/0.10.16/igraph-0.10.16.tar.gz \
    && tar -xzf igraph-0.10.16.tar.gz \
    && cd igraph-0.10.16 \
    && mkdir -p build && cd build \
    && cmake .. \
      -DBUILD_SHARED_LIBS=ON \
      -DIGRAPH_GLPK_SUPPORT=OFF \
      -DIGRAPH_GRAPHML_SUPPORT=OFF \
      -DIGRAPH_OPENMP_SUPPORT=OFF \
      -DIGRAPH_USE_INTERNAL_BLAS=ON \
      -DIGRAPH_USE_INTERNAL_LAPACK=ON \
      -DIGRAPH_USE_INTERNAL_ARPACK=ON \
      -DIGRAPH_USE_INTERNAL_GLPK=OFF \
      -DIGRAPH_USE_INTERNAL_GMP=ON \
      -DIGRAPH_WARNINGS_AS_ERRORS=OFF \
      -DCMAKE_BUILD_TYPE=Debug \
    && make -j8 \
    && make install \
    && cd / \
    && rm -rf igraph-0.10.16 igraph-0.10.16.tar.gz

# 6. 安装 ROS Noetic (使用清华源，并避免 curl 外部 Key) [cite: 5, 6]
ENV ROS_DISTRO=noetic
RUN apt-get update && apt-get install -y gnupg2 lsb-release && \
    # 直接添加 Key 到受信任列表 [cite: 5, 6]
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool && \
    rm -rf /var/lib/apt/lists/*

# 7. MAVROS & PlotJuggler (添加了 msgs 支持) [cite: 6, 7]
WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-octomap-rviz-plugins \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-plotjuggler-msgs \
    && wget -e "https_proxy=${PROXY_URL}" https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# 8. 工作空间初始化 [cite: 8]
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && mkdir -p /workspace/catkin_ws/src"
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]
