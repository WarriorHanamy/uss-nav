# syntax=docker/dockerfile:1.4

FROM dustynv/l4t-pytorch:r36.2.0

LABEL maintainer="Weiqi Gai <gaiwq@buaa.edu.cn>"

# 1. 代理与环境变量
ARG http_proxy
ARG https_proxy
ENV http_proxy=${http_proxy} \
    https_proxy=${https_proxy} \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    ROS_ROOT=/opt/ros/noetic \
    ROS_WS=/opt/ros_ws \
    CATKIN_WS=/workspace/catkin_ws \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/compat:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}

SHELL ["/bin/bash", "-c"]

# 2. 基础依赖安装 (包含已验证的所有修复)
RUN printf '%s\n' \
    'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy main restricted universe multiverse' \
    'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-updates main restricted universe multiverse' \
    'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-backports main restricted universe multiverse' \
    'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-security main restricted universe multiverse' \
    > /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg ca-certificates wget && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential git locales tzdata cmake \
        python3-dev python3-pip python3-venv python3-setuptools python3-wheel \
        python3-catkin-pkg python3-rospkg python3-rosdep python3-vcstool \
        python-is-python3 python3-future python3-netifaces python3-empy \
        sip-dev python3-sip-dev pyqt5-dev python3-pyqt5 python3-pyqt5.sip \
        # --- 系统核心库与 RViz/Rosbag 修复 ---
        libgpgme-dev libssl-dev liblzma-dev libbz2-dev \
        libtinyxml-dev libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev \
        libconsole-bridge-dev libboost-all-dev liblog4cxx-dev libpoco-dev \
        liblz4-dev libyaml-cpp-dev libopencv-dev libcurl4-openssl-dev \
        libgtest-dev \
        # --- 机器人学、渲染与物理 ---
        libogre-1.12-dev libassimp-dev liborocos-kdl-dev libbullet-dev \
        # --- Qt5 完整环境 (RViz 核心) ---
        qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
        libqt5opengl5-dev libqt5svg5-dev libqt5x11extras5-dev libqwt-qt5-dev \
        vim tmux htop net-tools unzip rsync && \
    rm -rf /var/lib/apt/lists/*

# 兼容性链接
RUN cd /usr/src/gtest && cmake . && make && cp lib/*.a /usr/lib
RUN if [ -f /usr/bin/sip ]; then ln -s /usr/bin/sip /usr/local/bin/sip; fi

# 3. 安装 CMake 3.23.5
RUN wget --no-check-certificate https://github.com/Kitware/CMake/releases/download/v3.23.5/cmake-3.23.5-linux-aarch64.tar.gz && \
    tar -zxvf cmake-3.23.5-linux-aarch64.tar.gz -C /usr/local --strip-components=1 && \
    rm cmake-3.23.5-linux-aarch64.tar.gz

# 4. Python 环境 (保持默认)
RUN python3 -m pip install --upgrade pip setuptools wheel \
    --trusted-host jetson.webredirect.org \
    --trusted-host pypi.jetson-ai-lab.dev \
    --trusted-host pypi.ngc.nvidia.com && \
    python3 -m pip install \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --trusted-host pypi.tuna.tsinghua.edu.cn \
    --trusted-host jetson.webredirect.org \
    --trusted-host pypi.jetson-ai-lab.dev \
    --trusted-host pypi.ngc.nvidia.com \
    numpy scipy matplotlib pandas pillow pyyaml seaborn onnx onnxsim protobuf pycuda ultralytics

###############################################################################
# 5. ROS1 Noetic 源码编译
###############################################################################
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}
COPY ros_noetic_src/ ${ROS_WS}/src/

# 忽略干扰包
RUN find src -name "*_tutorials" -type d -exec touch {}/CATKIN_IGNORE \;

# C++17 补丁
RUN find src -name CMakeLists.txt -type f -exec \
    sed -i 's/-std=c++11/-std=c++17/g' {} \; || true && \
    find src -name CMakeLists.txt -type f -exec \
    sed -i 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 17)/g' {} \; || true

# 【核心修复】：为 rviz 显式指定 Qt5 路径，并禁用测试
RUN ./src/catkin/bin/catkin_make_isolated \
    --install \
    --install-space /opt/ros/noetic \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCATKIN_ENABLE_TESTING=OFF \
    -DQt5_DIR=/usr/lib/aarch64-linux-gnu/cmake/Qt5 \
    -j8

###############################################################################
# 6. 用户工作空间与环境配置
###############################################################################
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}
COPY scene_graph/ ${CATKIN_WS}/src/scene_graph/

RUN source /opt/ros/noetic/setup.bash && \
    catkin_init_workspace src && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/catkin_ws/devel/setup.bash ]; then source /workspace/catkin_ws/devel/setup.bash; fi" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]