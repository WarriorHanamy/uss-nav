FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]

# ── system dependencies ────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    python3-catkin-tools python3-osrf-pycommon \
    # OpenGL / GLFW / GLEW (for pcl_render_node)
    libgl1-mesa-dev libglu1-mesa-dev \
    libglew-dev libglfw3-dev \
    mesa-utils \
    # ROS extras
    ros-noetic-tf ros-noetic-nodelet ros-noetic-image-transport \
    ros-noetic-pcl-ros ros-noetic-rviz \
    # Other planner deps
    libarmadillo-dev libigraph-dev libjpeg-dev \
    # Headless display
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# ── create catkin workspace ────────────────────────────────────────
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# ── copy workspace packages ────────────────────────────────────────
# Messages / utilities
COPY ws_main/src/utils/quadrotor_msgs    ./utils/quadrotor_msgs
COPY ws_main/src/utils/traj_utils        ./utils/traj_utils
COPY ws_main/src/utils/uav_utils         ./utils/uav_utils
COPY ws_main/src/utils/catkin_simple     ./utils/catkin_simple

# EGO planner core
COPY ws_main/src/planner/ego_plannerv3/plan_env          ./ego_plannerv3/plan_env
COPY ws_main/src/planner/ego_plannerv3/path_searching    ./ego_plannerv3/path_searching
COPY ws_main/src/planner/ego_plannerv3/traj_opt          ./ego_plannerv3/traj_opt
COPY ws_main/src/planner/ego_plannerv3/map_interface     ./ego_plannerv3/map_interface
COPY ws_main/src/planner/ego_plannerv3/plan_manage       ./ego_plannerv3/plan_manage

# UAV simulator
COPY ws_main/src/planner/uav_simulator/so3_quadrotor_simulator ./uav_simulator/so3_quadrotor_simulator
COPY ws_main/src/planner/uav_simulator/so3_control          ./uav_simulator/so3_control
COPY ws_main/src/planner/uav_simulator/local_sensing        ./uav_simulator/local_sensing
COPY ws_main/src/planner/uav_simulator/map_generator        ./uav_simulator/map_generator
    COPY ws_main/src/utils/pose_utils                ./utils/pose_utils
    COPY ws_main/src/planner/uav_simulator/fake_drone           ./uav_simulator/fake_drone
COPY ws_main/src/planner/uav_simulator/fake_so3_quadrotor   ./uav_simulator/fake_so3_quadrotor
COPY ws_main/src/planner/uav_simulator/fake_so3_controller  ./uav_simulator/fake_so3_controller

# Exploration (for planner_cmd_mux and exploration_node)
COPY ws_main/src/planner/exploration/exploration_manager    ./exploration/exploration_manager
COPY ws_main/src/planner/exploration/perception_utils       ./exploration/perception_utils
COPY ws_main/src/planner/exploration/active_perception      ./exploration/active_perception
COPY ws_main/src/planner/exploration/lkh_tsp_solver         ./exploration/lkh_tsp_solver

# Scene graph (needed by active_perception)
COPY ws_main/src/planner/scene_graph ./scene_graph

# ── patch hardcoded paths ──────────────────────────────────────────
RUN sed -i \
    's|string normal_filename = "/home/dji/meshmap/normal_files/Knowles_01_normals.pcd";|// string normal_filename = "/home/dji/meshmap/normal_files/Knowles_01_normals.pcd"; // patched for Docker|' \
    /catkin_ws/src/uav_simulator/local_sensing/include/opengl_sim.hpp && \
  sed -i \
    's|pcl::io::savePCDFileASCII(normal_filename, \*all_normals);|// pcl::io::savePCDFileASCII(normal_filename, *all_normals); // patched for Docker|' \
    /catkin_ws/src/uav_simulator/local_sensing/include/opengl_sim.hpp

RUN sed -i 's|fp = fopen("/home/ecstasy/catkin_ws/fov_data.csv","w");|// fp = fopen("/home/ecstasy/catkin_ws/fov_data.csv","w"); // patched for Docker|' \
    /catkin_ws/src/uav_simulator/local_sensing/include/FOV_Checker/FOV_Checker.cpp

# ── remove problematic packages before build ──────────────────────
RUN rm -rf /catkin_ws/src/utils/odom_visualization && \
  rm -rf /catkin_ws/src/exploration/active_perception && \
  rm -rf /catkin_ws/src/scene_graph && \
  sed -i '/scene_graph/d' /catkin_ws/src/ego_plannerv3/plan_manage/CMakeLists.txt && \
  sed -i '/scene_graph/d' /catkin_ws/src/ego_plannerv3/plan_manage/package.xml

# ── Layer 1: C++ build (slow, cached) ─────────────────────────────
WORKDIR /catkin_ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin build \
        quadrotor_msgs uav_utils traj_utils catkin_simple \
        plan_env path_searching traj_opt map_interface \
        so3_quadrotor_simulator so3_control \
        local_sensing_node map_generator \
        lkh_tsp_solver perception_utils \
        ego_planner \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-w"

# ── Layer 2: bringup (fast, rebuilt on every launch/config change) ─
COPY ws_main/src/planner/sim_bringup /catkin_ws/src/sim_bringup
RUN source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    catkin build sim_bringup

# ── entrypoint ────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
