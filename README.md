# uss-nav

无人机自主探索、目标感知、语义场景理解与多机通信的混合工作区。

## 工作区结构

```
uss-nav/
├── ws_main/                     # ROS catkin 工作区（源码，只读挂载到容器）
│   ├── src/
│   │   ├── planners/
│   │   │   ├── ego_planner/     # 局部规划（plan_manage, plan_env, path_searching,
│   │   │   │                     #   traj_opt, map_interface）
│   │   │   ├── exploration/     # 探索管理、前沿点选择、目标/探索模式切换
│   │   │   └── tracker/         # 目标跟踪
│   │   ├── perception/
│   │   │   ├── scene_graph/     # 语义目标融合、场景图、骨架生成、LLM 接口
│   │   │   ├── yoloe/           # YOLOE 检测（仿真 fake + 真实 predict 脚本）
│   │   │   └── camera_fov/      # 相机视场工具
│   │   ├── mission_executive/   # 任务状态机、启动入口、功能脚本
│   │   ├── uav_simulator/       # 仿真器、地图生成、深度/点云模拟
│   │   └── utils/               # 自定义消息、轨迹/命令工具（quadrotor_msgs, traj_utils 等）
│   ├── project.deps.yaml/       # 依赖记录
│   └── CMakeLists.txt
├── bringup_test/                # 测试启动配置、launch 文件、参数
├── docker/                      # Dockerfile & entrypoints
│   ├── Dockerfile.devel         # 系统依赖镜像（apt, ROS, 静态库）
│   ├── Dockerfile.release       # 发布镜像（含 entrypoint-release.sh）
│   ├── entrypoint.sh            # devel 镜像入口
│   ├── entrypoint-test.sh       # 无头测试入口（环境变量驱动）
│   └── entrypoint-release.sh    # 发布模式入口（REAL_YOLOE 切换）
├── docker-compose.yml           # 四阶段生命周期：devel / build / test / release
├── start_uss_nav_sim_rviz.sh    # 一键启动仿真 + RViz（依赖 ~/rviz_ws）
├── docs/                        # 架构文档（VIEW, EGO, SCENEGRAPH, CODEBASE）
├── tools/                       # md2html 文档渲染工具（Bun）
├── src/                         # CLI 工具（TypeScript，测试管理/流水线）
├── .artifacts/                  # 构建产物 & 运行时产出（PCD/CSV），dot-prefix 隔离
│   └── {devel,build,csv,pcd}/
├── .data/                       # 离线测试数据（PCD 地图 + SceneGraph 快照），untracked
│   ├── pcd/
│   └── scene_graph/
├── .pretrained/                 # 预训练模型权重（YOLOE, MobileCLIP），手动下载
└── test-plans/                  # 测试计划
```

## 构建与运行

采用 Docker 容器化构建，源码通过 bind mount 挂载，**无需在宿主机安装 ROS**。

### 编译

```bash
docker compose build devel     # 构建系统依赖镜像（仅 apt/ROS 变更时执行）
docker compose run --rm build  # 编译源码 → .artifacts/{devel,build}/
```

### 运行仿真

```bash
docker compose run --rm devel   # 启动仿真（需有 X11 DISPLAY）
```

### 一键仿真 + RViz

```bash
./start_uss_nav_sim_rviz.sh     # 启动 devel 容器 + RViz（依赖 ~/rviz_ws）
```

### 无头测试

```bash
TEST_ID=my-test DURATION=60 docker compose run --rm test
```

### 启动模式

| 模式                  | 入口                                                            | 说明                                                                  |
| --------------------- | --------------------------------------------------------------- | --------------------------------------------------------------------- |
| scenegraph-ego (默认) | `roslaunch bringup_test sim_scenegraph_main.launch`               | 离线 PCD 地图 + 离线 SceneGraph → EGO Planner（需 `.data/` 就绪）        |
| random-sim            | `roslaunch bringup_test sim_random_main.launch`                   | 程序化随机地图 + 仿真器 + EGO Planner（无需外部数据）                   |
| 切换模式              | `LAUNCH_MODE=random docker compose up devel`                      | 环境变量 `LAUNCH_MODE` 默认 `scenegraph`，设为 `random` 走程序化模式     |

#### scenegraph-ego 模式数据准备

```bash
mkdir -p .data/pcd .data/scene_graph

# PCD 地图
ln -s /path/to/J30V2_20260629.pcd .data/pcd/J30V2_latest.pcd

# Scene graph 快照 (需包含 manifest.json / scene_graph.json / objects/)
cp -r /path/to/J30V2_snapshot .data/scene_graph/
```

### 指令类型

向 `/bridge/Instruct`（`quadrotor_msgs/Instruction`）发送：

| ID | 类型 | 说明 |
|----|------|------|
| 1 | `TURN_OBJECT_NAV` | 全功能探索 |
| 2 | `TURN_OBJECT_ID_NAV` | 给定物体 ID 导航 |
| 3 | `TURN_REGULAR_EXPLORATION` | 纯建图探索 |
| 4 | `TURN_DF_DEMO` | Demo 演示 |
| 5 | `TURN_GOAL` | 点到点导航 |
| 6 | `TURN_TRACKING` | 跟踪 |
| 7 | `TURN_WAYPOINT_NAV` | 给定目标点导航 |

详情见 `ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg` 和 `instruction_description.md`。

## 核心模块

| 模块 | 位置 | 作用 |
|------|------|------|
| `plan_manage` | `planners/ego_planner/plan_manage` | 主规划入口（里程计、点云、目标 → 轨迹） |
| `plan_env` / `path_searching` / `traj_opt` | `planners/ego_planner/*` | 栅格地图、A* 搜索、MINCO 轨迹优化 |
| `exploration` | `planners/exploration` | 前沿点选择、探索/目标模式切换 |
| `scene_graph` | `perception/scene_graph` | 目标融合、场景图、骨架生成、LLM 交互 |
| `yoloe` | `perception/yoloe` | YOLOE 检测（fake/predict 模式，需要 CUDA） |
| `mission_executive` | `mission_executive` | 任务状态机、启动入口 |
| `uav_simulator` | `uav_simulator` | 仿真无人机、地图生成、深度/点云渲染 |
| `quadrotor_msgs` / `traj_utils` | `utils/*` | 自定义消息、轨迹/命令类型定义 |

## 依赖

### 运行环境

- Docker + Docker Compose
- Ubuntu 20.04 + ROS Noetic（容器内）
- NVIDIA GPU + CUDA（仅 YOLOE 检测链路）
- `~/rviz_ws`（可选，用于可视化）

### 外部 ROS 包（实机链路）

`mavros`、`fast_lio`、`px4ctrl`、`ekf_quat` 等——不在本仓库内，需额外配置。

### 模型权重

预训练权重手动下载到 `.pretrained/`：

```bash
./docker/download-models.sh   # → .pretrained/{yoloe,mobileclip}*.pt
```

## 文档

| 文件 | 说明 |
|------|------|
| `docs/VIEW.md` | 架构总览（SceneGraph + EGO Planner） |
| `docs/EGO.md` | EGO Planner 实时轨迹优化文档 |
| `docs/SCENEGRAPH.md` | SceneGraph 环境表征文档 |
| `docs/NEXT_SCENEGRAPH.md` | SceneGraph 重构提案 |
| `docs/CODEBASE.md` | 全量代码库参考 |

渲染 HTML：

```bash
cd tools/md2html && bun render.ts ../docs/VIEW.md      # 单文档
cd tools/md2html && bun render.ts --tabs                # 多标签 VIEW
```
