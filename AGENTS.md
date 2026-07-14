# AGENTS.md — 项目文档与工具索引

> 供 AI agent 快速了解 USS-NAV 的文档体系和可视化工具链。

## 文档索引（根目录 *.md）

| 文件 | 说明 |
|------|------|
| `README.md` | 项目概述、工作区结构、核心模块说明、构建与部署指南 |
| `docs/VIEW.md` | 架构总览入口，包含 SceneGraph 和 EGO Planner 章节概要，多文档导航 |
| `docs/EGO.md` | EGO-Planner 实时轨迹优化文档：12 状态 FSM、算法管道、ROS 话题接口、消息契约、代码组织 |
| `docs/SCENEGRAPH.md` | SceneGraph 上层环境表征：骨架生成、物体管线、区域聚类、LLM 交互、API 参考 |
| `docs/NEXT_SCENEGRAPH.md` | SceneGraph 重构提案：问题分析、目标架构、迁移计划 |
| `docs/CODEBASE.md` | 全量代码库参考：仓库结构、三层架构、62 个 ROS 消息定义、算法与数据流 |
| `instruction_description.md` | Instruction.msg 字段映射参考：12 种 Instruction 类型及各字段含义 |

### 文档渲染

`/tools/md2html/` — 将 Markdown 文档渲染为单页 HTML（语法高亮、Katex 公式、Mermaid 图表、TOC 侧边栏、暗色模式）。
使用 Bun 运行，支持单文档和多标签渲染（`docs/VIEW.md` 使用多标签模板 `template_tabs.html`）。

```bash
cd tools/md2html && bun render.ts ../docs/EGO.md     # 渲染单文档
cd tools/md2html && bun render.ts --tabs         # 多标签渲染 VIEW
```

## 构建系统 (Docker)

### 外层构建 (outer build)

规范 devel 镜像由 `docker/Dockerfile.devel` 构建。不存在规范的 build 镜像或 test 镜像——所有阶段均运行 devel 镜像。

```bash
# 构建 devel 镜像 (ego-planner-sim, docker/Dockerfile.devel)
docker compose build devel

# 或直接 docker build
docker build -t ego-planner-sim -f docker/Dockerfile.devel .
```

### Docker 生命周期

| 阶段 | Compose 服务 | 镜像 | 说明 |
|------|-------------|------|------|
| devel | `devel` | `ego-planner-sim` | 模拟运行镜像，含 ROS 仿真（**不含 rviz**） |
| build | `build` | `ego-planner-sim` | 热重载编译（源码变更后重新 catkin build） |
| test | `test` | `ego-planner-sim` | 无头测试，含 MQTT 遥测桥接 |

**无 canonical build 镜像，无 canonical test 镜像。** build 和 test 阶段复用 devel 镜像。MQTT 测试依赖已注入 devel 镜像。

```bash
# 启动仿真（带 X11 显示）
docker compose run --rm devel

# 热重载编译（源码只读挂载）
docker compose run --rm build

# 运行测试
TEST_ID=my-test DURATION=60 docker compose run --rm test
```

### 容器命名约定

```
uss-nav-{phase}-{GIT_SHA:-local}
```

示例：`uss-nav-devel-abc1234`、`uss-nav-build-local`、`uss-nav-test-local`

### 镜像命名约定

当前镜像简化命名为 `ego-planner-sim`（单体单架构项目，无需多架构 registry 前缀）。
若将来需要多架构或多配置，可采用以下规范格式：

```
{registry}/{app}-{profile}/{arch}-{os}-cuda{version}/{phase}:{tag}
```

例如：`docker.io/uss-nav/ego-planner-sim/x86_64-ubuntu20.04-cu118/devel:latest`

### 挂载隔离规则

| 主机路径 | 容器路径 | 类型 | 权限 | 说明 |
|---------|---------|------|------|------|
| `ws_main/src/` | `/workspace/src/` | bind | 只读 | 源码热重载 |
| `bringup_test/` | `/workspace/src/bringup_test/` | bind | 只读 | 启动测试配置 |
| `.artifacts/` | `/workspace/.artifacts/` | bind | 读写 | 测试产物、PCD/CSV 输出 |
| `/tmp/.X11-unix/` | `/tmp/.X11-unix` | bind | 读写 | X11 显示 |

**容器内可写路径（非主机挂载）** — build 阶段在容器内生成，生命周期随容器：

| 路径 | 说明 | 权限 |
|------|------|------|
| `/workspace/build/` | CMake 构建产物 | 可写（仅容器内） |
| `/workspace/devel/` | ROS workspace devel 空间 | 可写（仅容器内） |

test/release 容器不得共享可变主机路径（只读源码挂载除外）。

### 清理命令

```bash
# 停止所有 uss-nav 容器
docker rm -f $(docker ps -aq --filter name=uss-nav)

# Compose 清理
docker compose down --remove-orphans

# 清理 build/test 产生的构建产物（在构建容器内）
docker run --rm -v ws_main/src:/workspace/src:ro -v bringup_test:/workspace/src/bringup_test:ro ego-planner-sim \
  bash -c "rm -rf /workspace/build/* /workspace/devel/*"

# 清理 Docker 构建缓存
docker builder prune --filter until=24h

# 删除指定镜像
docker rmi ego-planner-sim
```

### 内层编译 (inner compile / hot-reload)

```bash
docker compose run --rm build
```

在 devel 容器内手动执行：

```bash
docker compose exec devel bash -c "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && catkin build --no-status -j4"
```

### 遗留说明

- `docker/Dockerfile.test` 是一个**非规范辅助文件**（历史遗留），不再用于规范测试验证。规范测试验证始终通过 `docker compose run --rm test` 运行 devel 镜像。
- `docker-image-naming` skill 中提到的 `ego-planner-test` 镜像名已不再使用，属于历史记录。

## 数据契约

### `.pretrained/` — 预训练模型权重

预训练模型权重（YOLOE、MobileCLIP）遵循与 `.artifacts/` 相同的 dot-prefix 约定，但语义分离：

| 目录 | 用途 | 生命周期 |
|------|------|----------|
| `.pretrained/` | 预下载输入模型权重 | 手动触发下载，跨容器复用 |
| `.artifacts/` | 运行时产出（PCD/CSV） | 每次测试自动生成 |

#### 路径映射

| 主机路径 | 容器路径 | 类型 | 说明 |
|---------|---------|------|------|
| `./.pretrained/` | `/workspace/.pretrained/` | bind | YOLOE / MobileCLIP 权重文件 |

#### 下载

```bash
./docker/download-models.sh          # → ./.pretrained/
```

#### ROS 参数配置

`bringup_test/params/yoloe_pretrained.yaml` 定义了容器内权重路径的 ROS 参数映射，通过 `rosparam load` 或 launch 文件 `<rosparam>` 引入。

```yaml
model_path: /workspace/.pretrained/yoloe-11m-seg-pf.pt
prompt_model_path: /workspace/.pretrained/yoloe-11m-seg.pt
clip_model_path: /workspace/.pretrained/mobileclip_blt.pt
prompt_file_path: /workspace/src/perception/yoloe/prompt/prompt.txt
```

#### 启动模式

`entrypoint-release.sh` 通过 `REAL_YOLOE` 环境变量切换：

```bash
# 默认：fake YOLOE（无需权重，用于仿真测试）
docker compose run --rm release

# 实时推理模式（需要 GPU + 相机话题）
REAL_YOLOE=1 docker compose run --rm release
```

当 `REAL_YOLOE=1` 且 `/workspace/.pretrained/yoloe-11m-seg-pf.pt` 存在时，自动启动 `predict_realtime_cam_sim.py`；否则回退到 `fake_realtime_cam_sim.py`。

## 可视化

devel 镜像基于 `ros:noetic-ros-base`，**不含 rviz**。仿真启动后所有节点正常运行（地图生成、四旋翼动力学、EGO 规划器），但无内置可视化输出。

可视化由 `~/rviz_ws` 提供。该工作区基于 `osrf/ros:noetic-desktop-full`（含完整 ROS 桌面 + rviz），
包含自定义 RViz 插件（PolyhedronArray / EllipsoidArray / ProbMap 等）和 EGO Planner 场景的配置。

启动流程：

```bash
# 终端 1：启动 uss-nav 仿真
cd ~/uss-nav && docker compose run --rm devel

# 终端 2：启动 RViz（共享 ROS master，network_mode: host）
cd ~/rviz_ws && ./start_uss_nav.sh
```
