# Codebase Documentation

> Auto-generated repository documentation — `/home/rec/uss-nav`

## Overview

**USS-NAV** (UAV Semantic Scene Navigation) is a hybrid robotics autonomy stack for **multi-drone autonomous exploration with semantic scene understanding and LLM-driven navigation**. The system integrates open-vocabulary object detection (YOLOE + MobileCLIP), topological scene graph construction, LLM-based decision making, real-time trajectory optimization (EGO-Planner), and multi-drone swarm coordination — all running on ROS1 Noetic.

The repository is a composite of three largely independent sub-projects:
- `ws_main/` — ROS1 catkin workspace with the core planning/exploration/semantic stack (C++, Python)
- `yoloe/` — Standalone YOLOE vision model (Python, fork of Ultralytics YOLO)
- `Elastic-Tracker/` — Forked trajectory planning framework for target tracking (C++)

---

## Repository Structure

```
uss-nav/
├── README.md                      # Primary project documentation (Chinese, 388 lines)
├── instruction_description.md     # Instruction.msg field mapping reference (629 lines)
├── .gitignore                     # Comprehensive (ROS build artifacts, ML weights, IDE)
├── CODEBASE.md                    # This document
│
├── ws_main/                       # [PRIMARY] ROS1 Noetic catkin workspace
│   ├── src/CMakeLists.txt         # Top-level catkin CMake
│   ├── src/
│   │   ├── planner/               # Core planning & navigation
│   │   │   ├── ego_plannerv3/     # EGO-planner: trajectory optimization, A*, grid map, path search
│   │   │   ├── exploration/       # Exploration FSM, frontier management, active perception
│   │   │   ├── scene_graph/       # Semantic scene graph: skeleton, objects, areas, LLM interface
│   │   │   ├── mission_fsm/       # Mission state machine, launch configs
│   │   │   └── uav_simulator/     # UAV simulator (so3_quadrotor, local_sensing, mockamap)
│   │   ├── network/
│   │   │   └── NetBridgeForSwarm/ # Multi-drone ROS bridge over ZMQ/UDP
│   │   ├── unity_utils/           # Unity ROS-TCP integration
│   │   ├── utils/                 # Messages (quadrotor_msgs, traj_utils), RViz plugins, tools
│   │   └── script/                # 42+ operational shell scripts
│   └── scripts/                   # pub_instrucion.sh
│
├── yoloe/                         # [SUBMODULE] YOLOE vision model (Ultralytics fork)
│   ├── pyproject.toml             # Python package (setuptools)
│   ├── requirements.txt           # Editable install: ., lvis-api, ml-mobileclip, CLIP
│   ├── ultralytics/               # Modified Ultralytics (YOLOE models, train/predict/val)
│   ├── track_engine/              # TensorRT tracking API
│   ├── third_party/               # Vendored: sam2, ml-mobileclip, CLIP, lvis-api
│   ├── fast-reid/                 # FastReID submodule
│   ├── prompt/                    # Detection prompt vocabulary
│   └── docker/                    # 10 Dockerfile variants (GPU, CPU, Jetson, ARM, etc.)
│
└── Elastic-Tracker/               # [SUBMODULE] Elastic tracker (ZJU FAST-Lab fork)
    ├── src/
    │   ├── planning/              # Main elastic tracker + traj_opt + DecompROS
    │   ├── uav_simulator/         # Duplicate simulator subset
    │   ├── mapping/               # Map adapter
    │   └── detection/             # Object detection + target EKF
    ├── sh_utils/                  # Utility shell scripts
    └── README.md
```

**Architecture Pattern**: Hybrid monorepo — 3 independent sub-projects aggregated under one root, each with its own build system and dependency graph. No formal workspace manager (npm/pnpm/Cargo workspaces). The `Elastic-Tracker/` is a near-duplicate subset of `ws_main/` for elastic trajectory planning.

### File Naming Conventions
| Type               | Convention      | Examples                                          |
| ------------------ | --------------- | ------------------------------------------------- |
| ROS msg            | PascalCase      | `Instruction.msg`, `PositionCommand.msg`           |
| ROS launch         | snake_case      | `obj_nav.launch`, `bridge_drone.launch`            |
| C++ source         | snake_case      | `fast_exploration_fsm.cpp`, `grid_map.h`            |
| Python scripts     | snake_case      | `predict_realtime_cam_sim.py`, `yoloe_server.py`    |
| Shell scripts      | snake_case .sh  | `run_with_unknown_map.sh`, `takeoff.sh`             |

### Key Entry Points
| Entry                                        | Type          | Purpose                                    |
| -------------------------------------------- | ------------- | ------------------------------------------ |
| `roslaunch ego_planner obj_nav.launch`       | Simulation    | Main sim launch (planner + simulator)      |
| `bash ws_main/src/script/run_with_unknown_map.sh` | Real   | Full real-UAV startup with live sensors    |
| `bash ws_main/src/script/run_with_known_map.sh`   | Real   | Startup with pre-saved map                 |
| `python yoloe/predict_realtime_cam_sim.py`   | Vision        | Real-time YOLOE detection (sim)            |
| `python yoloe/predict_realtime_cam_real.py`  | Vision        | Real-time YOLOE detection (real UAV)       |
| `roslaunch mission_fsm bridge_drone.launch`  | Multi-drone   | Bridge node for swarm communication        |
| `roslaunch mission_fsm rviz.launch`          | Visualization | RViz setup (recommended)                   |

---

## Getting Started

### Prerequisites
- **OS**: Ubuntu 20.04
- **ROS**: Noetic (full desktop install)
- **Python**: >= 3.8
- **CUDA**: Required for YOLOE GPU inference (>= 11.x)
- **System packages**: OpenCV >= 4.0, PCL >= 1.7, Eigen3, igraph >= 0.10, Armadillo, Qt5/RViz dev, ZeroMQ
- **External ROS packages**: mavros, fast_lio, px4ctrl, ekf_quat (for real drone operations)

### Build
```bash
# Build ROS workspace
cd ws_main
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

# Install YOLOE and dependencies
cd yoloe
pip install -e .
pip install -e third_party/lvis-api -e third_party/ml-mobileclip -e third_party/CLIP

# Or use requirements.txt
pip install -r requirements.txt
```

### Quick Start (Simulation)
```bash
# Terminal 1: RViz
roslaunch mission_fsm rviz.launch

# Terminal 2: Planner + Simulator
roslaunch ego_planner obj_nav.launch

# Set 2D Nav Goal in RViz to trigger exploration
```

### Real Drone
```bash
bash ws_main/src/script/run_with_unknown_map.sh
```

---

## Architecture

The system follows a **pipeline architecture** across three layers:

### Layer 1: Perception
```
Camera (RGB+D) → YOLOE Detector → EncodeMask msg → ObjectFactory → ObjectMap
                                       ↑
                                 MobileCLIP/CLIP
```

### Layer 2: Scene Understanding
```
ObjectMap + Free Space → SkeletonGenerator → Polyhedron Graph
                              ↓
                    SpectralCluster/AreaHandler → Areas/Rooms
                              ↓
                    LLM Interface (OpenAI-compatible API)
                              ↓
                    Room labels, area choices, navigation targets
```

### Layer 3: Planning & Control
```
Instruction (/bridge/Instruct) → FastExplorationFSM → FrontierManager
                                      ↓
                              EGOReplanFSM → EGOPlannerManager
                                      ↓
                              A* path search + MINCO trajectory opt
                                      ↓
                              PositionCommand → PX4 controller → UAV
```

### Key Architectural Components

| Component                | Role                                                                  | Location                                    |
| ------------------------ | --------------------------------------------------------------------- | ------------------------------------------- |
| `FastExplorationFSM`     | Top-level mission orchestrator, 25+ FSM states                        | `exploration/exploration_manager/`           |
| `EGOReplanFSM`           | Low-level trajectory execution, 12 FSM states                         | `ego_plannerv3/plan_manage/`                 |
| `EGOPlannerManager`      | Gradient-based trajectory optimization (L-BFGS + MINCO)               | `ego_plannerv3/plan_manage/`                 |
| `SceneGraph`             | Scene construction, LLM prompt generation, path-to-object queries     | `planner/scene_graph/`                       |
| `SkeletonGenerator`      | Topological skeleton: polyhedron decomposition of free space          | `planner/scene_graph/`                       |
| `ObjectFactory`          | Multi-threaded object detection fusion pipeline                       | `planner/scene_graph/`                       |
| `FrontierManager`        | Frontier selection, TSP-based viewpoint tour optimization             | `exploration/exploration_manager/`           |
| `GridMap`                | Occupancy grid mapping with ESDF and trilinear interpolation          | `ego_plannerv3/plan_env/`                    |
| `SwarmRosBridge`         | Multi-drone ROS topic/service/image forwarding via ZMQ                | `network/NetBridgeForSwarm/`                 |

### Two-Level Hierarchical FSM
1. **Outer FSM** (`FastExplorationFSM`): High-level mission states — initiate, explore, track, LLM-guided explore, VLA swarm, terminate
2. **Inner FSM** (`EGOReplanFSM`): Low-level trajectory execution — wait target, generate new traj, replan, exec, emergency stop

---

## Data Layer

**No traditional database or ORM.** The data layer consists of:

### Message Schemas (Primary Data Model)
- **62 `.msg` files** in `ws_main/src/utils/quadrotor_msgs/msg/` — core drone commands, odometry, commands, detection output
- **3 scene graph messages**: `EncodeMask`, `PromptMsg`, `WordVector`
- **17 trajectory messages** in `ws_main/src/utils/traj_utils/msg/`
- **3 network bridge messages**: `PtCloudCompress`, `NetworkInfo`, `NetworkArray`

### C++ Runtime Structures
Defined in `ws_main/src/planner/scene_graph/include/scene_graph/data_structure.h`:
- `ObjectNode` — detected object: id, label, confidence, 512-d feature vector, OBB, point cloud
- `Polyhedron` — convex free-space region in the skeleton
- `PolyhedronFtr` — frontier of a polyhedron (unexpanded facet)
- `PolyhedronCluster` — room/area: collection of polyhedra + objects
- `Vertex`, `Facet`, `Edge` — geometric primitives

### Serialization
- **ROS binary** (`ros::serialization`) — network transport
- **nlohmann JSON** — scene graph save/load (snapshots to `saved_data/`)
- **PCD binary** — per-object point cloud files
- **NumPy `.npy`/`.cache`** — YOLOE training label/image caches
- **PyTorch `.pt`** — model checkpoints with `SafeUnpickler`

### Data Flow
```
Camera → EncodeMask → ObjectFactory → ObjectNode → SceneGraph
                                                       ↓
                                             LLM Prompt → Answer
                                                       ↓
                                          FastExplorationFSM → Goal → Trajectory → Controller
```

---

## Core Logic

### Exploration
- **Frontier-based exploration**: Identify boundaries between known/unknown space
- **Two strategies**: `planExploreRapid()` (greedy) and `planExploreTSP()` (optimal via LKH solver)
- **LLM-guided exploration**: Consult LLM for semantic area selection

### Scene Graph Construction
- **Skeleton generation**: Ray-casting from drone + QuickHull convex decomposition
- **Object fusion**: Spatial + semantic similarity matching across frames (Hungarian algorithm)
- **Area clustering**: Leiden community detection on polyhedron adjacency graph (igraph)
- **LLM room prediction**: JSON prompt → classification into 10 room types

### Trajectory Planning
- **A\* path search** on grid map with dynamic replanning
- **MINCO trajectory optimization** via L-BFGS:
  - Obstacle avoidance (ESDF gradient)
  - Smoothness penalties
  - Feasibility constraints (vel/acc/jerk/snap limits)
  - Swarm collision avoidance

### Mission FSM State Machine
25+ states organized in groups:
- `INIT`, `WARM_UP`, `WAIT_TRIGGER` — initialization
- `PLAN_EXPLORE`, `LLM_PLAN_EXPLORE`, `APPROACH_EXPLORE` — exploration
- `PLAN_TRACK`, `APPROACH_TRACK` — target tracking
- `THINKING`, `YAW_HANDLE` — LLM deliberation and orientation
- `GO_TARGET_OBJECT`, `GO_TARGET_WITH_WAYPOINT` — object navigation
- `VLA_SWARM_*` (8 states) — multi-drone VLA coordination

---

## API Reference

The application exposes functionality through **ROS1 topics and services** (no REST/HTTP/GraphQL/gRPC).

### Primary Command Interface
**Topic**: `/bridge/Instruct` — `quadrotor_msgs/Instruction`

12 instruction types:
| ID | Name                    | Purpose                                        |
|----|-------------------------|------------------------------------------------|
| 1  | `OBJECT_NAV`            | Navigate to nearest object matching command    |
| 2  | `OBJECT_ID_NAV`         | Navigate to specific object by ID              |
| 3  | `REGULAR_EXPLORATION`   | Start autonomous exploration                   |
| 4  | `DF_DEMO`               | LLM demo: natural language → find object       |
| 5  | `GOAL`                  | Navigate to 3D point                           |
| 6  | `TRACKING`              | Track a moving target                          |
| 7  | `WAYPOINT_NAV`          | Navigate to waypoint(s)                        |
| 9  | `SAVE_SCENE_GRAPH`      | Save scene graph to disk                       |
| 10 | `LOAD_SCENE_GRAPH`      | Load scene graph from disk                     |
| 11 | `REQUEST_ALL_AREA_AND_OBJS` | Request full scene graph summary            |
| 12 | `VLA_SWARM`             | Multi-drone VLA task decomposition             |

### Key Topics
| Topic                         | Type                                | Direction       |
| ----------------------------- | ----------------------------------- | --------------- |
| `/bridge/Instruct`            | `quadrotor_msgs/Instruction`         | Command input   |
| `/Instruct_res`               | `quadrotor_msgs/InstructionResMsg`   | Response output |
| `/yoloe/encodemask`           | `scene_graph/EncodeMask`             | Perception      |
| `/scene_graph/prompt`         | `scene_graph/PromptMsg`              | LLM request     |
| `/scene_graph/llm_ans`        | `scene_graph/PromptMsg`              | LLM response   |
| `local_goal`                  | `quadrotor_msgs/EgoGoalSet`          | Planner goal    |
| `/position_cmd`               | `quadrotor_msgs/PositionCommand`     | Control output  |
| `/px4ctrl/takeoff_land`       | `quadrotor_msgs/TakeoffLand`         | Takeoff/land   |

### Swarm Bridge (ZMQ)
Configuration-driven bridge in `ws_main/src/network/NetBridgeForSwarm/swarm_ros_bridge/config/`:
- `obj_nav_cfg.yaml` — 360 lines mapping 20+ topics with IP/port/frequency/compression settings
- `ip_real.yaml` — Real drone IP mappings

### No Authentication
The system assumes a trusted ROS1 network. No encryption, tokens, or access control. API keys are hardcoded in `LLM_interface_thread.py`.

---

## Testing

**Minimal, uneven test coverage** — typical of academic/research codebases.

### C++ (Google Test)
- 2 active gtest packages: `uav_utils` (math functions), `cv_bridge_noetic_fit_version` (5 test files)
- 8 commented-out gtest targets in various CMakeLists.txt
- No C++ mocking framework (no GMock)
- ROS-node-level tests are hand-rolled `main()` executables requiring manual verification

### Python
- `yoloe/fast-reid/tests/`: 7 test files using `unittest.TestCase`
- `ws_main/src/unity_utils/ROS-TCP-Endpoint/test/`: 8 files using `pytest` + `unittest.mock` (best test hygiene)
- `yoloe/third_party/CLIP/tests/`: 1 pytest file with parametrize
- Coverage config in `yoloe/pyproject.toml` (only for `ultralytics/` package)

### No CI
No `.github/workflows/`, `.gitlab-ci.yml`, or Jenkinsfile exist.

### Test Commands
```bash
# C++ tests
catkin_make tests
# Run specific binary:
ws_main/devel/lib/uav_utils/uav_utils-test

# Python tests (yoloe/)
pytest
pytest --cov=ultralytics/
```

---

## Deployment

This is a **research codebase** with no production deployment infrastructure.

### ROS Launch-Based Deployment
- **Simulation**: `roslaunch ego_planner obj_nav.launch` (built-in simulator)
- **Real UAV**: Shell scripts in `ws_main/src/script/`
  - `run_with_unknown_map.sh` — full startup (odom, LiDAR, detection, planner, bridge)
  - `run_with_known_map.sh` — startup with pre-saved environment map
- **Multi-drone**: `mission_fsm/bridge_drone.launch` + `bridge_station.launch`

### Operational Scripts (42+ in `ws_main/src/script/`)
| Category      | Scripts                                              |
| ------------- | ---------------------------------------------------- |
| Startup       | `run_*.sh`, `run_planner.sh`, `run_sim.sh`            |
| Takeoff/Land  | `takeoff.sh`, `land.sh`, `topic_takeoff.sh`           |
| Recording     | `record*.sh`, `bag_record.sh`, `pointcloud_record.sh` |
| Monitoring    | `killros.sh`, `kill_node.sh`, `timesync.sh`           |
| Debug         | `triger_goal.sh`, `traj_trigger.sh`                  |

### Containerization (YOLOE only)
10 Dockerfiles under `yoloe/docker/` for various hardware targets (GPU, CPU, Jetson, ARM64, Jupyter). No Dockerfile for the ROS planning workspace.

### No CI/CD, K8s, IaC, or Release Process
Environment separation is via launch file variants (`run_in_sim.xml` vs `run_in_real.xml`).

---

## Dependencies

### ROS/C++ Layer (Core Planning)
| Dependency       | Purpose                                      |
| ---------------- | -------------------------------------------- |
| **Eigen3**       | Linear algebra (used in 12+ packages)        |
| **PCL >= 1.7**   | Point cloud processing                       |
| **OpenCV >= 4.0**| Image processing                             |
| **igraph >= 0.10** | Graph theory (scene graph clustering)      |
| **ZeroMQ**       | Inter-drone communication bridge             |
| **nlohmann/json**| JSON serialization (vendored, header-only)   |
| **Armadillo**    | C++ linear algebra (optional)                |
| **libleidenalg** | Community detection algorithm (vendored)      |
| **quickhull**    | 3D convex hull (vendored)                     |
| **lkh_tsp_solver** | LKH heuristic TSP solver (vendored)        |

### Python Layer (YOLOE Vision)
| Dependency         | Version       | Purpose                              |
| ------------------ | ------------- | ------------------------------------ |
| **torch**          | >= 1.8.0      | Deep learning backbone               |
| **torchvision**    | >= 0.9.0      | Image transforms                     |
| **numpy**          | >= 1.23.0     | Array operations                     |
| **opencv-python**  | >= 4.6.0      | Camera I/O, visualization            |
| **Pillow**         | >= 7.1.2      | Image loading                        |
| **ultralytics-thop** | >= 2.0.0   | FLOPs profiling                      |

### Vendored Python Sub-packages
- `lvis-api` — LVIS dataset API
- `ml-mobileclip` — Apple MobileCLIP text encoder
- `CLIP` — OpenAI CLIP text encoder
- `sam2` — Segment Anything 2 (aggressively pinned versions)
- `fast-reid` — Person/vehicle re-identification

### Dependency Tensions
- `sam2` pins `torch==2.5.1`, `numpy==1.26.4` — conflicts with ultralytics' relaxed constraints
- No lock files — `pip install` can pull different transitive versions
- Vendored C++ libraries require manual updates

### Security Notes
- **Hardcoded API keys** in `LLM_interface_thread.py` (documented known issue)
- No automated dependency scanning (no Dependabot, Renovate, Safety, or pip-audit)
- ZMQ bridge sends ROS topics unencrypted
- AGPL-3.0 license on Ultralytics fork

---

## Domain Glossary

### Core Concepts

| Term                    | Definition                                                                                                 |
| ----------------------- | ---------------------------------------------------------------------------------------------------------- |
| **Frontier**            | Boundary region between known free space and unknown space — the fundamental attractor for exploration     |
| **Skeleton**            | Topological graph of navigable free space represented as convex polyhedra                                  |
| **Polyhedron**          | A convex free-space region with vertices (WHITE=free, BLACK=occluded, GRAY=boundary) and facets            |
| **Scene Graph**         | Structured environment model: skeleton + areas (rooms) + objects + connectivity                            |
| **Object Factory**      | Multi-threaded pipeline: YOLOE detections → point cloud extraction → spatial merging → persistent object map |
| **ObjectNode**          | Persistent detected object: id, label, 512-d feature vector, OBB, point cloud, detection count             |
| **Area / PolyhedronCluster** | A room/region: group of connected polyhedra with label, objects, and adjacency                        |
| **MINCO Trajectory**    | Minimum Control polynomial trajectory optimized via L-BFGS                                                 |
| **ESDF**                | Euclidean Signed Distance Field — continuous distance to nearest obstacle with gradient                    |
| **EGO-Planner**         | Gradient-based local trajectory optimization (Edge-based Gradient Optimization)                            |
| **Elastic-Tracker**     | Target tracking framework with safety/visibility guarantees (from ZJU FAST-Lab)                           |
| **VLA Swarm**           | Vision-Language-Action multi-drone coordination: LLM decomposes task, assigns sub-tasks to followers      |
| **PromptMsg**           | ROS message for LLM communication: 21 prompt types (room prediction, area choice, task assignment, etc.)   |
| **EncodeMask**          | ROS message from YOLOE: labels, confidences, 512-d text features, segmentation masks                      |
| **Instruction**         | Central command message: 12 types (object nav, exploration, tracking, VLA swarm, etc.) with 68 fields      |

### Instruction Types
| ID | Name                       | ID | Name                        |
|----|----------------------------|----|-----------------------------|
| 1  | OBJECT_NAV                 | 7  | WAYPOINT_NAV                |
| 2  | OBJECT_ID_NAV              | 9  | SAVE_SCENE_GRAPH            |
| 3  | REGULAR_EXPLORATION        | 10 | LOAD_SCENE_GRAPH            |
| 4  | DF_DEMO (LLM Find)         | 11 | REQUEST_ALL_AREA_AND_OBJS   |
| 5  | GOAL (3D point nav)        | 12 | VLA_SWARM (multi-drone)     |
| 6  | TRACKING (moving target)   |    |                             |

### Key Algorithms
| Algorithm                     | Use                                                     |
| ----------------------------- | ------------------------------------------------------- |
| A* / Hybrid A*                | Path search on occupancy grid                           |
| MINCO + L-BFGS                | Trajectory optimization with obstacle avoidance         |
| QuickHull                     | Convex hull → polyhedron generation                     |
| Leiden community detection    | Clustering polyhedra into rooms/areas                   |
| Hungarian algorithm           | Object association across detection frames              |
| LKH TSP                       | Optimal frontier visitation order                       |
| Spectral clustering           | Room segmentation via Laplacian eigenmaps               |
| ESDF (trilinear interpolation)| Continuous collision cost with gradient                 |

### Technology Stack
| Layer            | Technology                                                    |
| ---------------- | ------------------------------------------------------------- |
| Middleware       | ROS 1 Noetic (C++/Python)                                     |
| Navigation       | EGO-Planner + MINCO + A\*                                     |
| Tracking         | Elastic-Tracker                                               |
| Mapping          | Occupancy grid + topological skeleton + scene graph           |
| Object Detection | YOLOE + MobileCLIP + CLIP                                     |
| Clustering       | igraph (Leiden) + spectral clustering                         |
| LLM Interface    | OpenAI-compatible API (Qwen, DeepSeek, etc.)                  |
| Simulator        | Custom: so3_quadrotor + local_sensing + map_generator         |
| Real Drone       | PX4 + MAVROS + Fast-LIO/EKF + OAK-D/RealSense camera         |
| Swarm            | NetBridgeForSwarm (custom ROS bridge over ZMQ/UDP)            |
| Spatial Index    | iKD-Tree (incremental KD-Tree)                                |

---

## Documentation Index

### Project Documentation
- `/home/rec/uss-nav/README.md` — Project overview, architecture, build & deploy guide (Chinese)
- `/home/rec/uss-nav/instruction_description.md` — Complete Instruction.msg reference (English/Chinese)
- `/home/rec/uss-nav/CODEBASE.md` — This document

### Module READMEs
- `ws_main/src/README.md` — Workspace root (3 lines, minimal)
- `ws_main/src/script/ReadMe.md` — Script table (16 lines)
- `ws_main/src/utils/ReadMe.md` — Utility table (12 lines)
- `ws_main/src/planner/scene_graph/README.md` — Scene graph overview, build log (Chinese)
- `ws_main/src/planner/scene_graph/scripts/guide_LLMInterface.md` — LLM interface guide
- `ws_main/src/planner/exploration/exploration_manager/README.md` — Historical Fast-Planner docs
- `ws_main/src/network/NetBridgeForSwarm/README.md` + `README-zh.md` — Bridge docs (English/Chinese)
- `ws_main/src/network/NetBridgeForSwarm/swarm_ros_bridge/IMPLEMENTATION_SUMMARY.md` — 211 lines
- `ws_main/src/network/NetBridgeForSwarm/swarm_ros_bridge/POINT_CLOUD_FACTORY_INTEGRATION.md` — 260 lines
- `ws_main/src/network/NetBridgeForSwarm/swarm_ros_bridge/CHANGES_LOG.md` — 299 lines
- `yoloe/README.md` — YOLOE overview (English)
- `yoloe/docs/` — MkDocs site with ~300+ pages (API reference, model guides, tutorials)
- `Elastic-Tracker/README.md` — 95 lines, original paper + fork notes

### Key Source Files for Domain Understanding
- `ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg` — Central command message
- `ws_main/src/planner/exploration/exploration_manager/include/exploration_manager/mission_data.h` — FSM states
- `ws_main/src/planner/exploration/exploration_manager/include/exploration_manager/fast_exploration_fsm.h` — Top-level orchestrator
- `ws_main/src/planner/scene_graph/include/scene_graph/data_structure.h` — Core data types
- `ws_main/src/planner/scene_graph/include/scene_graph/scene_graph.h` — Scene graph class
- `ws_main/src/planner/scene_graph/include/scene_graph/skeleton_generation.h` — Skeleton topology
- `ws_main/src/planner/scene_graph/include/scene_graph/object_factory.h` — Object fusion pipeline
- `ws_main/src/planner/scene_graph/include/scene_graph/skeleton_cluster.h` — Area clustering
- `ws_main/src/planner/ego_plannerv3/plan_manage/include/plan_manage/ego_replan_fsm.h` — EGO FSM
- `ws_main/src/planner/ego_plannerv3/plan_env/include/plan_env/grid_map.h` — Occupancy/ESDF map

### Known Documentation Gaps
- No project-level LICENSE, CONTRIBUTING, or CHANGELOG
- No architecture decision records (ADRs)
- No OpenAPI/Swagger specs for ROS message interfaces
- Doxygen stubs are empty placeholders in several modules
- Hardcoded paths in shell scripts reduce portability
- API keys hardcoded in Python source (known security issue)
