# uss-nav-super

UAV autonomous exploration, semantic scene understanding, and multi-drone
coordination monorepo (ROS Noetic, x86_64 Docker). This is the simulation
and development environment — see [l3-uss-nav-arm64](~/l3-uss-nav-arm64)
for the **arm64 Jetson deployment slice**.

## Structure

```
uss-nav-super/
├── ws_main/src/                    # ROS catkin workspace source
│   ├── planners/
│   │   ├── ego_planner/          # plan_manage, plan_env, path_searching, traj_opt
│   │   ├── super_planner/        # RogMap, A*, corridor generator, L-BFGS optimizer,
│   │   │                         #   Astar, CIRI, backup traj, yaw traj, exp traj
│   │   ├── exploration/          # frontier selection, explore/goal mode switch
│   │   └── tracker/              # target tracking
│   ├── perception/
│   │   ├── scene_graph/          # object fusion, skeleton, LLM interface, 3D search
│   │   ├── yoloe/                # YOLOE open-vocabulary detection (needs CUDA)
│   │   └── camera_fov/           # camera FOV utilities
│   ├── mission_executive/        # MissionFSM, instruction dispatch, planner mux
│   ├── uav_simulator/            # so3 quadrotor simulation, map generator, depth/cloud
│   └── utils/                    # quadrotor_msgs, traj_utils, box_odom_estimator
├── bringup_test/                 # test launch, params, scripts
├── docker/                       # Dockerfile.devel, Dockerfile.release, entrypoints
├── docker-compose.yml            # devel / build / test / release lifecycle
├── docs/                         # VIEW, EGO, SCENEGRAPH, SUPER, DATA_FLOW, CODEBASE
├── tools/                        # md2html, doxygen, opt_cases
├── manifest.yaml                 # vendored component provenance
├── CHANGELOG.md
└── ADR.md                        # architecture decision records
```

## Build & Run (simulation)

Docker-based, **no host ROS install needed**.

```bash
docker compose build devel        # build system-deps image
docker compose run --rm build     # catkin build → .artifacts/{devel,build}/
docker compose run --rm devel     # launch simulation (needs X11 DISPLAY)
```

### Launch modes

| Mode                   | Entry                                              | Description                                  |
|------------------------|----------------------------------------------------|----------------------------------------------|
| scenegraph-ego (default)| `roslaunch bringup_test sim_scenegraph_main.launch` | PCD map + SceneGraph → EGO Planner           |
| random-sim             | `roslaunch bringup_test sim_random_main.launch`     | procedural map + simulator + EGO Planner     |

Switch via `LAUNCH_MODE=random docker compose up devel`.

## Instruction types

Send to `/bridge/Instruct` (`quadrotor_msgs/Instruction`):

| ID | Type                   | Description              |
|----|------------------------|--------------------------|
| 1  | `TURN_OBJECT_NAV`        | full exploration         |
| 2  | `TURN_OBJECT_ID_NAV`     | nav to object ID         |
| 3  | `TURN_REGULAR_EXPLORATION`| map-only exploration     |
| 5  | `TURN_GOAL`              | point-to-point           |
| 7  | `TURN_WAYPOINT_NAV`      | waypoint navigation      |

```bash
# Navigate to scene graph object #2:
rostopic pub -1 /bridge/Instruct quadrotor_msgs/Instruction \
  "{instruction_type: 2, target_obj_id: 2, source_task_id: 2}"
```

## Deployment sub-repos

| Repo                      | Platform | Scope                                  | Upstream commit |
|---------------------------|----------|----------------------------------------|-----------------|
| `l3-uss-nav-arm64`          | arm64 Jetson | scene_graph + super_planner (no YOLOE, no sim) | `44c7d33b`        |

## Dependencies

- Docker + Docker Compose
- ROS Noetic (containerized)
- NVIDIA GPU + CUDA (YOLOE detection)
- `~/rviz_ws` (optional, visualization)

## Documentation

| File                      | Description                                 |
|---------------------------|---------------------------------------------|
| `docs/VIEW.md`              | architecture overview                       |
| `docs/SUPER.md`             | super_planner design                        |
| `docs/SCENEGRAPH.md`        | scene graph environment representation      |
| `docs/DATA_FLOW.md`         | data flow between modules                   |
| `docs/PACKAGE_DEPS.md`      | package dependency graph (Mermaid)          |
| `docs/EGO.md`               | EGO planner real-time trajectory optimization |
| `docs/CODEBASE.md`          | full codebase reference                     |
