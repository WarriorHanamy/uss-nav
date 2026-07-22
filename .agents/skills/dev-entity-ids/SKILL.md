---
name: dev-entity-ids
description: Four runtime entities in the USS-NAV test infrastructure (devel-host, devel-docker, test-container, test-image), their roles, paths, and lifecycle. Use when an agent confuses host operations with container operations or needs to understand the test infrastructure topology.
---

# Runtime Entities

## 1. Entity IDs

| ID               | Description                                    | How to manipulate                          |
| ---------------- | ---------------------------------------------- | ----------------------------------------- |
| `devel-host`     | Local development workstation (x86_64 Arch)    | `bun <cmd>`, direct filesystem            |
| `devel-docker`   | Docker daemon running on devel-host            | `docker <cmd>`, `bun test:*`              |
| `test-container` | EGO Planner test container (Docker)            | `docker exec ego-test-<name> <cmd>`       |
| `test-image`     | Docker image snapshot                         | `docker build -t ego-planner-<role> .`    |

### Entity Topology

```
devel-host (x86_64 Arch Linux)
    │
    └── devel-docker
            ├── build ego-planner-sim      (base: ROS Noetic + planner)
            └── run uss-nav-{devel,test}-* (sim / headless test containers)
                    │
                    └── ROS logs + trace bags → .artifacts/traces/<TRACE_ID>/
```

### Host → Container Comparison

| Aspect               | devel-host                          | test-container                                |
| -------------------- | ----------------------------------- | --------------------------------------------- |
| OS                   | Arch Linux (x86_64)                 | Ubuntu 20.04 (x86_64 Docker)                  |
| ROS                  | None                                | ROS Noetic                                    |
| Runtime              | Bun + Node.js, Vite                 | roscore, roslaunch, ego-planner               |
| Network              | Direct internet + LAN               | Docker bridge (`host.docker.internal` → host) |
| Filesystem           | Full project tree                   | `/catkin_ws/` (image) + bind-mounts           |
| Display              | Wayland (Hyprland)                  | Xvfb :99 (headless)                           |
| Visualization        | None (RViz via ~/rviz_ws Docker)    | None (diagnostics via ROS logs + trace bags)  |

## 2. Workspace Path

The canonical workspace root is the repository root on devel-host:

```
/home/rec/uss-nav/    ← getRepoRoot()
```

No remote device path exists. All operations are local.

## 3. Pipeline Chains

### Build Chain

```
devel-host (Dockerfile) ──docker build──> test-image
                                              │
                                          docker run
                                              │
                                              ▼
                                        test-container
```

### Test Chain

```
TEST_ID=<id> DURATION=60 docker compose run --rm test
    │
    ├── entrypoint-test.sh    ← launch sim + planner, wait DURATION
    └── trace artifacts       ← .artifacts/traces/<TRACE_ID>/{roslaunch.log,ros/,run.bag}
```

### Data Chain

```
test-container → ROS topics + ROS logs
    ├── key decisions      → [MissionFSM]/[SUPER] ROS_INFO/WARN → rosout.log / fluentbit_roslog.log
    └── visualization data → run.bag (/tf, /bridge/Instruct, /planner/fsm_state, odom, pos_cmd, ...)
```

## 4. Container Runtime Configuration

### Common Docker flags (all test containers)

```
--rm                          auto-remove on exit
--gpus all                    GPU for pcl_render_node OpenGL
--ipc=host                    shared memory for inter-process
--security-opt seccomp=unconfined  ROS nodelet compatibility
```

### Environment variables per container

| Variable      | Default  | Description                  |
| ------------- | -------- | ---------------------------- |
| `TEST_ID`     | `default`| Unique test run identifier   |
| `FLIGHT_TYPE` | `2`      | EGO planner flight mode      |
| `MAX_VEL`     | `0.6`    | Max velocity [m/s]           |
| `MAX_ACC`     | `1.0`    | Max acceleration [m/s²]      |
| `OBS_NUM`     | `30`     | Number of obstacles in map   |
| `X_SIZE`      | `50`     | Map X size [m]               |
| `Y_SIZE`      | `30`     | Map Y size [m]               |
| `DURATION`    | `300`    | Test duration [s]            |

## 5. Logs

### Docker logs (container stdout)

```
docker logs ego-test-<config>
docker logs --tail 50 -f ego-test-<config>
```

Captures from nodes with `output="screen"`:

| Node                | What to look for                       |
| ------------------- | -------------------------------------- |
| entrypoint-test.sh  | Planner readiness, test timing         |
| exploration_node    | FSM state, planning results, errors    |

### ROS logs (inside container)

```
docker exec ego-test-<config> tail -100 /root/.ros/log/latest/master.log
docker exec ego-test-<config> grep ERROR /root/.ros/log/latest/roslaunch-*.log
```

### Test results (on devel-host)

```
_site/test-results/<scenario>/<config>.json
```

## 6. Quick Reference

| Operation                     | Command                                    |
| ----------------------------- | ------------------------------------------ |
| Build devel image             | `docker compose build devel`               |
| Compile workspace             | `docker compose run --rm build`            |
| Run headless test             | `TEST_ID=<id> DURATION=60 docker compose run --rm test` |
| List running containers       | `docker ps --filter name=uss-nav`          |
| Stop all containers           | `docker rm -f $(docker ps -aq --filter name=uss-nav)` |
| Shell into test container     | `docker exec -it uss-nav-devel-local bash` |
| Inspect container logs        | `docker logs uss-nav-devel-local`          |
| Inspect trace artifacts       | `ls .artifacts/traces/<TRACE_ID>/`         |

## 7. Design Principles

1. **All operations are local** — No SSH, no remote devices, no cross-network deployment. devel-host and devel-docker are the only two runtime environments.

2. **Container is transient** — Containers are headless, ephemeral, and self-contained. They run for `DURATION` seconds and write trace artifacts before exiting.

3. **Data flows outward** — Diagnostics flow one direction: ROS logs → `.artifacts/traces/<TRACE_ID>/` (rosout.log, fluentbit_roslog.log, run.bag). No telemetry bus; text-diagnosable issues must not require the bag.

4. **RViz in separate container** — Visualization via `~/rviz_ws` Docker container, connected to the shared ROS master on localhost:11311.
