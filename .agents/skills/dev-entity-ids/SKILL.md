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
    ├── TypeScript CLI (bun src/cli/)
    ├── Bun Server (:3000) — data collection + WebSocket
    ├── Vite Dev Server (:5173) — frontend
    ├── Mosquitto MQTT (:1883) — data bus
    │
    └── devel-docker
            ├── build ego-planner-sim      (base: ROS Noetic + planner)
            ├── build ego-planner-test     (base-image + MQTT bridge)
            └── run ego-test-<config>      (headless test containers)
                    │
                    └── MQTT → test/<id>/{odom,plan_result,data_disp}
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
| Visualization        | Three.js web frontend (:5173)       | None (telemetry only via MQTT)                |

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
bun test:run [scenario]
    │
    ├── cmdTestStop()         ← kill all previous ego-test-* containers
    ├── docker run -d ...     ← start batch of test containers
    └── (blocking poll)       ← docker ps + MQTT data → progress table

MQTT bridge (inside container) ──→ Mosquitto (:1883) ──→ Bun Server (:3000)
                                                              │
                                                           WebSocket
                                                              │
                                                              ▼
                                                     Frontend (:5173/3000)
```

### Data Chain

```
test-container → ROS topics
    ├── /drone_0_visual_slam/odom      → MQTT test/<id>/odom
    ├── /planning/ego_plan_result      → MQTT test/<id>/plan_result
    ├── /planning/data_display         → MQTT test/<id>/data_disp
    │
    └── MQTT → devel-host → Bun Server → WebSocket → Frontend 3D visualization
```

## 4. Container Runtime Configuration

### Common Docker flags (all test containers)

```
--rm                          auto-remove on exit
--gpus all                    GPU for pcl_render_node OpenGL
--ipc=host                    shared memory for inter-process
--security-opt seccomp=unconfined  ROS nodelet compatibility
--add-host host.docker.internal:host-gateway  MQTT bridge routing
```

### Environment variables per container

| Variable      | Default  | Description                  |
| ------------- | -------- | ---------------------------- |
| `TEST_ID`     | `default`| Unique test run identifier   |
| `MQTT_HOST`   | `host.docker.internal` | MQTT broker address |
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
| ego_mqtt_bridge.py  | Bridge status, topic registration      |
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
| Build test image              | `bun test:build`                           |
| Run test scenario             | `bun test:run [scenario]`                  |
| List running containers       | `bun test:status`                          |
| Stop all containers           | `bun test:stop`                            |
| Stop one container            | `bun test:stop velocity_sweep-0_6`         |
| Start dashboard               | `bun dashboard`                            |
| Start data server only        | `bun server`                               |
| Shell into test container     | `docker exec -it ego-test-<config> bash`   |
| Inspect container logs        | `docker logs ego-test-<config>`            |
| Inspect test results (API)    | `curl http://localhost:3000/api/tests`     |
| View MQTT traffic             | `mosquitto_sub -t 'test/#' -v`             |

## 7. Design Principles

1. **All operations are local** — No SSH, no remote devices, no cross-network deployment. devel-host and devel-docker are the only two runtime environments.

2. **Container is transient** — Containers are headless, ephemeral, and self-contained. They publish telemetry via MQTT and exit after `DURATION` seconds.

3. **Data flows outward** — Containers never read from MQTT or the frontend. Data flows one direction: ROS → MQTT → Server → WebSocket → Frontend.

4. **No RViz** — All visualization is through the web frontend (React + Three.js), served from devel-host via Vite dev server or built static files.

5. **Test configs are code** — Test scenarios and parameter sweeps are defined in `src/cli/scenarios.ts`.
