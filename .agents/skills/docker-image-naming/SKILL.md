---
name: docker-image-naming
description: Canonical Docker image naming for USS-NAV test infrastructure. Use when naming, building, tagging, or referencing Docker images for ego-planner simulation and test containers.
---

# Docker Image Naming

## Convention

```text
<image>:<variant>
```

| Field      | Meaning                                       |
| ---------- | --------------------------------------------- |
| `<image>`  | Logical image role                            |
| `<variant>`| Build variant (`latest`, `no-cache`, `debug`) |

## Current Images

| Role                  | Image ref               | Dockerfile                  | Base                  |
| --------------------- | ----------------------- | --------------------------- | --------------------- |
| Base simulation       | `ego-planner-sim:latest` | `Dockerfile` (root)         | `ros:noetic-perception` |
| Test (adds MQTT bridge) | `ego-planner-test:latest` | `docker/Dockerfile.test`  | `ego-planner-sim`     |

## Build Commands

```bash
# Build base simulation image
docker build -t ego-planner-sim .

# Build test image (based on ego-planner-sim)
docker build -f docker/Dockerfile.test -t ego-planner-test .

# Via Bun CLI
bun test:build              # builds ego-planner-test
bun test:build no-cache     # --no-cache
```

## Tag Rules

1. Use explicit tags, not implicit `latest` for CI reproducibility.
2. No registry prefix (all images are local to the devel-host Docker daemon).
3. Arch is implicit (x86_64, as devel-host is x86_64 Arch Linux).
4. No CUDA version in tag (GPU is detected at runtime via `--gpus all`).
5. `no-cache` suffix means the image was built with `--no-cache` for debug builds.

## Image Contents

### `ego-planner-sim`

```
FROM ros:noetic-perception
  ├── ROS Noetic base
  ├── Planning packages (plan_env, path_searching, traj_opt, plan_manage)
  ├── Simulator packages (so3_quadrotor_simulator, so3_control, local_sensing)
  ├── Map generator (random_forest)
  ├── Exploration FSM (exploration_manager, perception_utils)
  ├── Support packages (quadrotor_msgs, traj_utils, uav_utils)
  └── Entrypoint → /entrypoint.sh
```

### `ego-planner-test`

```
FROM ego-planner-sim
  ├── python3-pip + paho-mqtt
  ├── docker/bridge/ego_mqtt_bridge.py  → /bridge/
  └── docker/entrypoint-test.sh          → /entrypoint.sh
```
