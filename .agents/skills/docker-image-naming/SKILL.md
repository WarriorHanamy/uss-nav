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

| Role                     | Image ref               | Dockerfile                  | Base                  |
| ------------------------ | ----------------------- | --------------------------- | --------------------- |
| Unified sim/build/test   | `ego-planner-sim:latest` | `docker/Dockerfile.devel`   | `ros:noetic-ros-base` |

The historical `ego-planner-test` image (separate MQTT-bridge variant) is retired:
`devel`, `build`, and `test` compose services all run the single `ego-planner-sim`
image. Source compilation happens at runtime into `.artifacts/`, not in the image.

## Build Commands

```bash
# Build the single image (system deps only, no source compile)
docker compose build devel

# Compile ROS workspace at runtime (artifacts persist to .artifacts/)
docker compose run --rm build
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

### Entry points (same image, different roles)

```
devel   → /entrypoint.sh        (interactive sim)
test    → /entrypoint-test.sh   (headless DURATION-bounded run + trace)
build   → catkin build into .artifacts/{build,devel}
```
