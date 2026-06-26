---
name: docker-artifact-migration
description: Migrate a ROS package from in-Dockerfile catkin_make to pre-built artifacts. Four-phase loop: experiment in container → extract + write MANIFEST → integrate into Dockerfile → build + verify. Use when eliminating catkin_make from the Dockerfile or adding a pre-built ROS package.
---

# Docker Artifact Migration

## Overview

Replace `catkin_make` in the Dockerfile with pre-built artifacts from a working container.

**Loop**: experiment (container) → extract + manifest → integrate → build → verify.

## Infrastructure

| Component                     | Purpose                                              | Location                    |
| ----------------------------- | ---------------------------------------------------- | --------------------------- |
| `docker/Dockerfile`           | Base simulation image                                | `Dockerfile` (repo root)    |
| `docker/Dockerfile.test`      | Test image (adds MQTT bridge)                        | `docker/Dockerfile.test`    |
| `docker/entrypoint-test.sh`   | Test entrypoint (env var parameterization)           | `docker/entrypoint-test.sh` |
| `docker/bridge/ego_mqtt_bridge.py` | ROS→MQTT telemetry bridge                       | `docker/bridge/`            |

## Phase 1: Experiment

Goal: determine minimal artifacts needed from a built workspace.

```bash
# 1. Start from current (catkin_make) image
docker run -it --rm ego-planner-sim bash

# 2. Inside container — iterate:
cp -a /catkin_ws/devel/lib/<pkg>/    /opt/ros/noetic/lib/<pkg>/
cp -a /catkin_ws/devel/include/<pkg>/ /opt/ros/noetic/include/<pkg>/
cp -a /catkin_ws/devel/share/<pkg>/   /opt/ros/noetic/share/<pkg>/
cp    /catkin_ws/src/<pkg>/package.xml /opt/ros/noetic/share/<pkg>/
cp -a /catkin_ws/devel/lib/python3/dist-packages/<pkg>/ \
      /opt/ros/noetic/lib/python3/dist-packages/<pkg>/ 2>/dev/null || true

# Fix cmake hardcoded paths
find /opt/ros/noetic/share/<pkg>/cmake -name '*.cmake' -exec \
  sed -i 's|/catkin_ws/devel|/opt/ros/noetic|g' {} \;

# Test if node still launchable
mv /catkin_ws/devel /catkin_ws/devel.bak
roslaunch sim_bringup sim_ego_main.launch &
```

## Phase 2: Extract & Write Manifest

```bash
# From the Docker image
CID=$(docker create ego-planner-sim)
docker cp "$CID:/catkin_ws/devel/lib/<pkg>/."          _vendor/<pkg>/lib/
docker cp "$CID:/catkin_ws/devel/include/<pkg>/."      _vendor/<pkg>/include/
docker cp "$CID:/catkin_ws/devel/share/<pkg>/."        _vendor/<pkg>/share/
docker cp "$CID:/catkin_ws/src/<pkg>/package.xml"      _vendor/<pkg>/share/
docker rm "$CID"
```

Write `_vendor/MANIFEST`:

```
_vendor/<pkg>/lib/      /opt/ros/noetic/lib/<pkg>/
_vendor/<pkg>/include/  /opt/ros/noetic/include/<pkg>/
_vendor/<pkg>/share/    /opt/ros/noetic/share/<pkg>/
```

## Phase 3: Integrate into Dockerfile

Replace `catkin_make` block with `COPY`:

```dockerfile
# Before:
# COPY <pkg>/ /catkin_ws/src/<pkg>/
# RUN catkin_build ...

# After:
COPY _vendor/<pkg>/lib/     /opt/ros/noetic/lib/<pkg>/
COPY _vendor/<pkg>/include/ /opt/ros/noetic/include/<pkg>/
COPY _vendor/<pkg>/share/   /opt/ros/noetic/share/<pkg>/
RUN find /opt/ros/noetic/share/<pkg>/cmake -name '*.cmake' -exec \
    sed -i 's|/catkin_ws/devel|/opt/ros/noetic|g' {} \;
```

## Phase 4: Verify

```bash
docker build -t ego-planner-sim .             # rebuild
docker run -it --rm ego-planner-sim bash        # shell in
ls /opt/ros/noetic/lib/<pkg>/                  # artifacts present
```

## Risk Points

| # | Symptom                                      | Root Cause                                      | Fix                                       |
| - | -------------------------------------------- | ----------------------------------------------- | ----------------------------------------- |
| 1 | `Cannot locate node of type [x] in package [y]` | cmake Config hardcodes `/catkin_ws/devel`       | `sed` replace in `*.cmake` (Phase 3)      |
| 2 | `[package_name] is neither a launch file...` | `ROS_PACKAGE_PATH` missing workspace             | Source `/catkin_ws/devel/setup.bash`      |
| 3 | `find_package` fails for downstream packages | cmake include resolves to old devel path        | Same as #1                                |
| 4 | Python import fails at runtime               | Artifacts in wrong python path                  | Must use `dist-packages/` (Ubuntu), not `site-packages/` |
