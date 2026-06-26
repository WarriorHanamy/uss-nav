---
name: docker-dev-mounts
description: Development container mount design for USS-NAV Docker test containers. Covers bind mount patterns, permission leakage, and the distinction between file mounts and runtime flags. Use when configuring container volumes, debugging permission errors on host after container writes, or designing the test container mount layout.
---

# Docker Dev Mounts

## 1. Mount Philosophy

Test containers use **no bind mounts**. The image is self-contained — all artifacts
(ROS packages, configuration, launch files) are baked into the image at build time.

This is the opposite of a development environment where live file editing is needed.
Test containers are **ephemeral and immutable**: every run starts from the same
immutable image.

## 2. Why No Mounts

| Requirement                     | How it's satisfied              |
| ------------------------------- | ------------------------------- |
| Parameter variation per test    | Environment variables           |
| Map size / obstacle config      | `OBS_NUM`, `X_SIZE`, `Y_SIZE` env vars |
| Planner tuning                  | `MAX_VEL`, `MAX_ACC`, `FLIGHT_TYPE`    |
| Test identity                   | `TEST_ID` env var               |
| Telemetry destination           | `MQTT_HOST` env var             |
| Log access                      | `docker logs` / `docker exec`   |

The entrypoint script (`docker/entrypoint-test.sh`) receives these env vars and
generates a custom map YAML at runtime:

```bash
sed -e "s/obs_num:.*/obs_num: ${OBS_NUM}/" \
    /catkin_ws/src/sim_bringup/params/sim_ego_map.yaml \
    > /tmp/sim_ego_map_${TEST_ID}.yaml
```

## 3. Runtime Flags (Not File Mounts)

| Flag / Config                       | Kind         | Purpose                              |
| ----------------------------------- | ------------ | ------------------------------------ |
| `--gpus all`                        | Docker flag  | GPU access for pcl_render_node OpenGL |
| `--ipc=host`                        | Docker flag  | Shared memory for inter-process       |
| `--security-opt seccomp=unconfined` | Docker flag  | ROS nodelet syscall compatibility     |
| `--add-host host.docker.internal:host-gateway` | Docker flag | Container → Host route    |
| `-e MQTT_HOST=host.docker.internal` | Env var      | MQTT broker addressing               |

Never call these "mounts" in code or docs.

## 4. Bind Mount Permission Leakage

Although test containers have no bind mounts, the general pattern is documented
here for reference in case a debug session adds one.

Container root (UID 0) = host root (UID 0) on bind mounts. If a container writes
to a bind-mounted path, files become root-owned on the host.

### Detection

| Symptom                                | Likely Cause                         |
| -------------------------------------- | ------------------------------------ |
| `ls -la` shows `root root` on host     | Container wrote to bind mount        |
| `Permission denied` editing host files | Files owned by root                  |

### Fix-up

```bash
sudo chown -R $USER:$USER <mount-path>
```

### Always-On Rules

1. Never let the container write to a development bind mount path unless the
   host user is prepared to fix up ownership.
2. Logs and generated data should go to container-internal paths (e.g., `/root/.ros/log`).
3. Test containers should prefer env-var parameterization over file mounts.
