---
name: bringup-naming
description: Naming conventions for ROS launch files and config files. Use when creating new launch/config files, referencing ROS paths, or extending the simulation bringup directory.
---

# Bringup Naming Convention (General ROS)

## Abstract Rules

### Launch files

```
{action}[_{variant}].launch
```

| Placeholder  | Meaning                      | Examples                              |
| ------------ | ---------------------------- | ------------------------------------- |
| `{action}`   | Subsystem / operation         | `sim_map`, `sim_sim`, `sim_planner`   |
| `{variant}`  | Optional behavioural variant  | `main` (implicit), `debug`, `headless` |

### Config / Parameter files

```
{domain}_{file}.{ext}
```

| Placeholder  | Meaning                      | Examples                              |
| ------------ | ---------------------------- | ------------------------------------- |
| `{domain}`   | Subsystem domain              | `sim_ego`, `sim_map`, `sim_control`   |
| `{file}`     | Descriptive name              | `planner`, `map`, `control`, `camera` |
| `{ext}`      | Format                        | `yaml`, `xml`, `json`                 |

## Concrete Examples (this project)

### Launch files (in `ws_main/src/planner/sim_bringup/launch/`)

| File                     | Includes                             |
| ------------------------ | ------------------------------------ |
| `sim_ego_main.launch`    | Entry: map + sim + planner + rviz    |
| `sim_ego_map.launch`     | Procedural random forest map         |
| `sim_ego_sim.launch`     | Quadrotor dynamics + controls + LiDAR|
| `sim_ego_planner.launch` | EGO Planner FSM + command mux        |

### Config files (in `ws_main/src/planner/sim_bringup/params/`)

| File                     | Purpose                              |
| ------------------------ | ------------------------------------ |
| `sim_ego_planner.yaml`   | Planner params (A*, MINCO, FSM)      |
| `sim_ego_map.yaml`       | Map generator params (size, obstacles)|
| `sim_ego_control.yaml`   | SO(3) controller gains               |
| `sim_ego_camera.yaml`    | Camera intrinsics                     |

## Design Principles

1. **Domain prefix** — The first segment identifies the subsystem: `sim_` for simulation, `ego_` for ego-planner specific.

2. **File extension signals content** — `.launch` for orchestration, `.yaml` for structured config, `.msg` for ROS message definitions.

3. **Consistent `$(find <pkg>)` paths** — All ROS references use package-relative paths:

   ```
   $(find sim_bringup)/launch/...
   $(find sim_bringup)/params/...
   $(find sim_bringup)/rviz/...
   ```

4. **Launch files compose** — `sim_ego_main.launch` is the entry point; it includes specialized launch files for each subsystem rather than duplicating logic.
