---
name: doxygen-docs
description: Add Doxygen-style unit annotations to C++ header files in the EGO Planner v3 codebase. Covers install (Arch/Ubuntu), comment conventions with [unit] notation, Doxyfile config, and Bun render wrapper. Use when annotating function signatures with physical units, generating API docs, or adding @param/@return blocks.
---

# Doxygen Documentation

## Install

```bash
# Arch
sudo pacman -S doxygen graphviz
# Ubuntu
sudo apt install doxygen graphviz
```

No Docker needed.

## Comment Convention

Place a Doxygen block before each public function/class definition:

```cpp
/**
 * Single-sentence description of the function's physical meaning.
 *
 * @param[in]  name  Description with unit [m/s]
 * @param[out] name  Description with unit [m]
 * @return Description with unit [m/s]
 */
```

Rules:
- One sentence, no `@brief`
- `@param` must include direction: `[in]`, `[out]`, or `[inout]`
- Every `@param` and `@return` includes unit in square brackets
- Total block: 4-6 lines max
- Inline `//` comments inside function bodies: only for non-obvious physics

## Units Reference

| Quantity      | Unit       |
|---------------|------------|
| position      | [m]        |
| velocity      | [m/s]      |
| acceleration  | [m/s^2]    |
| jerk          | [m/s^3]    |
| snap          | [m/s^4]    |
| yaw / angle   | [rad]      |
| angular vel   | [rad/s]    |
| angular acc   | [rad/s^2]  |
| time          | [s]        |
| cost          | [--]       |
| gradient      | [--/m]     |
| voxel index   | [voxel]    |
| step size     | [m]        |
| clearance     | [m]        |

## File Annotation Priority

P0 — Core API:
- `ws_main/src/planner/ego_plannerv3/plan_manage/include/plan_manage/planner_manager.h`
- `ws_main/src/planner/ego_plannerv3/traj_opt/include/optimizer/poly_traj_optimizer.h`

P1 — Execution & Infrastructure:
- `ws_main/src/planner/ego_plannerv3/plan_manage/include/plan_manage/traj_server.h`
- `ws_main/src/planner/ego_plannerv3/plan_env/include/plan_env/grid_map.h`
- `ws_main/src/planner/ego_plannerv3/traj_opt/include/optimizer/poly_traj_utils.hpp`

P2 — FSM & Shared Types:
- `ws_main/src/planner/ego_plannerv3/plan_manage/include/plan_manage/ego_replan_fsm.h`
- `ws_main/src/utils/traj_utils/include/traj_utils/plan_container.hpp`
- `ws_main/src/planner/ego_plannerv3/path_searching/include/path_searching/dyn_a_star.h`

P3 — Internal Helpers:
- `ws_main/src/planner/ego_plannerv3/map_interface/include/map_interface/map_interface.hpp`
- `ws_main/src/planner/ego_plannerv3/traj_opt/include/optimizer/poly_traj_utils.hpp` (internal methods)

## Doxyfile (project root)

Minimal config — see `Doxyfile` in project root. Key settings:

```
PROJECT_NAME           = "USS-NAV EGO Planner v3"
OUTPUT_DIRECTORY       = docs/api
INPUT                  = ws_main/src/planner/ego_plannerv3 \
                         ws_main/src/utils/traj_utils
EXTRACT_ALL            = YES
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
HAVE_DOT               = YES
CALL_GRAPH             = YES
CALLER_GRAPH           = YES
```

## Render via Bun

```bash
bun run doxygen            # generate docs
bun run doxygen --open     # generate and open in browser
```

## Workflow

1. Install doxygen + graphviz via system package manager.
2. Annotate header files following the Comment Convention.
3. Run `bun run tools/doxygen/render.ts` to generate HTML docs.
4. Open `docs/api/html/index.html` in a browser.
