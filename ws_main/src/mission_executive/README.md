# mission_executive

High-level decision FSM for autonomous navigation.

Receives `quadrotor_msgs::Instruction` on `/Instruct` and dispatches to the
appropriate planner (EGO exploration, elastic tracker, VLA swarm, DF demo, etc.)
via `planner_cmd_mux`. The core state machine `MissionFSM` runs in
`mission_executive_node`.

## Architecture

```
Instruction  →  MissionFSM  →  callExplorationPlanner()
                               callTrackPlanner()
                               callExplorationLLMPlanner()
                               VLA_Search submachine
                               DF_Demo submachine
                               Panorama rotation
                               Yaw scan
                    →  planner_cmd_mux  →  EGO planner / Elastic tracker
```

## FSM States

| State                  | Purpose                          |
| ---------------------- | -------------------------------- |
| INIT                   | Startup, wait for initialization |
| WAIT_TRIGGER           | Wait for external trigger        |
| WARM_UP                | Sensor/state warm-up             |
| PLAN_EXPLORE           | Frontier-based exploration       |
| LLM_PLAN_EXPLORE       | LLM-guided exploration           |
| APPROACH_EXPLORE       | Navigate to exploration goal     |
| PLAN_TRACK             | Plan tracking path               |
| APPROACH_TRACK         | Follow tracking target           |
| THINKING               | LLM planning deliberation        |
| YAW_HANDLE             | Yaw scan for wider FOV           |
| FIND_TERMINATE_TARGET  | Locate terminate target          |
| GO_TARGET_OBJECT       | Navigate to object               |
| GO_TARGET_WITH_WAYPOINT| Navigate via waypoints           |
| DF_DEMO                | Demonstration flight             |
| VLA_SEARCH_*           | VLA Swarm search submachine      |
| FINISH / STOP          | Terminal states                  |

## Binaries

| Binary                     | Role                              |
| -------------------------- | --------------------------------- |
| `mission_executive_node`     | Main FSM + EGO planner + map     |
| `planner_cmd_mux`           | EGO/Elastic command multiplexer  |
| `rc_replan_trigger`         | RC channel replan trigger        |

## Namespace

All internal types are under `namespace mission_executive`.

## Key Topics

| Topic                               | Type                            | Direction |
| ----------------------------------- | ------------------------------- | --------- |
| `/Instruct`                         | `Instruction`                   | Input     |
| `/planning/ego_plan_result`         | `EgoPlannerResult`              | Input     |
| `/move_base_simple/goal`            | `PoseStamped`                   | Input     |
| `/planner_mux/mode`                 | `String`                        | Output    |
| `~/planning/pos_cmd`                | `PositionCommand`               | Output    |
