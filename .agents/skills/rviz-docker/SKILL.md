---
name: rviz-docker
description: Diagnose and maintain RViz running in this project's Docker simulation, including an unresponsive 3D viewport, X11/Qt/OpenGL display failures, RViz tool and view configuration, image rebuilds, and repeatable container restart behavior. Use when working on sim_ego.rviz, sim_bringup RViz launch behavior, Dockerfile or entrypoint display setup, tools/infra.ts Docker settings, or the Bun docker:build and docker:run commands.
---

# RViz Docker

Keep RViz configuration, container lifecycle, and host display integration consistent. Treat `tools/infra.ts` as the source of truth for Docker CLI options; keep `package.json` scripts as thin compatibility aliases.

## Inspect in order

1. Read `package.json`, `tools/infra.ts`, `Dockerfile`, and `docker/entrypoint.sh`.
2. Read `ws_main/src/planner/sim_bringup/launch/sim_ego_main.launch` and `ws_main/src/planner/sim_bringup/rviz/sim_ego.rviz`.
3. Determine which boundary failed:
   - No RViz window: inspect `DISPLAY`, X11 socket, Xauthority, Qt, and launch logs.
   - Window and side panels work but the 3D viewport ignores drag: inspect RViz `Tools` first.
   - Viewport receives input but draws incorrectly: inspect GLX/NVIDIA and OGRE logs.
   - A second run conflicts or leaves an old simulation: inspect the infra restart invariant.

## Preserve the RViz input invariant

In `Visualization Manager`, retain all of the following:

```yaml
Name: root
Tools:
  - Class: rviz/MoveCamera
  - Class: rviz/Interact
    Hide Inactive Objects: true
  - Class: rviz/Select
  - Class: rviz/SetGoal
    Topic: /move_base_simple/goal
Value: true
```

Keep `MoveCamera` first so camera drag works immediately after loading. If side panels accept mouse input, do not initially blame Docker/X11; Qt is receiving pointer events and a missing or wrong active RViz tool is the stronger hypothesis.

## Preserve the container invariant

Maintain one fixed image name and one fixed container name in `tools/infra.ts`. Before `docker run`, execute `docker rm -f <container>` and tolerate the absence of the previous container. Start the replacement with the same `--name` and `--rm`.

Do not duplicate Docker flags in `package.json`. Expose both interfaces:

```bash
bun infra docker build
bun infra docker run
bun run docker:build
bun run docker:run
```

The aliases must delegate to `tools/infra.ts`.

## Rebuild and validate

RViz launch and config files are copied into the image. After changing them, rebuild before testing:

```bash
bun run docker:build
bun run docker:run
```

Validate proportionally:

- Run `bun tools/infra.ts` and confirm invalid usage exits nonzero with concise help.
- Run `bun run docker:build` after Dockerfile, entrypoint, launch, or RViz config changes.
- Inspect the built image when necessary to prove the expected config was copied.
- Ask the user to confirm actual GUI pointer interaction; logs alone cannot prove drag behavior.
- Preserve unrelated working-tree changes.

## Read launch logs

The entrypoint writes roslaunch output to `/tmp/roslaunch.log` inside the running container. Search for `rviz`, `error`, `warn`, `fatal`, `xcb`, `display`, `GLX`, and `OGRE`. Use the Bun infra lifecycle for normal starts; use direct Docker inspection only for diagnostics that the infra CLI does not expose.
