# Architecture Decision Records

| #       | Title                         | Status        | File                                |
| ------- | ----------------------------- | ------------- | ----------------------------------- |
| ADR-0001 | TypeScript Visualization replaces RViz | Accepted | `docs/adr/ADR-0001-typescript-viz.md` |
| ADR-0002 | JSONL for Data Persistence    | Accepted      | `docs/adr/ADR-0002-jsonl-persistence.md` |
| ADR-0003 | Docker Container Isolation    | Accepted      | `docs/adr/ADR-0003-container-isolation.md` |
| ADR-0004 | scene_graph/active_perception as build-time deps | Accepted | `docs/adr/ADR-0004-build-deps.md` |
| ADR-0005 | Test Data Pipeline Architecture | Accepted      | `docs/adr/ADR-0005-test-data-pipeline.md` |
| ADR-0006 | Post-Processing Visualization Tab | Accepted      | `docs/adr/ADR-0006-post-processing-viz.md` |
| ADR-0007 | Realsense D455 Body-Centric Image | Accepted      | `docs/adr/ADR-0007-realsense-d455.md` |

> Each ADR is stored in `docs/adr/ADR-NNNN-title.md`.  The content of
> ADR-0001 through ADR-0004 is preserved verbatim below (moved from the
> original monolithic file).

---

## ADR-0001: TypeScript Visualization replaces RViz

**Date**: 2026-06-26
**Status**: Accepted

### Context

EGO-Planner simulations were visualized exclusively through RViz running inside a Docker
container with X11 forwarding. This required:
- A running X11/Wayland display on the host
- GPU pass-through to the container for OpenGL rendering
- No remote access to simulation state
- No easy way to aggregate data from concurrent test containers

### Decision

Replace RViz with a **TypeScript-based web visualization stack**:

| Layer        | Technology                 | Role                            |
| ------------ | -------------------------- | ------------------------------- |
| Runtime      | Bun 1.3                    | CLI + HTTP+WebSocket server     |
| 3D Rendering | Three.js + @react-three/fiber | Browser-side trajectory display |
| UI Framework | React 19                   | Dashboard, stats, comparisons   |
| Data Bus     | MQTT (Mosquitto)           | Container → host telemetry      |
| Persistence  | JSONL files on disk        | _site/test-results/<id>/*.jsonl |

### Consequences

**Positive**:
- Any browser on the LAN can view simulation state without X11 forwarding
- Multiple concurrent test container data automatically aggregated
- Historical runs preserved on disk (server restart does not lose data)
- TypeScript is the single language for CLI, server, and frontend

**Negative**:
- Browser cannot match RViz's real-time rendering performance for very large point clouds
- MQTT adds a network hop between container and display (latency ~1ms, negligible)

---

## ADR-0002: JSONL for Data Persistence (single-file, append-only)

**Date**: 2026-06-26
**Status**: Accepted (with review)

### Context

Each test container publishes telemetry over MQTT. The Bun server receives
this data and must persist it for historical comparison, recovery after
restart, and frontend loading of completed runs.

### Decision

Use **JSONL (JSON Lines, one JSON object per line, append-only)** as the
persistence format. Each test run produces files:

```
_site/test-results/<scenario>/odom.jsonl
_site/test-results/<scenario>/plan_result.jsonl
_site/test-results/<scenario>/data_disp.jsonl
```

### Consequences

**Positive**: Append-only O(1), human-readable, no schema migration, trivially
streamable.

**Negative**: No indexing (linear scan), larger than binary formats, no type
validation.

### Review Trigger

Revisit when: `_site/test-results/` exceeds 10 GB, read latency exceeds 500ms,
or concurrent write contention measurable.

---

## ADR-0003: Docker Container Isolation with Env-Var Parameterization

**Date**: 2026-06-26
**Status**: Accepted

### Decision

- Each container is fully self-contained (no bind mounts)
- All parameter variation done through environment variables
- Containers are headless (Xvfb :99), CPU/GPU bound only
- Each container has its own ROS master (localhost:11311)

---

## ADR-0004: scene_graph and active_perception kept as build-time dependencies

**Date**: 2026-06-26
**Status**: Accepted (workaround)

### Decision

Keep `scene_graph/` and `active_perception/` in the repository so Docker
build succeeds. The Dockerfile follows `COPY → build → rm -rf`.
