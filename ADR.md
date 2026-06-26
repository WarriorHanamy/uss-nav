# Architecture Decision Records

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
- MQTT adds a network hop between container and display (latency ~1ms, negligible for aggregate metrics)

### Notes

Future performance concern: JSONL read/write may become a bottleneck at very large
scale (>100 concurrent containers). See ADR-0002.

---

## ADR-0002: JSONL for Data Persistence (single-file, append-only)

**Date**: 2026-06-26  
**Status**: Accepted (with review)  

### Context

Each test container publishes telemetry (odometry, plan results, debug data) over MQTT.
The Bun server receives this data and must persist it for:
1. Historical comparison across test runs
2. Recovery after server restart
3. Frontend loading of completed test runs

### Decision

Use **JSONL (JSON Lines, one JSON object per line, append-only)** as the persistence
format. Each test run produces files:

```
_site/test-results/<scenario>/odom.jsonl
_site/test-results/<scenario>/plan_result.jsonl
_site/test-results/<scenario>/data_disp.jsonl
```

Each line is a raw JSON payload received from MQTT (no wrapping envelope).

### Consequences

**Positive**:
- Append-only writes are O(1) — no read-modify-write cycle
- Trivially human-readable with `less`, `grep`, `jq`
- No schema migration needed — lines are self-describing
- Trivially streamable — read with `readline` or split on `\n`

**Negative**:
- No indexing — filtering a 10,000-line file requires linear scan (acceptable at current scale)
- Larger on disk than binary formats (protobuf, msgpack, flatbuffers)
- No type validation — malformed lines are silently skipped

### Review Trigger

Revisit this decision when any of:
- `_site/test-results/` exceeds 10 GB
- Read latency for `/api/test/:id` exceeds 500ms
- Concurrent write contention becomes measurable (>10 concurrent MQTT streams)

### Candidate Replacements

If review is triggered, evaluate in this order:

1. **SQLite** — single file, indexed queries, robust concurrent writes
2. **FlatBuffers** — zero-copy read, schema-enforced
3. **Apache Parquet** — columnar, good for analytical queries across runs

---

## ADR-0003: Docker Container Isolation with Env-Var Parameterization

**Date**: 2026-06-26  
**Status**: Accepted  

### Context

Each test run needs a different combination of parameters (map size, obstacle count,
flight speed, etc.). We need to run multiple containers simultaneously without file
system conflicts.

### Decision

- Each container is fully self-contained (no bind mounts)
- All parameter variation is done through **environment variables**:
  `OBS_NUM`, `X_SIZE`, `Y_SIZE`, `MAX_VEL`, `MAX_ACC`, `FLIGHT_TYPE`, `DURATION`, `TEST_ID`
- The entrypoint script generates a custom map YAML at runtime from these env vars
- Containers are headless (Xvfb :99), CPU/GPU bound only
- Each container has its own ROS master (localhost:11311 inside the container)

### Consequences

**Positive**: No file system conflicts, truly parallel runs, trivial to restart.

**Negative**: Image rebuild needed to change environment logic (entrypoint changes).

**Fix for negative**: Entrypoint is not in base image — it's added by `Dockerfile.test`
which is fast to rebuild.

---

## ADR-0004: scene_graph and active_perception kept as build-time dependencies

**Date**: 2026-06-26  
**Status**: Accepted (workaround)  

### Context

`exploration_manager` has hard `#include` dependencies on `active_perception/` and
`scene_graph/` C++ headers. These packages are not needed at runtime for EGO-Planner
simulation, but the codebase cannot compile without them present at build time.

### Decision

Keep `scene_graph/` and `active_perception/` directories in the repository so Docker
build succeeds. The Dockerfile follows this pattern:

```
COPY → build → rm -rf (after build)
```

### Consequences

**Positive**: No C++ patching needed. Build remains `git clone + docker build`.

**Negative**: ~150MB of source+headers that serve no runtime purpose.

**Future work**: Extract only the required headers from `active_perception` and
`scene_graph` into a minimal `_vendor/` stub, then delete the full packages.
