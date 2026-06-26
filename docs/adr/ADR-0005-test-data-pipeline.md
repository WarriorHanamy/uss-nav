# ADR-0005: Test Data Pipeline Architecture

**Date**: 2026-06-26
**Status**: Accepted

## Context

The EGO Planner test system generates rich telemetry during headless
simulation: odometry, IMU, planning results, position commands, LiDAR
point clouds, depth images, and obstacle occupancy grids.  The current
MQTT bridge only collects 5 topics, leaving most data uncaptured for
post-hoc analysis.  Additionally, the smoke test assertion strategy and
frontend technology choices were ad-hoc.  This ADR codifies the test
data pipeline architecture decisions.

## Decisions

### 1. Assertion Strategy — Layered

Smoke test assertions are split into three tiers:

| Tier | Assertion                      | Behavior           | Reasoning                                              |
| ---- | ------------------------------ | ------------------ | ------------------------------------------------------ |
| HARD | `data_flow.*` (odom, plan_result) | fail on empty    | Pipeline integrity — missing data means a broken path  |
| HARD | `odom.bounds`, `odom.max_vel`  | fail on violation | Safety — bad position/velocity data indicates a crash  |
| HARD | `plan.success_rate`            | fail below 50%    | Planner health — ensure the planner produces valid traj |
| SOFT | `state_trigger`, `exec_finish` | warn on empty     | Timing dependent — may not fire within test duration   |

This catches broken pipelines and planner failures without producing
false negatives from timing edge cases.

### 2. Charting Library — Recharts

| Criteria        | Chart.js          | Recharts (chosen)    | Hand-written SVG |
| --------------- | ----------------- | -------------------- | ---------------- |
| React 19 compat | wrapper needed    | native JSX           | N/A              |
| Bundle          | +250KB            | +180KB               | 0KB              |
| Time-series     | built-in          | manual domain calc   | full manual      |
| Maintenance     | medium            | low                  | high             |

Recharts is chosen because this project already uses React 19 + Three.js,
and Recharts produces native JSX components that compose cleanly with
the existing state management pattern (hooks + useMemo).

### 3. Depth Image Encoding — JPEG in JSONL, PNG on disk

| Format        | JSONL size   | Precision            | Decode cost |
| ------------- | ------------ | -------------------- | ----------- |
| JPEG+base64   | ~15KB/frame  | 8-bit, lossy         | low         |
| PNG+base64    | ~150KB/frame | 16-bit, lossless     | medium      |
| Raw float32   | ~1.2MB/frame | full (32-bit float)  | none        |

The pcl_render_node depth image is 32FC1 (single-channel float).
For post-processing visualization the 8-bit approximation is sufficient;
lossless precision is not needed in the frontend display.  JPEG is
embedded as base64 in the JSONL stream.  A disk-side PNG file is
optionally written alongside the JSONL directory for offline analysis.

### 4. ADR Storage — Separate File Per ADR

Stored in `docs/adr/ADR-XXXX-title.md`.  The root `ADR.md` is kept as a
table-of-contents index for quick scanning.  This follows the standard
ADR pattern and keeps each decision independently commit-able and
reference-able.

### 5. Point Cloud Downsampling

`sensor_cloud` at 10 Hz @ ~30000 points/frame is too large for MQTT.
Downsampling strategy:

- Take 1 point every N (configurable, default N=10) → ~3000 points/frame
- Encode as flat float array `[x1, y1, z1, x2, y2, z2, ...]`
- Publish at 1 Hz (skip 9 out of 10 frames)

## Consequences

Positive:
- All 5 data dimensions captured for post-processing
- Layered assertions catch real failures without false alerts
- Recharts keeps the React component model consistent
- Point clouds and depth are persistent and replayable

Negative:
- JPEG discard depth precision; not suitable for metric evaluation
- Point cloud downsampling loses fine structure
- Recharts lacks built-in zoom/pan; needs manual axis wheel logic
