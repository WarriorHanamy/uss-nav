# ADR-0006: Post-Processing Visualization Tab

**Date**: 2026-06-26
**Status**: Accepted

## Context

The frontend currently shows a single live 3D trajectory view. When a
test run finishes, operators need to inspect its data in depth:
trajectory quality, obstacle occupancy, body-centric LiDAR frames, and
depth images.  Adding a separate Post-Processing tab provides this
without cluttering the live view.

## Decision

### Tab Structure

A simple `activeTab` state variable (`"live" | "post"`) in `App.tsx`
switches between two 3-panel layouts:

```
Header: [Live] [Post-Processing]    ← tab buttons in <header>
```

**Live tab** — unchanged (Scene3D + StatsGrid + LiveFeed).

**Post-Processing tab:**

```
┌──────────────────────────────────────────────────┐
│ Run Selector ▼  [Load]    [samples: 1283 odom] │
├──────────────────────────┬───────────────────────┤
│                          │                       │
│  3D View                 │  Tracking Charts      │
│  (trajectory + point     │  ┌─────────────────┐  │
│   cloud obstacles)       │  │ pos x/y/z       │  │
│                          │  │ vel x/y/z       │  │
│                          │  │ acc x/y/z       │  │
│                          │  │ ang_vel (yaw)   │  │
│                          │  └─────────────────┘  │
│                          │                       │
│  ┌──── Time ────┐        │                       │
│  │ ◄───────►   │        │                       │
│  └──────────────┘        │                       │
│                          │                       │
│  Body Pointcloud         │  Body Depth           │
│  (sensor_cloud at        │  (depth_img at        │
│   selected time)         │   selected time)      │
├──────────────────────────┴───────────────────────┤
│ Common time slider: ◄───────────────►            │
└──────────────────────────────────────────────────┘
```

### Data Loading

- `GET /api/test/<id>` returns all subtopics as JSON
- Each subtopic is lazily loaded when its panel is visible
- Large data (point cloud, depth) is loaded on-demand when the
  time slider reaches a specific frame

### Component Tree

```
App.tsx
├── header + tabs
├── [activeTab === "live"]
│   ├── Scene3D
│   ├── StatsGrid
│   └── LiveFeed
└── [activeTab === "post"]
    └── PostProcess
        ├── RunSelector
        ├── P3DView         (trajectory + obstacles)
        ├── TrackingCharts  (Recharts time-series)
        ├── BodyCloud       (Three.js Points)
        └── BodyDepth       (canvas 2D)
```

## Consequences

Positive:
- Separation of concerns — live and post-processing are independent
- Time-aligned visualization — single slider drives 3D view, cloud, depth
- Clean data flow — API already provides all needed endpoints

Negative:
- Single-threaded JSON loading — large JSONL files may block UI
  (mitigated by `fetch` streaming and incremental rendering)
- No browser-based comparison — operators cannot overlay two runs
  (future feature)

## Notes

Recharts was chosen over Chart.js for the tracking charts
(see ADR-0005 §2).  Time-slider synchronization across panels
is implemented with a shared `useState<number>` frame index.
