# ADR-0007: Realsense D455 Body-Centric Image Simulation

**Date**: 2026-06-26
**Status**: Accepted

## Context

The ego-planner simulation needs a body-centric depth image stream
(simulating a forward-mounted Realsense D455) for post-processing
visualization of what the drone "sees".  The existing `pcl_render_node`
produces a depth image (`/drone_0_pcl_render_node/depth_img`) using a
LiDAR-style polar scan — not a pinhole camera model.

## Decision

### Approach: Polar Scan as D455 Proxy

Rather than building a new pinhole depth renderer, the existing
`pcl_render_node` polar depth image is re-parameterized to approximate
the D455 field of view.  This is a pragmatic trade-off:

| Specification   | Real D455         | Polar proxy              |
| --------------- | ----------------- | ------------------------ |
| HFOV            | 86°               | `yaw_fov: 86`            |
| VFOV            | 57°               | `vertical_fov: 57`       |
| Resolution      | 848 × 480         | `polar_resolution: 0.09` (≈ 955×633) |
| Max range       | 10m               | `sensing_horizon: 10`    |
| Frame rate      | 30 fps            | `sensing_rate: 30`       |
| Projection      | pinhole           | polar (equi-angular)     |

The polar projection differs geometrically from a pinhole, but for
post-processing visualization (human inspection of obstacle shapes,
depth contours) the difference is acceptable.

### Configuration File

New file: `ws_main/src/planner/sim_bringup/params/rec_learn_realsense_d455.yaml`

Values:
```yaml
sensing_rate: 30.0         # fps
sensing_horizon: 10.0      # max depth [m]
is_360lidar: 0             # forward-facing only
yaw_fov: 86.0              # horizontal FOV [deg]
vertical_fov: 57.0         # vertical FOV [deg]
polar_resolution: 0.09     # angular step [deg] (~0.1-m px at 10 m)
livox_linestep: 0.0        # not a livox pattern
curvature_limit: 100.0     # no curvature-based downsampling
use_avia_pattern: 0
use_vlp32_pattern: 0
use_minicf_pattern: 0      # uniform scan, no non-repeating pattern
downsample_res: 0.01       # keep nearly all points
```

### Launch Integration

The `rec_learn_ego_standalone.launch` gains an optional argument:

```xml
<arg name="use_d455" default="false"/>
<!-- Use D455-like config when use_d455:=true -->
```

When `use_d455:=true`, the pcl_render_node loads `rec_learn_realsense_d455.yaml`
instead of the default LiDAR parameters.

## Consequences

Positive:
- Body-centric depth image available on `/drone_0_pcl_render_node/depth_img`
- Lean config file — no new renderer, no new dependencies
- Easy to toggle between LiDAR and D455 modes

Negative:
- Polar projection ≠ pinhole; straight lines in the world will appear
  curved in the depth image (fish-eye effect)
- No RGB stream — the simulator has no color camera model
- `polar_resolution` 0.09° produces ~10580 rays/frame at 30 fps;
  may increase CPU load on the pcl_render_node thread
