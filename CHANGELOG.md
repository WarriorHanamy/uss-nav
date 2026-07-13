# Changelog

## 2026-07-13 - Merge `company/main`

### Intent

Integrate the company branch's runtime navigation updates into local `main`.
The merged work focuses on making object-oriented navigation more robust in
partially changing maps, especially when topological waypoints become blocked
after the scene graph has already been built.

### Main Changes

- Added SceneGraph update freeze/unfreeze controls so a loaded map can be reused
  without continuous online topology expansion.
- Added topo-block detection and repair flow for object navigation:
  blocked polyhedra are marked, debounced, revalidated after TTL, and optionally
  repaired by projecting to inflate-free positions or inserting replacement nodes.
- Added RC-triggered replan support and protections for receiving replan signals
  when object-id navigation is not active.
- Added VLA swarm map support, observation messages, prompt routing, and a local
  LLM interface path for swarm-oriented semantic context.
- Added support point cloud publishing in simulation and target-estimation
  utilities for bounding-box/lidar and MOGE-based target localization.
- Updated real/sim launch files and planner parameters to expose topo-block,
  replan, support-cloud, and endpoint-rotation behavior.

### Merge Notes

- Two SceneGraph header conflicts were resolved manually:
  - `data_structure.h`: preserved Doxygen annotations and added company runtime
    fields for collision vertex indexing and navigation-blocked polyhedra.
  - `scene_graph.h`: preserved API documentation and kept company functional
    additions for frozen updates, topo-block repair, prompt polling, and VLA
    semantic context.
- Existing local documentation and tooling from `main` were retained.
