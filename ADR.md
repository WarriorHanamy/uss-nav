# Architecture Decision Record

## ADR-001: Merge Company Runtime Navigation Extensions

Date: 2026-07-13

### Status

Accepted

### Context

The local `main` branch contains documentation, web rendering, Docker/test
tooling, and Doxygen annotations. The company branch contains runtime changes
for object navigation, SceneGraph map reuse, VLA swarm prompts, support point
clouds, and target estimation.

Object navigation can fail when a SceneGraph topology node remains semantically
valid but becomes physically blocked in the inflated local map. The original
graph-level `can_reach_` flag is not enough to model this temporary runtime
condition because it mixes topological reachability with dynamic occupancy.

### Decision

Merge the company runtime work and keep the local documentation/tooling layer.
SceneGraph will own a runtime navigation-block layer on top of the persisted
topology:

- `Polyhedron::nav_blocked_`, `blocked_stamp_`, and `blocked_hits_` represent
  temporary occupancy-derived blockage.
- `SceneGraph` keeps access to `MapInterface` so it can validate whether a path
  point is inside the local map and inflate-occupied.
- Blocked polyhedra are debounced before being marked, then revalidated by TTL.
- Repair is handled by projecting blocked points toward inflate-free positions,
  optionally using visibility-sphere midpoint search or replacement-node
  insertion.
- SceneGraph update freezing is exposed separately from map loading so a loaded
  map can be used as a stable navigation substrate.
- VLA swarm prompt generation accepts semantic context explicitly, keeping LLM
  prompt routing decoupled from lower-level map mutation.

### Consequences

- Saved SceneGraph topology stays compatible with older maps because
  `nav_blocked_` state is runtime-only and defaults to unblocked on load.
- Object navigation can recover from newly blocked topology without discarding
  the entire SceneGraph.
- The architecture now has a clear split between persistent topology, dynamic
  occupancy repair state, and semantic/VLA prompt state.
- Future changes should avoid serializing temporary topo-block state unless a
  separate map-version migration is introduced.
- Planner and SceneGraph changes must be validated together because topo-block
  behavior spans `SceneGraph`, exploration FSM replan logic, launch parameters,
  and map occupancy queries.
