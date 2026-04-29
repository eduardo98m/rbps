@page feature_broadphase_bvh Pluggable Broad Phase + BVH

# Pluggable Broad Phase + BVH Backend

## Current State
The broad phase is a single Sweep-and-Prune (SAP) implementation in
[include/rbc/BroadPhase.hpp](../../../include/rbc/BroadPhase.hpp) /
[src/rbc/BroadPhase.cpp](../../../src/rbc/BroadPhase.cpp). State lives in
`BroadPhaseState` (sorted X-axis endpoint list + per-object record);
the API is `broad_phase_init`, `_insert`, `_remove`, `_move`,
`_update`. Output is `std::vector<BroadPhasePair>` (pairs of user IDs).

SAP is hard-coded into the call sites. `update_broad_phase_aabbs` and
`run_collision_pipeline` in
[include/rbps/CollisionPipeline.hpp](../../../include/rbps/CollisionPipeline.hpp)
both take `rbc::BroadPhaseState&` directly.

## Motivation
SAP is great for nearly-uniform 1D layouts (insertion sort is O(n)
on near-sorted arrays) but pathological for "tall stacks of objects"
where every dynamic AABB shares the X projection. A dynamic-AABB BVH
trades the steady-state cache wins for `O(log n)` worst-case behaviour
and is the broad phase used by Bullet, ODE, and modern game engines
for general scenes.

## Proposed Approach
1. Introduce an interface header
   `include/rbc/BroadPhaseBackend.hpp` defining a small C-style API:
   ```cpp
   struct BroadPhaseOps {
       void  (*init)   (void* state, const BroadPhaseConfig& cfg);
       BPHandle (*insert)(void* state, uint32_t user_id, const AABB&);
       void  (*remove) (void* state, BPHandle h);
       void  (*move)   (void* state, BPHandle h, const AABB&);
       void  (*update) (void* state, std::vector<BroadPhasePair>& out);
   };
   ```
   `BroadPhaseState` becomes one of two structs; the active backend is
   chosen at construction.
2. Refactor existing SAP code to fit the interface (rename internal
   functions, leave behaviour unchanged). All current tests should keep
   passing.
3. Add a BVH backend `BVHBroadPhase`:
   - Bottom-up build using surface-area heuristic (existing
     `aabb_surface_area` helper in [AABB.hpp](../../../include/rbc/AABB.hpp)).
   - Refit on `move` (climb the tree, recompute parent AABBs).
   - Periodic rebuild every N frames when the SAH cost grows by > 25 %.
   - Pair generation via tree-vs-tree traversal.
4. Add a `BroadPhaseKind` enum to `World` so users can pick at startup:
   ```cpp
   World w; w.set_broad_phase(BroadPhaseKind::BVH);
   ```
5. New benchmark in `tests/rbc/` (or a dedicated `benches/`) that drops
   N stacked boxes and reports broad-phase update time on both backends.

## Risks / Open Questions
- The function-pointer indirection adds one call per broad-phase API
  use. Acceptable — these are not inner-loop calls.
- BVH refits leak memory on bodies that move every frame; need a
  rebuild trigger that doesn't pathologically pessimise.
- Exposing the backend to the user means the `World::set_broad_phase`
  API has to clear all colliders or rehome them — keep this simple
  by requiring it to be called before any `create_collider`.
- Selection of default: keep SAP as default until BVH benchmarks win
  on representative scenes (≥ 95 % parity).
