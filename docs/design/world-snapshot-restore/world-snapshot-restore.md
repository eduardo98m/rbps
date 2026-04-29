@page feature_world_snapshot_restore World Snapshot / Restore

# World Snapshot / Restore

## Current State
`World` in [include/rbps/API/World.hpp](../../../include/rbps/API/World.hpp)
owns `BodyCollection`, `ColliderCollection`, `ConstraintCollection`,
`JointCollection`, and `ContactList`, all SoA-formatted via
[include/storage/Soa.hpp](../../../include/storage/Soa.hpp)'s
`DEFINE_SOA` / `DEFINE_DYN_SOA` macros. Broad-phase state lives in
`rbc::BroadPhaseState`. There is no checkpoint, clone, save, or
restore API — `World` cannot be copied today (the SoA collections are
move-only by convention; the broad-phase endpoint list assumes stable
addresses).

## Motivation
- **Centerpiece paper (in-agent MPC).** The agent must fork off K
  short-horizon rollouts at every action selection, run them on the
  *real* simulator, score them, then restore. Without sub-millisecond
  snapshot/restore, the entire premise of paper 2 collapses.
- **Sample-based MPC** (CEM / MPPI, see [sample-mpc](../sample-mpc/sample-mpc.md))
  evaluates action sequences via repeated forward simulation from a
  fixed start state.
- **Determinism reproduction.** Bit-exact replay of a trajectory
  requires bit-exact state restore.
- **Tests / debugging.** Snapshotting before a flaky test step lets
  us re-run with logging turned up.

## Proposed Approach
**Two-tier API: a structural snapshot for any case, an opaque-blob
fast path for the hot-loop case.**

1. New header `include/rbps/snapshot/WorldSnapshot.hpp`:
   ```cpp
   struct WorldSnapshot {
       BodyCollection      bodies;
       ColliderCollection  colliders;
       ConstraintCollection constraints;
       JointCollection     joints;
       ContactList         contacts;
       rbc::BroadPhaseState broad_phase;
       float               time;
   };

   WorldSnapshot snapshot(const World&);
   void          restore(World&, const WorldSnapshot&);
   ```
   First implementation: deep-copy each SoA collection by copying
   every `std::vector<T>` field. The macros in `Soa.hpp` already
   know the field list — a one-line `clone_soa` helper hooks every
   collection.
2. **Shared shape data.** `MeshData` and `HeightmapData` are
   pointer-referenced by colliders and assumed immutable; snapshots
   keep raw pointers — no deep copy of mesh / heightmap arrays.
3. **Fast-path:** `WorldBlob` — preallocate a contiguous arena per
   snapshot slot, `memcpy` each `vector::data()` into it, restore via
   reverse `memcpy`. Skips allocator round-trips. Target: <50 µs
   round-trip on a 50-body, 100-contact scene.
4. **Snapshot pool.** Caller-provided `SnapshotPool` of N
   pre-allocated blobs prevents allocator churn under MPC's K-rollout
   loop. Pool size chosen by user (typical K ≤ 256).
5. **Python bindings.** Expose `world.snapshot()` and
   `world.restore(snap)` in [python-bindings](../python-bindings/python-bindings.md);
   `WorldSnapshot` becomes opaque from Python.
6. **Tests:** `tests/rbps/test_world_snapshot.cpp` validates
   bit-exact restore across (a) immediate snapshot/restore,
   (b) snapshot → step 100 frames → restore, (c) snapshot → restore
   on a different `World` instance.

## Risks / Open Questions
- **Latency budget verification.** The whole arc rests on this. First
  validation work in any implementation is to confirm <50 µs on the
  reference scene; if it's > 200 µs at small scale the design needs
  fundamental rework (e.g., persistent diff snapshots).
- **Broad-phase state.** SAP endpoints reference body indices; if the
  underlying body slots change generation, restore must replay
  `_insert` rather than `memcpy` the endpoint array. Audit
  [BroadPhase.cpp](../../../src/rbc/BroadPhase.cpp) for invariants.
- **Determinism on restore.** Floating-point reductions during
  collision detection should produce identical bytes given identical
  inputs — verify with a bit-exact diff harness before claiming
  determinism in any paper.
- **Allocator thrash.** Vector-of-snapshots in a tight loop hammers
  the allocator. The fast-path arena/pool above is required, not
  optional, for MPC use.
