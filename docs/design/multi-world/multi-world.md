@page feature_multi_world Multi-World Support

# Multi-World Support

## Current State
The engine assumes one `World` per process. Tests and demos
construct multiple `World` instances sequentially but never
concurrently; there is no parallel-step API, no shared resource
audit, and no enforcement that subsystems are free of mutable global
state. The visualizer thread spins one physics thread, owning one
`World` (see [include/visr/VisrApp.hpp](../../../include/visr/VisrApp.hpp)).

## Motivation
- **Paper 3 (heterogeneous batching).** Direction C needs N
  independent worlds running on N CPU threads, each holding a
  different morphology / scene. Each world steps independently; no
  cross-world synchronization. This is precisely the regime
  GPU-batched simulators struggle with.
- **Paper 2 MPC parallelism.** [sample-mpc](../sample-mpc/sample-mpc.md)
  rollouts can fan out across worlds for K-sample evaluation. With
  [world-snapshot-restore](../world-snapshot-restore/world-snapshot-restore.md)
  feeding state, multi-world is the throughput substrate.
- **Vectorized RL envs.** `gymnasium.vector.AsyncVectorEnv` and its
  cousins expect N parallel environments. Without multi-world,
  Python-side multiprocessing pays IPC + serialization overhead per
  step; in-process multi-world is far cheaper.
- **Trivially deterministic.** Worlds share no state; reproducibility
  per-world is unaffected by sibling worlds. Bit-exact replay
  composes.

## Proposed Approach
**Audit `World` for hidden global state, then add a thin `WorldPool`
helper. The architectural lift is small because the SoA layout
already enforces world-local data.**

1. **Audit phase.** Grep [include/](../../../include/) and [src/](../../../src/)
   for `static` non-const variables, file-scope globals, and
   thread-local storage. Expected: math3d is pure functional;
   broad-phase state is per-`World`; constraint solvers take
   collections by reference. Document any findings in this design
   doc's `Outcome` section after the audit.
2. **`WorldPool` API** in `include/rbps/api/WorldPool.hpp`:
   ```cpp
   struct WorldPool {
       std::vector<World> worlds;
       explicit WorldPool(size_t n);
       void parallel_step(scalar dt);                       // OpenMP-parallel
       template <typename Fn> void parallel_each(Fn&& fn);  // arbitrary per-world op
   };
   ```
   `parallel_step` uses the OpenMP from
   [openmp-parallelization](../openmp-parallelization/openmp-parallelization.md)
   *across* worlds (one world per thread), independent of intra-world
   parallelism.
3. **Two-level parallelism.** Outer loop = worlds; inner loop =
   constraint coloring within a world. Configure via OpenMP nested
   parallelism with `omp_set_max_active_levels(2)`. Default: outer
   parallel, inner serial — most heterogeneous-RL workloads benefit
   more from world-level parallelism.
4. **Python API.** Expose `WorldPool` in
   [python-bindings](../python-bindings/python-bindings.md); add a
   `RbpsVectorEnv` Gymnasium wrapper subclassing
   `gymnasium.vector.VectorEnv`. Numpy returns are stacked
   `(n_worlds, ...)` arrays.
5. **Snapshot fan-out.** `WorldPool::broadcast_snapshot(snap)` —
   copy one snapshot into every world. Used by the MPPI/CEM runner
   to seed K rollouts from a common start state.
6. **Tests:** `tests/rbps/test_world_pool.cpp` — N=8 worlds,
   verify (a) correct results match N=1 single-world results,
   (b) no data races under TSan, (c) parallel speedup at 4 threads.

## Risks / Open Questions
- **Hidden globals.** The audit may surface ones we don't expect
  (allocator caches, raylib globals reachable through the visualizer,
  Doxygen-only static helpers). Guard via TSan in CI from day one.
- **Cache pressure.** N hot worlds all touching distinct memory may
  thrash L3 / DRAM bandwidth. Profile on the reference machine;
  recommend N ≤ physical cores.
- **Allocator contention.** `std::vector` resizes inside the inner
  loop hit the global allocator. Either preallocate per world (reset
  via `clear()` not `=`) or hand out a per-thread arena allocator.
- **Visualizer interaction.** The render thread iterates a single
  `World`. If `WorldPool` is in use the visualizer must either pick
  one to render or grow a "render world #i" selector; defer until the
  visualizer refactor lands.
