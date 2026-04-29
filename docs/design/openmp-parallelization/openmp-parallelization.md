@page feature_openmp OpenMP Parallelization

# OpenMP Parallelization

## Current State
The solver is fully serial. Hot loops live in:
- `update_position_and_orientation` and `update_velocities` in
  [src/rbps/Body.cpp](../../../src/rbps/Body.cpp) — embarrassingly parallel
  per-body integration.
- `solve_contacts_position_level` / `solve_contacts_velocity_level` in
  [src/rbps/constraints/Contact.cpp](../../../src/rbps/constraints/Contact.cpp)
  — NOT trivially parallel (two contacts on the same body race each
  other), but the dependency graph is already coloured by
  `get_collision_groups` in
  [include/rbps/CollisionPipeline.hpp](../../../include/rbps/CollisionPipeline.hpp).
- `solve_constraints` in [src/rbps/constraints/Constraint.cpp](../../../src/rbps/constraints/Constraint.cpp)
  — same race; no graph colouring yet, would need one.

CMake currently does not link OpenMP.

## Motivation
For dense scenes (≥1000 dynamic bodies, common in granular / cloth-like
simulations) the integration loops alone take ~30 % of frame time.
Linear speed-up on 8 cores is a 4× wall-clock win at zero algorithmic
cost — the SoA layout and existing graph colouring make this almost
free.

## Proposed Approach
1. Top-level CMake: `find_package(OpenMP REQUIRED)` (gated behind
   `option(RBPS_USE_OPENMP "Parallelize hot loops" ON)`). Link
   `OpenMP::OpenMP_CXX` into `rbps_lib`.
2. Body integration: add `#pragma omp parallel for` to the per-body
   loops in `update_position_and_orientation` and `update_velocities`.
   No data dependencies — each body writes only its own row.
3. Contacts: `solve_contacts_position_level` / `_velocity_level` iterate
   the contact list serially today. Replace with:
   ```cpp
   for (const auto& group : groups) {
       #pragma omp parallel for
       for (size_t k = 0; k < group.size(); ++k)
           apply_constraint_position_level(cl, group[k], bc, inv_h);
   }
   ```
   `groups` comes from `get_collision_groups` (existing) — each colour
   has no shared bodies, so writes are conflict-free.
4. Joint constraints: extend the graph-colouring to constraint rows
   (`ConstraintCollection`) and parallelise `solve_constraints` the
   same way.
5. Add a benchmark target `bench_solver` (Google Benchmark or hand-
   rolled) that drops 1024 bodies and times one frame; CI runs it on
   1 / 4 / 8 threads to catch regressions.

## Risks / Open Questions
- OpenMP scheduling: prefer `schedule(static)` for the body loops
  (uniform cost) and `schedule(dynamic, 32)` for the contact loops
  (cost varies per pair).
- False sharing on `BodyCollection` arrays: every field is a separate
  `std::vector<T>` so different threads read disjoint cache lines —
  except for the small ones (`type`, generation counter). Profile on
  the target platform before tuning.
- `CmdApplyImpulse` and other commands run on the physics thread;
  they implicitly assume serial access. If commands ever move into the
  parallel region, gate them behind a "not in parallel" assert.
- Determinism: graph-coloured parallel solving still produces the same
  result modulo floating-point reduction order. Document that bit-exact
  reproducibility requires `RBPS_USE_OPENMP=OFF`.
