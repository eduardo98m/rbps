@page feature_benchmark_harness Benchmark Harness

# Benchmark Harness

## Current State
The engine has comprehensive correctness tests under
[tests/](../../../tests/) (unit-level, CTest-based, with coverage
badges) but no throughput, latency, or stability benchmarks. The
`visr_demo` executable runs scenes interactively but does not log
performance numbers. There is no comparison against MuJoCo, Bullet,
or any other engine — the README claims no throughput figures.

## Motivation
- **Figure 1 of every paper.** Every research output starting from
  paper 1 needs a "RBPS vs MuJoCo on standardized scenes" plot.
  Building it once amortizes across the whole thesis.
- **Regression detection.** Optimization work (OpenMP wire-up, BVH
  broad phase) needs an objective measure of "did this make things
  faster or slower." Without benchmarks, performance refactors fly
  blind.
- **Engine credibility.** Reviewers and prospective adopters check
  README for numbers. A small public-facing throughput table is
  high-leverage.
- **Validates the paper-2 premise.** The "real simulator inside the
  agent" claim only holds if step latency and snapshot latency hit
  specific budgets — the harness operationalizes those budgets.

## Proposed Approach
**A `benches/` module structurally parallel to `tests/`, plus a
Python comparison runner. C++ side measures RBPS; Python side runs
the same scenes in MuJoCo and PyBullet for head-to-head plots.**

1. New top-level `benches/` directory, with a CMake target
   `rbps_bench` linking [Google Benchmark](https://github.com/google/benchmark)
   via `FetchContent`. Gated behind `option(RBPS_BUILD_BENCHES
   "Build benchmarks" OFF)`.
2. **Standard scenarios** — each as a separate `.cpp` under
   `benches/scenarios/`:
   - `sphere_pile_N` (N = 100, 500, 2000) — granular pile, gravity,
     friction. Stresses contact generation.
   - `box_stack_H` (H = 4, 8, 16) — stability under XPBD.
   - `pendulum_chain_N` (N = 5, 20, 50) — articulated stress test
     once [articulated-bodies-xpbd](../articulated-bodies-xpbd/articulated-bodies-xpbd.md)
     lands.
   - `mixed_scene` — heterogeneous shapes, 200 bodies.
   - `heightmap_vehicle` — capsule on heightmap, rolling.
3. **Metrics per run** (logged to JSON in `benches/results/`):
   - Wall-clock per `World::step` (mean, p50, p95, p99).
   - Total throughput (bodies × steps / second).
   - Snapshot/restore latency (when available).
   - Stability proxy: max penetration depth, total energy drift over
     the run, position drift on a known equilibrium pose.
4. **Comparison runner** (`benches/python/compare.py`):
   - Defines the same scenarios in MuJoCo (via `mujoco`) and PyBullet
     (via `pybullet`), parameterized identically (same gravity,
     friction, substep budget *to the extent the engines support it*).
   - Dumps a unified JSON, plots throughput / latency / drift bars.
   - Cited as Figure 1 in every paper output.
5. **CI integration** (post-OpenMP, since scaling matters):
   - Run `rbps_bench` on a fixed reference machine label; track
     regressions against the previous commit; fail PR if regression
     > 10 % on any scenario.
6. **Documentation.** A `benches/README.md` explains the methodology,
   the reference machine, and how reviewers can re-run results from
   a release tag.

## Risks / Open Questions
- **Apples-to-apples comparison is non-trivial.** MuJoCo's solver
  (Featherstone + soft constraints) and RBPS's (XPBD) have different
  knobs. Match wall-clock budget, not internal substep counts;
  document divergences explicitly.
- **Hardware variability in CI.** Cloud CI runners are noisy. Either
  pin to a self-hosted reference machine for absolute numbers, or
  use CI only for relative regression detection.
- **Stability metric definition.** "Max penetration depth" is easy
  to game with high stiffness; supplement with energy drift rate to
  catch silent instability. Document the precise definitions.
- **Scope creep.** Benchmark suites grow forever. Cap the standard
  set at the five scenarios above; per-paper extensions live in
  paper-specific branches, not in the canonical suite.
