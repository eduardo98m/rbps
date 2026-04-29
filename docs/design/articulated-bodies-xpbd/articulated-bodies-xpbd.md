@page feature_articulated_bodies_xpbd Articulated Bodies (Maximal-Coordinate XPBD)

# Articulated Bodies (Maximal-Coordinate XPBD)

## Current State
The engine simulates free-floating rigid bodies via `BodyCollection` in
[include/rbps/Body.hpp](../../../include/rbps/Body.hpp) and supports
pair-wise joint constraints (revolute, prismatic, fixed) in
[include/rbps/constraints/Joint.hpp](../../../include/rbps/constraints/Joint.hpp).
Joint actuators are stubbed via the `FREE / SPEED / POSITION` enum but
have no PD-gain or torque-limit machinery (see
[actuator-models](../actuator-models/actuator-models.md)). There is no
notion of a "robot" or kinematic tree: every body is independent and
joints are inserted by hand. URDF cannot be loaded today.

## Motivation
- Every standard robotics RL benchmark (Hopper, Walker2d, HalfCheetah,
  manipulators, quadrupeds) requires multi-link articulated bodies.
- URDF ([scene-format](../scene-format/scene-format.md)) maps `<link>`
  → body and `<joint>` → constraint; without articulated chains the
  loader cannot exercise its core path.
- The thesis arc's centerpiece paper (in-agent MPC + PPO) needs at
  minimum a 2-D bipedal or simple manipulator scene, neither of which
  is constructible without joint chains.
- Müller's 2022 paper *"Detailed Rigid Body Simulation with Extended
  Position Based Dynamics"* shows XPBD handles articulated chains via
  positional constraints — no algorithmic invention required, just
  infrastructure work.

## Proposed Approach
**Stay in maximal coordinates.** Each link is an independent rigid
body; joints are constraints between them. This is the natural XPBD
route, reuses the existing constraint solver, and preserves
determinism. Reduced-coordinate Featherstone-style integration is a
much larger refactor and orthogonal to the thesis arc.

1. New module `include/rbps/articulation/` with:
   ```cpp
   struct Articulation {
       std::vector<uint32_t> body_ids;       // BFS order from root
       std::vector<uint32_t> joint_ids;      // parallel to body_ids[1:]
       uint32_t              root_body;
       std::optional<int>    floating_base;  // joint id, or none if fixed-base
   };
   ```
   Articulations are containers; the underlying solver still iterates
   bodies and constraints by index.
2. Extend joint types in [Joint.hpp](../../../include/rbps/constraints/Joint.hpp)
   with `Spherical`, `Universal`, `Planar`, and `Continuous` to cover
   the URDF surface. All map to combinations of existing positional /
   angular constraints — no new constraint primitives required.
3. Joint limits: add `JointLimit { scalar lower, upper, compliance; }`
   as inequality constraints (compliance > 0 gives soft limits, → 0
   gives hard stops). Reuse the lambda-clamp pattern already used for
   contact normals in [Contact.cpp](../../../src/rbps/constraints/Contact.cpp).
4. Floating base: a 6-DoF "joint" between world and root link, used
   for free-floating robots (humanoids, quadrupeds).
5. Validation harness: a 7-DoF revolute chain whose forward dynamics
   under gravity is compared against pinocchio or RBDL across 1000
   integration steps. Per-frame error budget: < 1e-3 rad on joint
   angles after 1 s of simulated time at 1 ms substeps.
6. Tests: `tests/rbps/articulation/test_chain_pendulum.cpp`,
   `test_floating_base.cpp`, `test_joint_limits.cpp`. Reuse the
   fixture pattern from `tests/rbps/test_joint.cpp`.

## Risks / Open Questions
- **Drift at low substep counts.** XPBD chains can elongate visibly
  with N≤4 substeps and high mass ratios. Document the recommended
  substep floor for articulations (likely 8–10) and report stability
  ranges in the validation harness.
- **Joint limit jitter.** Inequality constraints with hard compliance
  oscillate on contact. Match Müller's 2022 recommendation: clamp the
  multiplier, don't clamp the position correction directly.
- **Mass-ratio robustness.** Big-link-on-small-link pairs (URDF
  inertia tensors are arbitrary) need a recommended fix-up; the
  literature uses generalized inverse mass weighting which the
  engine already implements — verify it doesn't degenerate.
- **API ergonomics.** Should `Articulation` own the bodies, or just
  reference them by ID? Owning simplifies cleanup; referencing
  preserves the SoA-everywhere principle. Recommend referencing,
  matching the existing `JointCollection` pattern.
