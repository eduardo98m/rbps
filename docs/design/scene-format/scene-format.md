@page feature_scene_format Scene File Format

# Scene File Format

## Current State
Scenes are constructed in C++ — see
[src/visr_demo/main.cpp](../../../src/visr_demo/main.cpp) which creates
bodies, colliders, and joints by hand for the demo. No serialisation
exists. `World`, `BodyCollection`, `ColliderCollection`, etc. are SoA
containers with no I/O hooks.

## Motivation
Hand-coded scenes block the user from:
- Reusing scenes across the test app and headless benchmarks.
- Sharing test cases (drop a `.urdf` next to the test).
- Importing existing models from URDF / MJCF / USD libraries common
  in robotics and graphics.

## Proposed Approach
**Recommendation: URDF first, MJCF as a follow-up. Skip USD for v1.**

URDF is the simplest of the three (plain XML, well-supported in
robotics), maps cleanly to RBPS primitives (link = body, joint = joint,
geometry = collider), and unlocks ROS 2 integration as a side effect
([ros2-integration.md](../ros2-integration/ros2-integration.md)).

1. New module: `include/rbpx/` (RBPS eXchange) with a small public API:
   ```cpp
   namespace rbpx {
       std::optional<std::string> load_urdf(rbps::World&, const std::string& path);
       void save_urdf(const rbps::World&, const std::string& path);
   }
   ```
2. URDF parser: pull in `tinyxml2` via `FetchContent` (single-header,
   permissive licence). Parse `<link>` / `<joint>` / `<visual>` /
   `<collision>` and translate to `BodyParams` / `ColliderParams` /
   `JointParams`. Map URDF inertia tensors to `m3d::smat3`.
3. Shape mapping table: URDF supports `box`, `sphere`, `cylinder`,
   `mesh`. Map directly to `rbc::Box`, `rbc::Sphere`, a
   capsule approximation for `cylinder`, and `rbc::Mesh` (require an
   external loader for the actual `.obj` / `.stl` data).
4. Add a `--scene <path>` CLI flag to `visr_demo` so users can drop
   `.urdf` files in directly.
5. MJCF as a follow-up: similar surface area, adds joint actuators and
   contact tuning that URDF lacks. Defer until URDF is shipping.

## Risks / Open Questions
- URDF's "convention is metres / radians / kg / right-handed" — make
  sure the loader rejects scenes with unit attributes set otherwise.
- Mesh loading is out of scope for `rbpx`; either require pre-loaded
  `MeshData*` from the caller or add a thin `tiny_obj_loader` dependency.
- USD is the right format for visually rich scenes but its SDK is
  heavyweight (Pixar's `OpenUSD` ~ 200 MB). Revisit if the engine grows
  visual fidelity.
