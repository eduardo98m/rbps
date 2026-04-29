@page feature_rendering Rendering Pipeline Upgrade

# Rendering Pipeline Upgrade

## Current State
[include/visr/ui/RaylibDraw.hpp](../../../include/visr/ui/RaylibDraw.hpp)
draws every collider with raylib's immediate-mode wireframes
(`DrawSphereWires`, `DrawCubeWires`, `DrawCapsuleWires`, …) one body at
a time. Colour conventions are documented in the file header; lighting
is the default raylib ambient. There is no shadow pass, no texturing,
no instancing. Heightmaps are drawn as a wireframe grid; meshes are not
rendered (their surface area is just summarised numerically).

## Motivation
- A wireframe-only view is hard to read once scenes have more than
  ~50 bodies.
- No shadows means depth is ambiguous on a uniform background, which
  makes contact debugging harder than it should be.
- Lacking instancing puts a hard ceiling on the number of bodies that
  the demo can show at 60 Hz (~ 200 boxes today on integrated graphics).

## Proposed Approach
**Stay on raylib for v1.** raylib 5.5 supports custom shaders via
`Shader` + `BeginShaderMode`, mesh instancing via
`DrawMeshInstanced`, and shadow maps via the standard
`SHADOWMAP_RESOLUTION` examples. Switching to a heavyweight engine
(bgfx / Vulkan) is a separate decision; not required for these wins.

1. Convert each shape kind in `RaylibDraw.hpp` to use raylib's `Mesh`
   (`GenMeshSphere`, `GenMeshCube`, …) cached once per shape. Build
   per-frame instance arrays grouped by shape; one `DrawMeshInstanced`
   call per group.
2. Add a single directional light + shadow map pass. Reuse raylib's
   `examples/shaders/shaders_shadowmap.c` shader as the starting point.
   Toggle in `DrawFlags` so headless / debug builds can disable it.
3. PBR-lite material: hook `restitution` and `friction` from
   `ColliderSnap` into base colour and roughness so the user gets a
   visual cue for material differences without an extra panel.
4. Mesh and heightmap rendering: build a real raylib `Mesh` from
   `MeshData` / `HeightmapData` once at first draw, cache by data
   pointer (the `Mesh*` already serves as a stable key).
5. Post-processing: a simple FXAA pass via `LoadShader` — cheap, large
   visual win on ImGui text overlays.

## Risks / Open Questions
- raylib's instancing path is single-precision only; `m3d::scalar` is
  `double` so we already cast at the draw boundary (see `to_rl`). No
  new precision concerns.
- Shadow maps add ~1.5 ms / frame on integrated GPUs. Make the toggle
  default-off until the user enables it.
- Mesh caching by `MeshData*` requires a `std::unordered_map<const
  MeshData*, Mesh>` owned by `RenderSystem` — invalidated when the
  data is destroyed. Document the lifetime contract.
- Skip global illumination and SSAO; those belong to a future
  high-fidelity rendering doc, not this one.
