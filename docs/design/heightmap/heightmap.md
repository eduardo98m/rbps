@page feature_heightmap Heightmap Collider Rewrite

# Heightmap Collider Rewrite

## Current State
[include/rbc/shapes/Heightmap.hpp](../../../include/rbc/shapes/Heightmap.hpp)
defines `HeightmapData` (row-major grid + scale + origin) and a
non-owning `Heightmap` shape that wraps a pointer to it. The shape is
flagged `is_gjk_convex == false`; analytic dispatch lives in
[include/rbc/analytic/HeightmapCollision.hpp](../../../include/rbc/analytic/HeightmapCollision.hpp)
and uses the per-triangle helpers in
[analytic/TriangleUtils.hpp](../../../include/rbc/analytic/TriangleUtils.hpp)
(`sphere_vs_triangle`, `capsule_vs_triangle`).

Working pairs (have tests under [tests/rbc/analytic/](../../../tests/rbc/analytic/)):
- Sphere–Heightmap (`test_heightmap_sphere`)
- Capsule–Heightmap (`test_heightmap_capsule`)

Missing or broken: Box–Heightmap, Mesh–Heightmap, Heightmap–Plane (stub
returns `false`), and the Cone / Ellipsoid pairs route through the
generic GJK path which is not appropriate for an unbounded shape.

## Motivation
Heightmaps are the standard ground representation for outdoor
simulations (vehicles, robots, ragdolls on terrain). Without box and
mesh contact the user can only roll spheres / capsules — the
visr_demo cannot construct any realistic terrain scene.

## Proposed Approach
1. Add `box_vs_triangle` to [TriangleUtils.hpp](../../../include/rbc/analytic/TriangleUtils.hpp)
   using SAT (3 box face normals + 3 box edges × triangle edges +
   triangle face normal = 7 axes).
2. Add a `CollisionAlgorithm<Box, Heightmap>` specialisation in
   `HeightmapCollision.hpp` that reuses the existing
   `detail::heightmap_test` infrastructure (cell-range projection +
   per-triangle dispatch) with the new helper. Add the symmetric shim.
3. Add `CollisionAlgorithm<Mesh, Heightmap>`: iterate every face of the
   mesh and run `triangle_vs_triangle` against the cells covered by the
   mesh's world AABB.
4. Mark Heightmap–Plane / Heightmap–Heightmap explicitly (return `false`,
   rationale: two infinite shapes).
5. New tests: `test_heightmap_box`, `test_heightmap_mesh`. Reuse the
   fixture pattern from `test_heightmap_sphere`.
6. Remove the `@warning` block in
   [Heightmap.hpp](../../../include/rbc/shapes/Heightmap.hpp) and
   [HeightmapCollision.hpp](../../../include/rbc/analytic/HeightmapCollision.hpp)
   once tests pass.

## Pair coverage

This doc owns finishing the **Box–HM** and **Mesh–HM** pairs (steps 1–3
above). The remaining heightmap pair gaps are tracked in
[collider-completeness](../collider-completeness/collider-completeness.md):

| Pair | Owned by | Status |
|---|---|---|
| Sphere–Heightmap | this doc | working (`test_heightmap_sphere`) |
| Capsule–Heightmap | this doc | working (`test_heightmap_capsule`) |
| Box–Heightmap | this doc | proposed (steps 1–2) |
| Mesh–Heightmap | this doc | proposed (step 3) |
| Heightmap–Plane | this doc | explicit `false` (step 4) |
| Heightmap–Heightmap | this doc | explicit `false` (step 4) |
| Cone–Heightmap, Ellipsoid–Heightmap | [collider-completeness](../collider-completeness/collider-completeness.md) | needs convex-vs-triangle helper |
| ConvexHull–Heightmap | [collider-completeness](../collider-completeness/collider-completeness.md) | depends on [convex-hull](../convex-hull/convex-hull.md) shipping |

## Risks / Open Questions
- For very large heightmaps (≥ 1024×1024) the per-frame cell scan may
  dominate. Consider a coarse acceleration grid — but only after profiling.
- Triangle winding consistency: the current `tri0` / `tri1` split
  produces consistent normals only if `scale.y > 0`. Document the
  invariant.
- `MeshData` exposes precomputed face normals — reuse them rather than
  recomputing per query.
