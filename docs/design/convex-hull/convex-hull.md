@page feature_convex_hull Convex Hull Collider

# Convex Hull Collider

## Current State
The `Shape` variant in
[include/rbc/shapes/ShapeTypes.hpp](../../../include/rbc/shapes/ShapeTypes.hpp)
holds eight alternatives — `Sphere`, `Box`, `Ellipsoid`, `Capsule`, `Cone`,
`Plane`, `Heightmap`, `Mesh`. There is no `ConvexHull`. A user with an
arbitrary convex polytope (e.g. a procedurally-generated wheel hub) has
two options today, both bad:

1. Encode it as `Mesh`. `Mesh` is flagged `is_gjk_convex == false`
   ([Mesh.hpp:178](../../../include/rbc/shapes/Mesh.hpp)) so the
   dispatcher routes every pair through the per-triangle analytic path
   in [MeshCollision.hpp](../../../include/rbc/analytic/MeshCollision.hpp).
   GJK / EPA never run, even though the shape is convex.
2. Approximate it with primitives. Loses fidelity and inflates the
   collider count.

The underlying machinery for a real convex-hull collider is already in
the repo: [include/rbc/gjk/GJK.hpp](../../../include/rbc/gjk/GJK.hpp),
[include/rbc/gjk/EPA.hpp](../../../include/rbc/gjk/EPA.hpp), and
[include/rbc/gjk/ContactManifoldGenerator.hpp](../../../include/rbc/gjk/ContactManifoldGenerator.hpp)
work for any pair where both shapes report `is_gjk_convex == true` and
expose a `support()`. `Mesh` already implements a vertex-scan `support()`
([Mesh.hpp:120](../../../include/rbc/shapes/Mesh.hpp)) — it just isn't
allowed to use it.

## Motivation
- Scene loaders (URDF mesh import in
  [scene-format](../scene-format/scene-format.md)) routinely receive
  convex hulls; users currently have no type-safe way to assert
  convexity. Without this, every imported convex mesh pays the
  `Mesh` triangle-soup penalty.
- Procedurally generated shapes (game-style level pieces, randomised
  benchmarks) need a convex polytope primitive that isn't a sphere /
  box / capsule.
- Closes a gap in the `is_gjk_convex` invariant: every shape that *is*
  convex should advertise it, so the dispatcher's GJK fast path covers
  every convex–convex pair without exception.

## Proposed Approach
1. Add `ConvexHull` to the variant alternative list in
   [ShapeTypes.hpp](../../../include/rbc/shapes/ShapeTypes.hpp:67) —
   one line plus a constructor overload, mirroring the `Mesh` lines.
2. New header `include/rbc/shapes/ConvexHull.hpp`. Backing data is a
   non-owning view over a vertex array (and optional face index array
   for `face_corners`):
   ```cpp
   struct ConvexHullData {
       const m3d::vec3* vertices;
       std::size_t      vertex_count;
       const uint32_t*  face_indices; // optional, for face features
       const uint32_t*  face_offsets;
       std::size_t      face_count;
   };
   struct ConvexHull { const ConvexHullData* data; };
   ```
   Mirrors the `Heightmap` / `Mesh` non-owning pattern so the storage
   side is the caller's problem (consistent with the rest of `rbc`).
3. Implement the per-shape contract:
   - `support(hull, dir)` — vertex scan (identical to
     [Mesh.hpp:120](../../../include/rbc/shapes/Mesh.hpp), but unconditional).
   - `compute_aabb(hull, tf)` — min/max over transformed vertices.
   - `compute_inertia_tensor(hull)` — tetrahedral decomposition from
     the centroid; standard formula, unit density.
   - `representative_radius(hull)` — max `|v|` over vertices.
   - `face_corners(hull, tf, dir, out[4])` — pick the face whose normal
     is closest to `dir`, return up to 4 of its corners. If face data
     wasn't provided, fall back to the support point as a single
     vertex (this matches the contact-feature path that
     [contact-generation](../contact-generation/contact-generation.md)
     introduces).
   - `is_gjk_convex(const ConvexHull*) -> true`.
4. Dispatcher table grows from 8×8 to 9×9 in
   [Dispatcher.hpp](../../../include/rbc/Dispatcher.hpp). Most cells
   need no new analytic — every convex pair routes through GJK + EPA
   via the existing template. Pairs against unbounded shapes (`Plane`,
   `Heightmap`) use the existing `convex_vs_plane` helper and (after
   [contact-generation](../contact-generation/contact-generation.md))
   the generic feature path against terrain triangles.
5. Tests under [tests/rbc/analytic/](../../../tests/rbc/analytic/):
   `test_convex_hull_sphere`, `test_convex_hull_box`, `test_convex_hull_self`.
   Reuse fixtures from `test_dispatcher` and `test_gjk`.

## Risks / Open Questions
- **Convexity validation.** Reject (or `assert`) on construction if
  the input is non-convex? A debug-only check that walks face
  half-spaces is O(F·V) — acceptable. In release, trust the caller.
- **Hull builder.** Out of scope here — users either pre-hull
  externally (Bullet, libccd, qhull) or pass primitives. Document
  this in `ConvexHull.hpp`.
- **Storage ownership.** Non-owning matches `Mesh` / `Heightmap` but
  forces the caller to keep the vertex buffer alive. Consider an
  owning `ConvexHullCollection` SoA later if URDF import wants it.
- **Face data optional vs. required.** Without faces, manifold
  generation falls back to single-vertex features which produce
  worse contacts. URDF importers that already triangulate the hull
  should fill it; procedural callers can omit.
