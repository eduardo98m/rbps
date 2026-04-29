@page feature_collider_completeness Collider Completeness

# Collider Completeness

## Current State
After the dispatcher is generated from the `Shape` variant
([ShapeTypes.hpp:67](../../../include/rbc/shapes/ShapeTypes.hpp)) the
collision layer has the following gaps that no other design doc owns:

- **Plane bypasses SAP.** Its AABB is infinite
  ([Plane.hpp:72](../../../include/rbc/shapes/Plane.hpp)) so the broad
  phase cannot index it; planes are tested directly inside
  `run_narrow_phase`. Heightmap is partially registered. The two
  unbounded shapes therefore enter the dispatcher through *different*
  paths than every other shape.
- **No compound colliders.** A `Collider` holds a single `Shape`. A
  robot link with a primary box plus a small spherical bump must spawn
  two colliders attached to the same body — currently expressed
  through duplicated body references rather than a real compound type.
- **No `Cylinder` primitive.** [scene-format](../scene-format/scene-format.md)
  notes that URDF `cylinder` is loaded as a *capsule approximation*
  today. A capsule is not a cylinder — wheels, drums, and pegs all
  behave wrong on flat ground.
- **No CCD.** Fast-moving objects can tunnel through thin geometry.
  Velocity-swept AABBs in the broad phase mitigate but do not solve
  this; the narrow phase has no time-of-impact path.
- **Stubbed Heightmap pairs not covered by the heightmap doc.** The
  [heightmap rewrite](../heightmap/heightmap.md) finishes Box–HM and
  Mesh–HM. The remaining stubs (`Cone–HM`, `Ellipsoid–HM`, `HM–Plane`,
  `HM–HM`) currently either return `false` or route a non-convex
  shape through GJK, which is unsafe.

## Motivation
Every level-1+ feature in [the roadmap](../roadmap/roadmap.md) implicitly
assumes a complete, uniformly integrated collision surface. Tracking
these gaps under one umbrella avoids the failure mode of a downstream
feature (URDF import, benchmark scenarios, sensor raycasting) catching
them late and stalling. Each subsection below can ship as its own PR;
this doc is a coordination point, not a single deliverable.

## Proposed Approach

### 1. Plane / Heightmap broadphase unification
The natural moment to fix this is the
[broadphase-bvh](../broadphase-bvh/broadphase-bvh.md) refactor — the
abstract broad-phase interface should accept "unbounded shape"
registration explicitly rather than implicitly via `BP_INVALID_HANDLE`
([BroadPhase.hpp:54](../../../include/rbc/BroadPhase.hpp)). Two
options to evaluate:

1. **Sentinel registration.** Unbounded shapes register with the
   broad phase but always pair against every dynamic. The broad phase
   yields the pair list with a uniform shape; the narrow-phase loop
   stops special-casing planes.
2. **Per-cell registration for heightmaps.** Each terrain cell becomes
   a finite AABB in the broad phase. Higher per-frame cost but unifies
   the heightmap with finite shapes for free; pairs naturally with
   the BVH backend.

Pick one, document the rationale in the broadphase-bvh doc, and remove
the explicit plane / heightmap branches from
[CollisionPipeline](../../../include/rbps/CollisionPipeline.hpp).

### 2. Compound colliders
Add a `Compound` shape variant that owns an array of `(Shape, m3d::tf)`
pairs. Variant-level dispatchers (`compute_aabb`, `support`,
`feature_for_normal`) iterate the children. Narrow phase against a
non-compound `T` recurses on each child; compound–compound is the
cartesian product (n·m).

Storage: keep `Shape` non-recursive for now by making `Compound` hold
a non-owning `ShapeView*` array, mirroring the
[ConvexHull](../convex-hull/convex-hull.md) and
[Heightmap](../heightmap/heightmap.md) patterns. SoA layout in
[Collider.hpp](../../../include/rbc/Collider.hpp) is unaffected
because the compound's children live in their own buffer.

### 3. Cylinder primitive
True right cylinder with `radius` + `half_height` along a body-local
axis. Convex, GJK-friendly:
- `support(cyl, dir)` — pick top or bottom disc by `dir.z` sign,
  return the disc point in the `dir.xy` direction. Standard formula.
- `face_corners(cyl, tf, dir, out[4])` — return the disc rim sampled
  at four points when the contact normal is along the axis (matches
  the contact-feature contract introduced by
  [contact-generation](../contact-generation/contact-generation.md)).
- `is_gjk_convex == true`.

Once available, change [scene-format](../scene-format/scene-format.md)
to map URDF `cylinder` to `rbc::Cylinder` (instead of a capsule).

### 4. CCD — scoping only, not implementation
Out of scope for v1; documented here so it is tracked. When tackled,
the design choices are conservative (swept-AABB + bisection for time
of impact) vs. analytical (per-pair TOI). Defer until a concrete user
need surfaces (likely from
[benchmark-harness](../benchmark-harness/benchmark-harness.md)
high-velocity scenarios).

### 5. Remaining stubbed Heightmap pairs
After the heightmap doc lands:
- `Cone–Heightmap`, `Ellipsoid–Heightmap` — both convex, both
  reachable through the same per-triangle path the heightmap doc
  builds. Add specialisations that reuse the convex-vs-triangle helper
  (a generalised version of `sphere_vs_triangle` /
  `capsule_vs_triangle` from
  [TriangleUtils.hpp](../../../include/rbc/analytic/TriangleUtils.hpp)).
- `Heightmap–Plane`, `Heightmap–Heightmap` — return `false` with a
  clear rationale (two unbounded shapes); add tests asserting the
  contract so the stubs can't silently start producing contacts.
- `ConvexHull–Heightmap` (after [convex-hull](../convex-hull/convex-hull.md)
  ships) — drops out of the generic convex-vs-triangle helper.

## Risks / Open Questions
- **Compound colliders touch the SoA storage layout** for colliders
  even though the variant change is contained. The
  [storage](../../../include/storage/Soa.hpp) macros may need a
  variable-length child handle column; design this with an eye on the
  body-collider relationship before implementing.
- **CCD interacts with XPBD.** Position-based correction can produce
  non-physical "impulses" when a TOI-clipped step is later resolved.
  Worth a separate design doc when CCD is taken on.
- **Plane/HM unification picks a long-lived API.** Whichever path is
  chosen (sentinel vs. per-cell registration) becomes the stable
  contract for the broad phase. Validate against
  [sensors-gpu](../sensors-gpu/sensors-gpu.md) raycasting before
  freezing — it queries the same broad phase.
- **Cylinder vs capsule in URDF**: existing scenes loaded via the
  capsule-approximation path will behave differently after the
  switch. Add a one-flag fallback in the URDF loader during the
  transition.
