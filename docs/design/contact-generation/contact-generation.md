@page feature_contact_generation Contact-Manifold Generalisation

# Contact-Manifold Generalisation

## Current State
Only Box–Box produces a true face/face manifold (1–4 contact points)
via the SAT specialisation in
[include/rbc/analytic/BoxBox.hpp](../../../include/rbc/analytic/BoxBox.hpp).
Box–Plane and Capsule–Plane also produce multi-point manifolds inside
[analytic/PlaneCollision.hpp](../../../include/rbc/analytic/PlaneCollision.hpp).
Every other convex pair routes through GJK + EPA + the generic
clipping in
[include/rbc/gjk/ContactManifoldGenerator.hpp](../../../include/rbc/gjk/ContactManifoldGenerator.hpp);
shapes without flat faces (Sphere, Capsule, Ellipsoid, Cone) fall back
to the disc approximation in
[shapes/FaceHelpers.hpp](../../../include/rbc/shapes/FaceHelpers.hpp).

The disc fallback gives a stable 4-point manifold but the four points
are placed on a square patch around the EPA support point — they are
NOT physically meaningful contact points and produce visible jitter on
resting stacks.

## Motivation
- Stable resting contact for boxes on heightmaps, capsules on stairs,
  meshes on planes — none of which currently get a true manifold.
- Correct friction cone behaviour requires multiple contact points;
  the disc fallback over-estimates the friction surface area.
- Box–Box being special-cased ages poorly as new shapes are added.

## Proposed Approach
1. Promote the box face-corners helper in
   [shapes/Box.hpp](../../../include/rbc/shapes/Box.hpp) — `face_corners` —
   to a richer "feature" abstraction that also reports vertex / edge
   features when the EPA normal is closer to a vertex / edge than a
   face. New struct in `shapes/Feature.hpp`:
   ```cpp
   struct ContactFeature { enum Kind { Face, Edge, Vertex } kind; vec3 pts[4]; int n; };
   ```
2. Per-shape `feature_for_normal(shape, tf, normal)` overloads (drop-in
   replacement for `face_corners`). Box, Mesh, and Heightmap can return
   real face polygons; Sphere / Capsule / Ellipsoid / Cone return
   single-vertex features (`n == 1`).
3. Rewrite `generate_manifold` in
   [ContactManifoldGenerator.hpp](../../../include/rbc/gjk/ContactManifoldGenerator.hpp)
   to clip face-vs-face, project edge-vs-face, and short-circuit
   vertex-vs-anything — the existing Sutherland–Hodgman code generalises.
4. Keep the disc fallback only for Sphere–Sphere / Sphere–Capsule
   contacts where a single-point manifold is actually correct.
5. Box–Box can remove its bespoke clipping and call the new generic
   path; verify the existing `test_box_box` cases still pass before
   deleting.

## Risks / Open Questions
- The `Feature` enum changes the public dispatch surface (`face_corners`
  is currently part of the per-shape contract). Either keep both or
  bump `rbc` API version.
- Manifold quality for highly skewed contacts (sharp edge into flat
  face) needs a coherent reduction strategy; the existing `reduce_to_4`
  helper can be reused but tune the metric.
- Mesh-Mesh face-feature pairs scale `O(faces²)`; a BVH (see
  [broadphase-bvh.md](../broadphase-bvh/broadphase-bvh.md)) is a prerequisite for large
  meshes.
