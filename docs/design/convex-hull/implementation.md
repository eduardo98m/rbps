@page feature_convex_hull_impl Convex Hull — Implementation Notes

# Convex Hull — Implementation Notes

Companion to [convex-hull.md](convex-hull.md). The design doc explains
*why*; this file explains *how* and tracks task progress.

## Recap

`ConvexHull` is a new `Shape` variant alternative for arbitrary convex
polytopes. It re-uses the existing GJK + EPA + manifold pipeline by
flipping `is_gjk_convex(const ConvexHull*) == true` and providing a
vertex-scan `support()`. Storage is non-owning (caller owns the vertex
array), mirroring `Mesh` and `Heightmap`. Adding the variant also grows
the dispatcher table from 8×8 to 9×9 automatically because the table
size is driven by `std::variant_size_v<decltype(Shape::v)>` in
[Dispatcher.hpp:194](../../../include/rbc/Dispatcher.hpp).

## Task list

- [x] **Task 0** — write this doc.
- [x] **Task 1** — `include/rbc/shapes/ConvexHull.hpp`: `ConvexHullData`,
      `ConvexHull`, `convex_hull_data_create`, `convex_hull_data_destroy`.
- [x] **Task 2** — per-shape free functions: `support`, `compute_aabb`,
      `is_gjk_convex`, `representative_radius`, `face_corners`,
      `compute_volume`, `compute_inertia_tensor` (with debug warning when
      face data missing).
- [x] **Task 3** — wired into `Shape` variant in
      [ShapeTypes.hpp](../../../include/rbc/shapes/ShapeTypes.hpp).
- [x] **Task 4** — `RBC_PLANE_SPEC(ConvexHull)` added in
      [PlaneCollision.hpp](../../../include/rbc/analytic/PlaneCollision.hpp).
- [x] **Task 5** — four analytic tests under
      [tests/rbc/analytic/](../../../tests/rbc/analytic/), all passing.
- [x] **Task 6** — visr bindings: `ConvexHullSnap`, builder arm, raylib
      draw arm (real wireframe), panels label + properties.
- [x] **Task 7** — demo: tetrahedron body in
      [src/visr_demo/main.cpp](../../../src/visr_demo/main.cpp)
      (`build_convex_hull_demo`).
- [x] **Task 8** — `make build` clean; `make test` 36/36 passing;
      visualizer is the user's smoke-run.

## File-by-file touch list

### `rbc` (collision core)

| File | Change |
|---|---|
| `include/rbc/shapes/ConvexHull.hpp` | **NEW** header — see contract below. |
| `include/rbc/shapes/ShapeTypes.hpp` | `+#include` after Mesh; append `ConvexHull` to the `std::variant<…>` at line 67; add `Shape(const ConvexHull&)` ctor after line 86. |
| `include/rbc/analytic/PlaneCollision.hpp` | Append `RBC_PLANE_SPEC(ConvexHull)` after line 84. |

The dispatcher in `include/rbc/Dispatcher.hpp` and the GJK/EPA pipeline
in `include/rbc/gjk/*` need **no changes** — they pick up the new
variant arm automatically once `is_gjk_convex(const ConvexHull*) == true`.

### Tests

| File | Purpose |
|---|---|
| `tests/rbc/analytic/test_convex_hull_sphere.cpp` | Tetrahedron vs sphere — separated, face overlap, vertex overlap, GJK reference cross-check. |
| `tests/rbc/analytic/test_convex_hull_box.cpp` | Hull-encoding-of-a-box vs `rbc::Box` — depth/normal parity (sanity check). |
| `tests/rbc/analytic/test_convex_hull_plane.cpp` | Single-point manifold via `RBC_PLANE_SPEC(ConvexHull)`. |
| `tests/rbc/analytic/test_convex_hull_self.cpp` | Two tetrahedra — separated / resting / penetrating. |

### `visr` (visualization)

| File | Change |
|---|---|
| `include/visr/Snapshot.hpp` | New `ConvexHullSnap { vector<vec3> vertices; vector<uint32_t> face_indices; }`; append to `ShapeSnap` variant. |
| `include/visr/SnapshotBuilder.hpp` | New `operator()(const rbc::ConvexHull&)` arm in `ShapeToSnap` that copies the hull's vertices/faces into the snapshot. |
| `include/visr/ui/RaylibDraw.hpp` | New `if constexpr (... ConvexHullSnap)` branch — DrawLine3D along each triangle edge in world space; fallback to per-vertex DrawSphereWires if `face_indices` is empty. |
| `include/visr/ui/Panels.hpp` | Type-name arm (~L332) returning `"ConvexHull"`; properties arm (~L383) showing vertex/face counts. |
| `src/visr_demo/main.cpp` | Instantiate a tetrahedron-as-`ConvexHull` body, drop it onto the floor, free the data on shutdown. |

## ConvexHull contract (header sketch)

```cpp
namespace rbc {

struct ConvexHullData {
    const m3d::vec3* vertices;       // local-space, non-owning
    const uint32_t*  face_indices;   // CCW triangle indices; nullptr if no faces
    const uint32_t*  face_offsets;   // optional polygon-face offsets (currently unused: triangulated faces only)
    const m3d::vec3* face_normals;   // owned by helper when face data given
    uint32_t         vert_count;
    uint32_t         face_count;     // triangle count when face_indices != nullptr; else 0
    AABB             local_aabb;     // precomputed
};

struct ConvexHull {
    const ConvexHullData* data;
    ConvexHull() : data(nullptr) {}
    explicit ConvexHull(const ConvexHullData* d) : data(d) {}
    bool operator==(const ConvexHull& o) const { return data == o.data; }
    bool operator!=(const ConvexHull& o) const { return data != o.data; }
};

// Helper: precomputes face_normals (if faces given) + local_aabb.
ConvexHullData* convex_hull_data_create(
    const m3d::vec3* vertices, uint32_t vert_count,
    const uint32_t*  face_indices = nullptr,
    uint32_t face_count = 0);

void convex_hull_data_destroy(ConvexHullData*);

// Per-shape contract (ADL free functions):
m3d::vec3   support(const ConvexHull&, const m3d::vec3& dir);
AABB        compute_aabb(const ConvexHull&, const m3d::tf&);
constexpr bool is_gjk_convex(const ConvexHull*) { return true; }
m3d::scalar representative_radius(const ConvexHull&);
int         face_corners(const ConvexHull&, const m3d::tf&,
                         const m3d::vec3& dir, m3d::vec3 out[4]);
m3d::scalar compute_volume(const ConvexHull&);
m3d::smat3  compute_inertia_tensor(const ConvexHull&);

} // namespace rbc
```

## Inertia / volume fallback contract

`compute_volume` and `compute_inertia_tensor` need topology to do the
exact tetrahedral decomposition. The contract is:

| Input | Result |
|---|---|
| Faces present (`face_count > 0`) | Real tetrahedral decomposition from the centroid; unit density. Standard formula. |
| Vertices only (`face_count == 0`) | **Fallback**: inertia / volume of the local AABB (Box formula). One-shot debug message to `stderr` so the caller is aware: `"[rbc] ConvexHull inertia: face data missing; using AABB approximation"`. The message is gated behind `#ifndef NDEBUG` and uses a function-local `static bool warned` so it fires once per process. |

This matches the user's preference: simplified inertia is acceptable, but
the user must be told. Document in the doxygen header for both functions.

## Plane fast path

Once `RBC_PLANE_SPEC(ConvexHull)` is added, ConvexHull–Plane bypasses
GJK/EPA and uses the single-point `detail::convex_vs_plane` helper, the
same path Sphere/Ellipsoid/Cone/Mesh use. The macro takes care of the
symmetric `(Plane, ConvexHull)` direction via `CollisionAlgorithmSym`.

## Visualizer integration

`ConvexHullSnap` carries the actual vertex + face data (small — typical
hulls are 4–32 vertices). The snapshot builder copies them every frame.
For shapes that are too large to deep-copy per frame, a future
optimization would be a "shape registry" that the renderer caches by
ID; for now the eager copy is fine and the simplest possible path.

The renderer draws each triangle edge with `DrawLine3D` after rotating
the local vertices by `world_rot` and translating by `world_pos`. If
`face_indices` is empty the renderer falls back to a per-vertex marker
so vertex-only hulls remain visible.

## Usage example

```cpp
#include "rbc/shapes/ConvexHull.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

// 1. Static tetrahedron data (caller owns).
static const m3d::vec3 verts[4] = {
    {  1.0,  1.0,  1.0 },
    { -1.0, -1.0,  1.0 },
    { -1.0,  1.0, -1.0 },
    {  1.0, -1.0, -1.0 },
};
static const uint32_t faces[4 * 3] = {
    0, 1, 2,
    0, 3, 1,
    0, 2, 3,
    1, 3, 2,
};

// 2. Pre-compute face normals + local AABB.
rbc::ConvexHullData* hull = rbc::convex_hull_data_create(verts, 4, faces, 4);

// 3. Use it like any other shape.
rbc::Shape s = rbc::ConvexHull{hull};
rbc::AABB world_aabb = rbc::compute_aabb(s, body_tf);

// 4. Free at shutdown.
rbc::convex_hull_data_destroy(hull);
```

## Open questions resolved during planning

1. **Helper functions vs pure POD?** Ship `convex_hull_data_create` /
   `_destroy` (mirrors Mesh). Saves every caller from re-implementing
   the face-normal precompute.
2. **Inertia without face data?** Fall back to AABB-of-hull inertia;
   emit a one-shot debug message so the caller is aware. Real
   tetrahedral formula when faces are present.
3. **Doc scope?** This file (separate from `convex-hull.md`) so the
   design doc stays at the ~1-page convention from
   [docs/design/README.md](../README.md).

## Out of scope (won't ship in this PR)

- Hull builder. Users pre-hull externally (qhull, libccd, Bullet).
- Owning `ConvexHullCollection` SoA — defer to scene-format work.
- ConvexHull–Mesh / ConvexHull–Heightmap analytic specialisations —
  they fall back to GJK against the convex side and the existing
  per-triangle/per-cell paths handle the non-convex side. A
  collider-completeness follow-up can do better.
- Convexity validation. Document caller responsibility for now.
- Non-triangulated faces (`face_offsets` is in the struct for future
  use but unused this iteration — only triangulated face data is
  consumed by `face_normals` precompute and visualizer rendering).
