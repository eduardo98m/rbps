#pragma once
// ============================================================================
//  PlaneCollision.hpp
//
//  Analytic convex-shape-vs-plane collision.  Planes are infinite half-spaces
//  and cannot enter the SAP broad-phase (infinite AABB).  Instead they live
//  in a separate list and are tested against every dynamic collider each frame
//  directly inside run_narrow_phase() in CollisionPipeline.cpp.
//
//  KEY INSIGHT: Any convex shape S collides with a plane iff its "deepest"
//  point (support in the -normal direction) lies on the wrong side of the
//  plane.
//
//    world_normal  = tf_plane.rotate_vector(plane.normal)
//    world_d       = plane.d + dot(world_normal, tf_plane.pos)
//    local_dir     = tf_shape.inverse_rotate_vector(-world_normal)
//    deepest_local = support(shape, local_dir)
//    deepest_world = tf_shape.pos + tf_shape.rotate_vector(deepest_local)
//
//    depth = world_d - dot(deepest_world, world_normal)
//    if depth > 0 → collision
//
//  Normal convention (A→B):
//    CollisionAlgorithm<Shape, Plane>:  normal points from shape toward plane
//      → normal = +world_normal  (pushing shape away)
//    CollisionAlgorithm<Plane, Shape>:  normal is flipped by CollisionAlgorithmSym
//
//  All specialisations fill ContactManifold (1 point: the deepest support
//  point on the shape surface).
// ============================================================================

#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Plane.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Mesh.hpp"

namespace rbc
{

    // ── Internal helper ───────────────────────────────────────────────────────
    namespace detail
    {
        // Returns true and fills `out` when `shape` penetrates `plane`.
        // Works for ANY convex shape that has a support() function.
        template <typename A>
        inline bool convex_vs_plane(const A       &shape,   const m3d::tf &tf_shape,
                                    const Plane   &plane,   const m3d::tf &tf_plane,
                                    ContactManifold &out)
        {
            // World-space plane parameters
            const m3d::vec3   world_n = tf_plane.rotate_vector(plane.normal);
            const m3d::scalar world_d = plane.d + m3d::dot(world_n, tf_plane.pos);

            // Deepest point of `shape` in the -world_n direction
            const m3d::vec3 local_dir     = tf_shape.inverse_rotate_vector(-world_n);
            const m3d::vec3 local_support = support(shape, local_dir);
            const m3d::vec3 world_support = tf_shape.pos + tf_shape.rotate_vector(local_support);

            // Signed distance from the deepest point to the plane.
            // Negative means the point is on the wrong side (inside the half-space).
            const m3d::scalar signed_dist = m3d::dot(world_support, world_n) - world_d;
            if (signed_dist >= 0.0)
                return false; // shape is entirely above or just touching the plane

            // Normal: push the shape in +world_n direction (away from the plane)
            // Convention is A→B, so for <Shape, Plane> normal points toward the plane.
            // This is enough for plane v sphere, but for boxes and capsules we have more contact details to fill in the manifold.
            out.normal                    = -world_n;
            out.num_points                = 1;
            out.points[0].penetration_depth = -signed_dist;      // positive depth
            out.points[0].position          = world_support;     // deepest contact point
            return true;
        }
    } // namespace detail

    // ── Partial specialisations for all convex shapes vs Plane ────────────────
    //
    //  The macro generates:
    //    CollisionAlgorithm<ShapeType, Plane>  — analytic convex_vs_plane
    //    CollisionAlgorithm<Plane, ShapeType>  — symmetric shim (flips normal)
    //
#define RBC_PLANE_SPEC(ShapeType)                                               \
    template <>                                                                  \
    struct CollisionAlgorithm<ShapeType, Plane>                                  \
    {                                                                            \
        static bool test(const ShapeType &a, const m3d::tf &tf_a,              \
                         const Plane     &b, const m3d::tf &tf_b,              \
                         ContactManifold &out)                                   \
        { return detail::convex_vs_plane(a, tf_a, b, tf_b, out); }             \
    };                                                                           \
    template <>                                                                  \
    struct CollisionAlgorithm<Plane, ShapeType>                                  \
        : CollisionAlgorithmSym<Plane, ShapeType> {};

    RBC_PLANE_SPEC(Sphere)
    RBC_PLANE_SPEC(Box)
    RBC_PLANE_SPEC(Ellipsoid)
    RBC_PLANE_SPEC(Capsule)
    RBC_PLANE_SPEC(Cone)
    RBC_PLANE_SPEC(Mesh) // convex-hull support only — for concave meshes use MeshCollision

#undef RBC_PLANE_SPEC

    // ── Plane vs Plane — always false (two infinite half-spaces) ──────────────
    template <>
    struct CollisionAlgorithm<Plane, Plane>
    {
        static bool test(const Plane &, const m3d::tf &,
                         const Plane &, const m3d::tf &,
                         ContactManifold &) { return false; }
    };

    // ── Plane vs Heightmap / Heightmap vs Plane — not meaningful ──────────────
    // A heightmap already acts as a ground surface; intersecting it with a plane
    // is undefined in this engine.  Handle at the simulation layer if needed.
    template <>
    struct CollisionAlgorithm<Plane, Heightmap>
    {
        static bool test(const Plane &, const m3d::tf &,
                         const Heightmap &, const m3d::tf &,
                         ContactManifold &) { return false; }
    };
    template <>
    struct CollisionAlgorithm<Heightmap, Plane>
        : CollisionAlgorithmSym<Heightmap, Plane> {};

} // namespace rbc