#pragma once
// ── Generic Convex-Shape vs Plane (partial specialisation) ───────────────────
//
// KEY INSIGHT: Any convex shape S collides with a plane iff its "deepest" point
// (support in the -normal direction) lies on the wrong side of the plane.
//
//   world_normal  = tf_plane.rotate_vector(plane.normal)
//   world_d       = plane.d + dot(world_normal, tf_plane.pos)
//   local_dir     = tf_shape.inverse_rotate_vector(-world_normal)
//   deepest_local = support(shape, local_dir)
//   deepest_world = tf_shape.pos + tf_shape.rotate_vector(deepest_local)
//
//   If dot(deepest_world, world_normal) < world_d  →  collision
//   penetration_depth = world_d - dot(deepest_world, world_normal)
//
// This works for Sphere, Box, Ellipsoid, Capsule, Cone, and the convex-hull
// fallback of Mesh.  Plane vs Plane / Plane vs Heightmap return false.

#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Plane.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Mesh.hpp"

namespace rbc
{

    // ── Internal helper ───────────────────────────────────────────────────────
    namespace detail
    {
        template <typename A>
        inline bool convex_vs_plane(const A &shape, const m3d::tf &tf_shape,
                                    const Plane &plane, const m3d::tf &tf_plane,
                                    ContactManifold &out)
        {
            const m3d::vec3   world_n = tf_plane.rotate_vector(plane.normal);
            const m3d::scalar world_d = plane.d + m3d::dot(world_n, tf_plane.pos);

            // Deepest point of shape in the -world_n direction
            const m3d::vec3 local_dir     = tf_shape.inverse_rotate_vector(-world_n);
            const m3d::vec3 local_support = support(shape, local_dir);
            const m3d::vec3 world_support = tf_shape.pos + tf_shape.rotate_vector(local_support);

            const m3d::scalar depth = m3d::dot(world_support, world_n) - world_d;
            if (depth >= 0.0) return false; // shape is entirely above or on plane

            out.normal            = world_n;      // push shape in +normal direction
            out.penetration_depth = -depth;
            out.pos               = world_support; // deepest contact point
            return true;
        }
    } // namespace detail

    // ── Partial specialisations for all convex shapes vs Plane ────────────────
#define RBC_PLANE_SPEC(ShapeType)                                             \
    template <>                                                                \
    struct CollisionAlgorithm<ShapeType, Plane>                                \
    {                                                                          \
        static bool test(const ShapeType &a, const m3d::tf &tf_a,             \
                         const Plane &b, const m3d::tf &tf_b,                 \
                         ContactManifold &out)                                         \
        { return detail::convex_vs_plane(a, tf_a, b, tf_b, out); }           \
    };                                                                         \
    template <>                                                                \
    struct CollisionAlgorithm<Plane, ShapeType>                                \
        : CollisionAlgorithmSym<Plane, ShapeType> {};

    RBC_PLANE_SPEC(Sphere)
    RBC_PLANE_SPEC(Box)
    RBC_PLANE_SPEC(Ellipsoid)
    RBC_PLANE_SPEC(Capsule)
    RBC_PLANE_SPEC(Cone)
    RBC_PLANE_SPEC(Mesh) // convex-hull support — for concave meshes use MeshCollision

#undef RBC_PLANE_SPEC

    // ── Plane vs Plane — degenerate, always false ─────────────────────────────
    template <>
    struct CollisionAlgorithm<Plane, Plane>
    {
        static bool test(const Plane &, const m3d::tf &,
                         const Plane &, const m3d::tf &,
                         ContactManifold &) { return false; }
    };

    // ── Plane vs Heightmap / Heightmap vs Plane — skip ────────────────────────
    // Heightmap already covers the infinite-plane role; intersecting two of them
    // is undefined. Handle in simulation layer if needed.
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