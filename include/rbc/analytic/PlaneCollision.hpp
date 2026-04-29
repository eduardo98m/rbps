#pragma once

/**
 * @file PlaneCollision.hpp
 * @brief Analytic convex-shape-vs-Plane collision algorithms.
 * @ingroup rbc
 * @ingroup internals
 *
 * Planes are infinite half-spaces and cannot enter the SAP broad phase
 * (infinite AABB). They live in a separate list and are tested against
 * every dynamic collider each frame inside `run_narrow_phase` in
 * `CollisionPipeline.cpp`.
 *
 * @par Key insight
 * Any convex shape S collides with a plane iff its "deepest" point —
 * `support(S, -world_normal)` — lies on the wrong side of the plane.
 * This drives the generic `convex_vs_plane` helper used for shapes that
 * only need a single contact point. Box and Capsule have dedicated
 * specialisations that produce multi-point manifolds (4 corners or 2
 * endpoints).
 */

#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Plane.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Mesh.hpp"
#include "rbc/shapes/ConvexHull.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/Capsule.hpp"
#include <algorithm>

namespace rbc
{

    // ── Internal helper ───────────────────────────────────────────────────────
    namespace detail
    {
        // Returns true and fills `out` when `shape` penetrates `plane`.
        // Works for ANY convex shape that has a support() function.
        template <typename A>
        inline bool convex_vs_plane(const A &shape, const m3d::tf &tf_shape,
                                    const Plane &plane, const m3d::tf &tf_plane,
                                    ContactManifold &out)
        {
            const m3d::vec3 world_n = tf_plane.rotate_vector(plane.normal);
            const m3d::scalar world_d = plane.d + m3d::dot(world_n, tf_plane.pos);

            const m3d::vec3 local_dir = tf_shape.inverse_rotate_vector(-world_n);
            const m3d::vec3 local_support = support(shape, local_dir);
            const m3d::vec3 world_support = tf_shape.pos + tf_shape.rotate_vector(local_support);

            const m3d::scalar signed_dist = m3d::dot(world_support, world_n) - world_d;
            if (signed_dist >= 0.0)
                return false;

            out.normal = -world_n;
            out.num_points = 1;
            out.points[0].penetration_depth = -signed_dist;
            out.points[0].position = world_support;
            return true;
        }
    }

    // ── Partial specialisations for single-point convex shapes vs Plane ───────
#define RBC_PLANE_SPEC(ShapeType)                                  \
    template <>                                                    \
    struct CollisionAlgorithm<ShapeType, Plane>                    \
    {                                                              \
        static bool test(const ShapeType &a, const m3d::tf &tf_a,  \
                         const Plane &b, const m3d::tf &tf_b,      \
                         ContactManifold &out)                     \
        {                                                          \
            return detail::convex_vs_plane(a, tf_a, b, tf_b, out); \
        }                                                          \
    };                                                             \
    template <>                                                    \
    struct CollisionAlgorithm<Plane, ShapeType>                    \
        : CollisionAlgorithmSym<Plane, ShapeType>                  \
    {                                                              \
    };

    RBC_PLANE_SPEC(Sphere)
    RBC_PLANE_SPEC(Ellipsoid)
    RBC_PLANE_SPEC(Cone)
    RBC_PLANE_SPEC(Mesh)
    RBC_PLANE_SPEC(ConvexHull)

#undef RBC_PLANE_SPEC

    // ── Box vs Plane (Multi-point manifold) ───────────────────────────────────
    template <>
    struct CollisionAlgorithm<Box, Plane>
    {
        static bool test(const Box &a, const m3d::tf &tf_a, const Plane &b, const m3d::tf &tf_b, ContactManifold &out)
        {

            // 1. Transform plane to world space
            const m3d::vec3 world_n = tf_b.rotate_vector(b.normal);
            const m3d::scalar world_d = b.d + m3d::dot(world_n, tf_b.pos);

            // 2. Get Box local axes in world space
            const m3d::vec3 u[3] = {
                tf_a.rotate_vector(m3d::vec3(1, 0, 0)),
                tf_a.rotate_vector(m3d::vec3(0, 1, 0)),
                tf_a.rotate_vector(m3d::vec3(0, 0, 1))};

            // 3. Find reference face (most anti-aligned with plane normal)
            m3d::scalar min_alignment = std::numeric_limits<m3d::scalar>::max();
            int best_axis = -1;
            int best_sign = 0;

            for (int i = 0; i < 3; ++i)
            {
                for (int s = -1; s <= 1; s += 2)
                {
                    m3d::vec3 f_n = static_cast<m3d::scalar>(s) * u[i];
                    m3d::scalar align = m3d::dot(f_n, world_n);
                    if (align < min_alignment)
                    {
                        min_alignment = align;
                        best_axis = i;
                        best_sign = s;
                    }
                }
            }

            // 4. Calculate the 4 corners of the incident face
            m3d::vec3 face_center = tf_a.pos + static_cast<m3d::scalar>(best_sign) *
                                                   (best_axis == 0 ? a.half_extents.x * u[0] : best_axis == 1 ? a.half_extents.y * u[1]
                                                                                                              : a.half_extents.z * u[2]);

            int a1 = (best_axis + 1) % 3;
            int a2 = (best_axis + 2) % 3;
            const m3d::vec3 span1 = (a1 == 0 ? a.half_extents.x * u[0] : a1 == 1 ? a.half_extents.y * u[1]
                                                                                 : a.half_extents.z * u[2]);
            const m3d::vec3 span2 = (a2 == 0 ? a.half_extents.x * u[0] : a2 == 1 ? a.half_extents.y * u[1]
                                                                                 : a.half_extents.z * u[2]);

            m3d::vec3 face_pts[4] = {
                face_center + span1 + span2,
                face_center + span1 - span2,
                face_center - span1 - span2,
                face_center - span1 + span2};

            // 5. Check vertices against the plane (NO CLIPPING REQUIRED)
            out.num_points = 0;
            out.normal = -world_n; // Keeping your convention

            const m3d::scalar epsilon = 1e-4; // Collision margin/epsilon

            for (int i = 0; i < 4; ++i)
            {
                // Distance from point to plane
                m3d::scalar dist = m3d::dot(face_pts[i], world_n) - world_d;

                // If the point is behind the plane (plus small epsilon), it's a contact
                if (dist <= epsilon)
                {
                    out.points[out.num_points].position = face_pts[i];
                    out.points[out.num_points].penetration_depth = -dist; // Positive depth
                    out.num_points++;
                }
            }

            return out.num_points > 0;
        }
    };

    template <>
    struct CollisionAlgorithm<Plane, Box> : CollisionAlgorithmSym<Plane, Box>
    {
    };

    // ── Capsule vs Plane (Multi-point manifold) ───────────────────────────────
    template <>
    struct CollisionAlgorithm<Capsule, Plane>
    {
        static bool test(const Capsule &a, const m3d::tf &tf_a,
                         const Plane &b, const m3d::tf &tf_b,
                         ContactManifold &out)
        {
            // 1. Transform plane to world space
            const m3d::vec3 world_n = tf_b.rotate_vector(b.normal);
            const m3d::scalar world_d = b.d + m3d::dot(world_n, tf_b.pos);

            // 2. Get capsule internal segment endpoints in world space
            m3d::vec3 p1, p2;
            capsule_endpoints(a, tf_a, p1, p2);

            // 3. Distance from internal endpoints to the plane
            const m3d::scalar dist1 = m3d::dot(p1, world_n) - world_d;
            const m3d::scalar dist2 = m3d::dot(p2, world_n) - world_d;

            out.num_points = 0;
            out.normal = -world_n;

            // 4. Optimization: Cache the radius offset vector and collision margin
            const m3d::scalar epsilon = 1e-4; // Tune to match your physics scale
            const m3d::vec3 radius_offset = world_n * a.radius;
            const m3d::scalar threshold = a.radius + epsilon;

            // Check if endpoint 1 (hemisphere) penetrates
            if (dist1 <= threshold)
            {
                out.points[out.num_points].position = p1 - radius_offset;
                out.points[out.num_points].penetration_depth = a.radius - dist1;
                out.num_points++;
            }

            // Check if endpoint 2 (hemisphere) penetrates
            if (dist2 <= threshold)
            {
                out.points[out.num_points].position = p2 - radius_offset;
                out.points[out.num_points].penetration_depth = a.radius - dist2;
                out.num_points++;
            }

            return out.num_points > 0;
        }
    };

    template <>
    struct CollisionAlgorithm<Plane, Capsule> : CollisionAlgorithmSym<Plane, Capsule>
    {
    };

    // ── Plane vs Plane — always false (two infinite half-spaces) ──────────────
    template <>
    struct CollisionAlgorithm<Plane, Plane>
    {
        static bool test(const Plane &, const m3d::tf &,
                         const Plane &, const m3d::tf &,
                         ContactManifold &) { return false; }
    };

    // ── Plane vs Heightmap / Heightmap vs Plane — not meaningful ──────────────
    template <>
    struct CollisionAlgorithm<Plane, Heightmap>
    {
        static bool test(const Plane &, const m3d::tf &,
                         const Heightmap &, const m3d::tf &,
                         ContactManifold &) { return false; }
    };
    template <>
    struct CollisionAlgorithm<Heightmap, Plane>
        : CollisionAlgorithmSym<Heightmap, Plane>
    {
    };

} // namespace rbc