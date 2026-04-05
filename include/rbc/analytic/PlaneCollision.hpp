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
// ============================================================================

#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Plane.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Mesh.hpp"
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
        inline bool convex_vs_plane(const A       &shape,   const m3d::tf &tf_shape,
                                    const Plane   &plane,   const m3d::tf &tf_plane,
                                    ContactManifold &out)
        {
            const m3d::vec3   world_n = tf_plane.rotate_vector(plane.normal);
            const m3d::scalar world_d = plane.d + m3d::dot(world_n, tf_plane.pos);

            const m3d::vec3 local_dir     = tf_shape.inverse_rotate_vector(-world_n);
            const m3d::vec3 local_support = support(shape, local_dir);
            const m3d::vec3 world_support = tf_shape.pos + tf_shape.rotate_vector(local_support);

            const m3d::scalar signed_dist = m3d::dot(world_support, world_n) - world_d;
            if (signed_dist >= 0.0)
                return false; 

            out.normal                    = -world_n;
            out.num_points                = 1;
            out.points[0].penetration_depth = -signed_dist;      
            out.points[0].position          = world_support;     
            return true;
        }
    } 

    // ── Partial specialisations for single-point convex shapes vs Plane ───────
#define RBC_PLANE_SPEC(ShapeType)                                                \
    template <>                                                                  \
    struct CollisionAlgorithm<ShapeType, Plane>                                  \
    {                                                                            \
        static bool test(const ShapeType &a, const m3d::tf &tf_a,                \
                         const Plane     &b, const m3d::tf &tf_b,                \
                         ContactManifold &out)                                   \
        { return detail::convex_vs_plane(a, tf_a, b, tf_b, out); }               \
    };                                                                           \
    template <>                                                                  \
    struct CollisionAlgorithm<Plane, ShapeType>                                  \
        : CollisionAlgorithmSym<Plane, ShapeType> {};

    RBC_PLANE_SPEC(Sphere)
    RBC_PLANE_SPEC(Ellipsoid)
    RBC_PLANE_SPEC(Cone)
    RBC_PLANE_SPEC(Mesh) 

#undef RBC_PLANE_SPEC

    // ── Box vs Plane (Multi-point manifold) ───────────────────────────────────
    template <>
    struct CollisionAlgorithm<Box, Plane>
    {
        static bool test(const Box &a, const m3d::tf &tf_a,
                         const Plane &b, const m3d::tf &tf_b,
                         ContactManifold &out)
        {
            const m3d::vec3 world_n = tf_b.rotate_vector(b.normal);
            const m3d::scalar world_d = b.d + m3d::dot(world_n, tf_b.pos);

            // Get the scaled local axes in world space
            const m3d::vec3 axes[3] = {
                tf_a.rotate_vector(m3d::vec3(1, 0, 0)) * a.half_extents.x,
                tf_a.rotate_vector(m3d::vec3(0, 1, 0)) * a.half_extents.y,
                tf_a.rotate_vector(m3d::vec3(0, 0, 1)) * a.half_extents.z
            };

            struct DeepPoint { m3d::vec3 pos; m3d::scalar depth; };
            DeepPoint hits[8];
            int hit_count = 0;

            // Test all 8 corners of the box
            for (int i = 0; i < 8; ++i)
            {
                m3d::vec3 pt = tf_a.pos
                    + axes[0] * ((i & 1) ? 1.0f : -1.0f)
                    + axes[1] * ((i & 2) ? 1.0f : -1.0f)
                    + axes[2] * ((i & 4) ? 1.0f : -1.0f);

                m3d::scalar signed_dist = m3d::dot(pt, world_n) - world_d;
                if (signed_dist < 0.0)
                {
                    hits[hit_count++] = { pt, -signed_dist };
                }
            }

            if (hit_count == 0) return false;

            // Sort descending by depth so we keep the deepest points
            std::sort(hits, hits + hit_count, [](const DeepPoint& A, const DeepPoint& B) {
                return A.depth > B.depth;
            });

            out.normal = -world_n; // Pushing box away
            out.num_points = (hit_count > 4) ? 4 : hit_count; // Cap at 4 points max
            
            for (int i = 0; i < out.num_points; ++i)
            {
                out.points[i].position = hits[i].pos;
                out.points[i].penetration_depth = hits[i].depth;
            }

            return true;
        }
    };

    template <>
    struct CollisionAlgorithm<Plane, Box> : CollisionAlgorithmSym<Plane, Box> {};

    // ── Capsule vs Plane (Multi-point manifold) ───────────────────────────────
    template <>
    struct CollisionAlgorithm<Capsule, Plane>
    {
        static bool test(const Capsule &a, const m3d::tf &tf_a,
                         const Plane &b, const m3d::tf &tf_b,
                         ContactManifold &out)
        {
            const m3d::vec3 world_n = tf_b.rotate_vector(b.normal);
            const m3d::scalar world_d = b.d + m3d::dot(world_n, tf_b.pos);

            m3d::vec3 p1, p2;
            capsule_endpoints(a, tf_a, p1, p2);

            const m3d::scalar dist1 = m3d::dot(p1, world_n) - world_d;
            const m3d::scalar dist2 = m3d::dot(p2, world_n) - world_d;

            out.num_points = 0;
            out.normal = -world_n;

            // Check if top hemisphere penetrates
            if (dist1 < a.radius)
            {
                out.points[out.num_points].position = p1 - world_n * a.radius;
                out.points[out.num_points].penetration_depth = a.radius - dist1;
                out.num_points++;
            }

            // Check if bottom hemisphere penetrates
            if (dist2 < a.radius)
            {
                out.points[out.num_points].position = p2 - world_n * a.radius;
                out.points[out.num_points].penetration_depth = a.radius - dist2;
                out.num_points++;
            }

            return out.num_points > 0;
        }
    };

    template <>
    struct CollisionAlgorithm<Plane, Capsule> : CollisionAlgorithmSym<Plane, Capsule> {};


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
        : CollisionAlgorithmSym<Heightmap, Plane> {};

} // namespace rbc