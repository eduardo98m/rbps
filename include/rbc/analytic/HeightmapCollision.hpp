#pragma once
// ── Heightmap collision detection ─────────────────────────────────────────────
// For each query, determines the relevant grid cells by projecting the query
// shape's AABB onto the heightmap grid, then tests the resulting triangles.
// The heightmap transform tf is ignored — world position is baked into
// HeightmapData::origin (see Heightmap.hpp).

#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Capsule.hpp"
#include "rbc/analytic/TriangleUtils.hpp"
#include <cstdint>
#include <algorithm>

namespace rbc
{

    namespace detail
    {
        // Find the grid cell range overlapping a world-space AABB.
        inline void heightmap_cell_range(const HeightmapData &hd,
                                         const AABB &query_aabb,
                                         int &r_min, int &r_max,
                                         int &c_min, int &c_max)
        {
            r_min = std::max(0, static_cast<int>((query_aabb.min.z - hd.origin.z) / hd.scale.z));
            r_max = std::min(static_cast<int>(hd.rows) - 2,
                             static_cast<int>((query_aabb.max.z - hd.origin.z) / hd.scale.z));
            c_min = std::max(0, static_cast<int>((query_aabb.min.x - hd.origin.x) / hd.scale.x));
            c_max = std::min(static_cast<int>(hd.cols) - 2,
                             static_cast<int>((query_aabb.max.x - hd.origin.x) / hd.scale.x));
        }

        // Iterate relevant triangles, call per_tri callback, keep deepest contact.
        // The callback fills a single-point ContactManifold (num_points=1) per
        // triangle; we keep the deepest one as the output manifold.
        template <typename PerTriFn>
        inline bool heightmap_test(const Heightmap &hm, const AABB &query_aabb,
                                   PerTriFn per_tri, ContactManifold &out)
        {
            if (!hm.data)
                return false;
            const HeightmapData &hd = *hm.data;

            int r0, r1, c0, c1;
            heightmap_cell_range(hd, query_aabb, r0, r1, c0, c1);

            bool hit = false;
            m3d::scalar best_depth = 0.0;
            ContactManifold candidate;

            for (int r = r0; r <= r1; ++r)
            {
                for (int c = c0; c <= c1; ++c)
                {
                    const m3d::vec3 V00 = heightmap_vertex(hd, r, c);
                    const m3d::vec3 V10 = heightmap_vertex(hd, r + 1, c);
                    const m3d::vec3 V01 = heightmap_vertex(hd, r, c + 1);
                    const m3d::vec3 V11 = heightmap_vertex(hd, r + 1, c + 1);

                    // Two triangles per cell
                    const m3d::vec3 tris[2][3] = { {V00, V10, V01}, {V10, V11, V01} };
                    for (const auto &tri : tris)
                    {
                        const m3d::vec3 n = m3d::normalize(
                            m3d::cross(tri[1] - tri[0], tri[2] - tri[0]));
                        if (per_tri(tri[0], tri[1], tri[2], n, candidate))
                        {
                            const m3d::scalar d = candidate.points[0].penetration_depth;
                            if (d > best_depth)
                            {
                                best_depth = d;
                                out = candidate;
                                hit = true;
                            }
                        }
                    }
                }
            }
            return hit;
        }
    } // namespace detail

    // ── Sphere vs Heightmap ───────────────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Sphere, Heightmap>
    {
        static bool test(const Sphere &sphere, const m3d::tf &tf_sphere,
                         const Heightmap &hm, const m3d::tf & /*tf_hm*/,
                         ContactManifold &out)
        {
            const AABB query = compute_aabb(sphere, tf_sphere);
            return detail::heightmap_test(hm, query,
                [&](const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C,
                    const m3d::vec3 &n, ContactManifold &c) {
                    return tri::sphere_vs_triangle(tf_sphere.pos, sphere.radius,
                                                   A, B, C, n, c);
                }, out);
        }
    };
    template <>
    struct CollisionAlgorithm<Heightmap, Sphere> : CollisionAlgorithmSym<Heightmap, Sphere>
    {
    };

    // ── Capsule vs Heightmap ──────────────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Capsule, Heightmap>
    {
        static bool test(const Capsule &cap, const m3d::tf &tf_cap,
                         const Heightmap &hm, const m3d::tf & /*tf_hm*/,
                         ContactManifold &out)
        {
            m3d::vec3 p1, p2;
            capsule_endpoints(cap, tf_cap, p1, p2);
            const AABB query = compute_aabb(cap, tf_cap);

            return detail::heightmap_test(hm, query,
                [&](const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C,
                    const m3d::vec3 &n, ContactManifold &c) {
                    return tri::capsule_vs_triangle(p1, p2, cap.radius, A, B, C, n, c);
                }, out);
        }
    };
    template <>
    struct CollisionAlgorithm<Heightmap, Capsule> : CollisionAlgorithmSym<Heightmap, Capsule>
    {
    };

    // ── Box vs Heightmap ──────────────────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Box, Heightmap>
    {
        static bool test(const Box &box, const m3d::tf &tf_box,
                         const Heightmap &hm, const m3d::tf & /*tf_hm*/,
                         ContactManifold &out)
        {
            const AABB query = compute_aabb(box, tf_box);
            return detail::heightmap_test(hm, query,
                [&](const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C,
                    const m3d::vec3 &wn, ContactManifold &c) -> bool {
                    const m3d::scalar centre_dist = m3d::dot(tf_box.pos - A, wn);
                    const m3d::vec3   local_n     = tf_box.inverse_rotate_vector(wn);
                    const m3d::scalar box_r =
                        m3d::abs(local_n.x) * box.half_extents.x +
                        m3d::abs(local_n.y) * box.half_extents.y +
                        m3d::abs(local_n.z) * box.half_extents.z;
                    if (centre_dist > box_r) return false;

                    const m3d::vec3  proj    = tf_box.pos - wn * centre_dist;
                    const m3d::vec3  closest = tri::closest_point_on_triangle(proj, A, B, C);
                    const m3d::scalar edge_d = m3d::length(proj - closest);
                    if (edge_d > box_r) return false;

                    const m3d::scalar depth = box_r - centre_dist;
                    if (depth <= 0.0) return false;

                    c.normal                          = wn;
                    c.num_points                      = 1;
                    c.points[0].position              = closest;
                    c.points[0].penetration_depth     = depth;
                    return true;
                }, out);
        }
    };
    template <>
    struct CollisionAlgorithm<Heightmap, Box> : CollisionAlgorithmSym<Heightmap, Box>
    {
    };

    // ── Heightmap vs Heightmap — not supported ────────────────────────────────
    template <>
    struct CollisionAlgorithm<Heightmap, Heightmap>
    {
        static bool test(const Heightmap &, const m3d::tf &,
                         const Heightmap &, const m3d::tf &, ContactManifold &)
        {
            return false;
        }
    };

    // ── Ellipsoid / Cone / Mesh vs Heightmap — GJK-based via support ──────────
    // The default CollisionAlgorithm<A,B> template uses GJK + EPA.
    // For concave meshes this is incorrect; for convex shapes it works.
    // No explicit specialisation needed — the primary template handles these.

} // namespace rbc