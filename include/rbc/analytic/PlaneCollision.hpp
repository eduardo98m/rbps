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

#undef RBC_PLANE_SPEC

    // ── ConvexHull vs Plane (Multi-point manifold) ────────────────────────────
    template <>
    struct CollisionAlgorithm<ConvexHull, Plane>
    {
        static bool test(const ConvexHull &a, const m3d::tf &tf_a,
                         const Plane &b, const m3d::tf &tf_b,
                         ContactManifold &out)
        {
            if (!a.data || a.data->vert_count == 0)
                return false;

            const m3d::vec3 world_n = tf_b.rotate_vector(b.normal);
            const m3d::scalar world_d = b.d + m3d::dot(world_n, tf_b.pos);
            const m3d::vec3 local_n = tf_a.inverse_rotate_vector(world_n);
            const m3d::scalar local_d = world_d - m3d::dot(tf_a.pos, world_n);

            constexpr m3d::scalar epsilon = 1e-4f;

            // ── EARLY OUT: deepest support point is above the plane ──────────
            const m3d::vec3 deepest_local = support(a, -local_n);
            if (m3d::dot(deepest_local, local_n) - local_d > epsilon)
                return false;

            // ── STEP 1: Gather ALL penetrating vertices (local space) ────────
            struct TempPoint
            {
                m3d::vec3 pos;
                m3d::scalar depth;
            };
            TempPoint temp_points[32];
            int temp_count = 0;

            for (uint32_t i = 0; i < a.data->vert_count; ++i)
            {
                const m3d::vec3 &lv = a.data->vertices[i];
                const m3d::scalar dist = m3d::dot(lv, local_n) - local_d;
                if (dist <= epsilon && temp_count < 32)
                    temp_points[temp_count++] = {lv, -dist};
            }

            if (temp_count == 0)
                return false;

            out.normal = -world_n;
            out.num_points = 0;

            // ── STEP 2: Manifold Reduction → best 4 contacts ─────────────────
            if (temp_count <= 4)
            {
                // Trivial: keep all points
                for (int i = 0; i < temp_count; ++i)
                {
                    out.points[i].position = tf_a.transform_point(temp_points[i].pos);
                    out.points[i].penetration_depth = temp_points[i].depth;
                }
                out.num_points = temp_count;
            }
            else
            {
                // ── Area-Maximization (4-point reduction) ────────────────────
                //
                // Strategy (all work is done in the contact plane, i.e. the 2-D
                // projection along world_n, so we never need sqrt for area):
                //
                //  P0 – deepest penetration point   (most physically important)
                //  P1 – farthest from P0            (maximises the first edge)
                //  P2 – farthest from line P0-P1    (maximises triangle area)
                //  P3 – farthest from triangle P0P1P2 on the *opposite* side
                //       (maximises the final quadrilateral area)
                //
                // Cross-products are computed in 3-D but the plane-normal
                // component is what gives the signed area, which is exactly
                // what we want (and avoids building a 2-D basis).

                int idx[4] = {-1, -1, -1, -1};

                // ── P0: deepest point ────────────────────────────────────────
                {
                    m3d::scalar best = -1e18f;
                    for (int i = 0; i < temp_count; ++i)
                    {
                        if (temp_points[i].depth > best)
                        {
                            best = temp_points[i].depth;
                            idx[0] = i;
                        }
                    }
                }

                // ── P1: farthest from P0 (squared distance, no sqrt needed) ──
                {
                    const m3d::vec3 &p0 = temp_points[idx[0]].pos;
                    m3d::scalar best = -1e18f;
                    for (int i = 0; i < temp_count; ++i)
                    {
                        if (i == idx[0])
                            continue;
                        const m3d::vec3 d = temp_points[i].pos - p0;
                        const m3d::scalar dist2 = m3d::dot(d, d);
                        if (dist2 > best)
                        {
                            best = dist2;
                            idx[1] = i;
                        }
                    }
                }

                // ── P2: farthest from line P0–P1 ─────────────────────────────
                // |cross(edge, v - p0)| is proportional to the distance to the
                // line; we compare squared magnitudes to stay branch-friendly.
                {
                    const m3d::vec3 &p0 = temp_points[idx[0]].pos;
                    const m3d::vec3 edge = temp_points[idx[1]].pos - p0;
                    m3d::scalar best = -1e18f;
                    for (int i = 0; i < temp_count; ++i)
                    {
                        if (i == idx[0] || i == idx[1])
                            continue;
                        const m3d::vec3 c = m3d::cross(edge, temp_points[i].pos - p0);
                        const m3d::scalar dist2 = m3d::dot(c, c);
                        if (dist2 > best)
                        {
                            best = dist2;
                            idx[2] = i;
                        }
                    }
                }

                // ── P3: maximise signed area of the quadrilateral ────────────
                //
                // We want the point that extends the quad the most.
                // For each candidate we try it as the 4th vertex inserted
                // between each pair of consecutive triangle edges and pick
                // the insertion that gives the largest *additional* area.
                //
                // A stable, branch-free approach: score = max over the three
                // "pockets" of the signed area contribution (negative means
                // it is on the interior – we clamp to 0).
                //
                // pocket 0: outside edge P0→P1  → cross(P1-P0, X-P0)·n
                // pocket 1: outside edge P1→P2  → cross(P2-P1, X-P1)·n
                // pocket 2: outside edge P2→P0  → cross(P0-P2, X-P2)·n
                {
                    const m3d::vec3 &p0 = temp_points[idx[0]].pos;
                    const m3d::vec3 &p1 = temp_points[idx[1]].pos;
                    const m3d::vec3 &p2 = temp_points[idx[2]].pos;

                    // Pocket edge directions
                    const m3d::vec3 e01 = p1 - p0;
                    const m3d::vec3 e12 = p2 - p1;
                    const m3d::vec3 e20 = p0 - p2;

                    m3d::scalar best = -1e18f;
                    for (int i = 0; i < temp_count; ++i)
                    {
                        if (i == idx[0] || i == idx[1] || i == idx[2])
                            continue;

                        const m3d::vec3 &x = temp_points[i].pos;

                        // Signed area contribution for each pocket (dot with world_n
                        // gives the component perpendicular to the contact plane)
                        const m3d::scalar a0 = m3d::dot(m3d::cross(e01, x - p0), local_n);
                        const m3d::scalar a1 = m3d::dot(m3d::cross(e12, x - p1), local_n);
                        const m3d::scalar a2 = m3d::dot(m3d::cross(e20, x - p2), local_n);

                        // Best pocket area this candidate can contribute
                        // (negative → interior, not useful)
                        m3d::scalar score = a0;
                        if (a1 > score)
                            score = a1;
                        if (a2 > score)
                            score = a2;

                        if (score > best)
                        {
                            best = score;
                            idx[3] = i;
                        }
                    }
                }

                // ── Emit contacts ─────────────────────────────────────────────
                // Determine how many unique indices we found (idx[3] can be -1
                // if all remaining candidates were strictly interior).
                const int emit = (idx[3] >= 0) ? 4 : 3;
                for (int i = 0; i < emit; ++i)
                {
                    out.points[i].position = tf_a.transform_point(temp_points[idx[i]].pos);
                    out.points[i].penetration_depth = temp_points[idx[i]].depth;
                }
                out.num_points = emit;
            }

            return out.num_points > 0;
        }
    };

    template <>
    struct CollisionAlgorithm<Plane, ConvexHull> : CollisionAlgorithmSym<Plane, ConvexHull>
    {
    };

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

            // 3. Early out: support radius along plane normal
            constexpr m3d::scalar epsilon = 1e-4f;
            const m3d::scalar r = a.half_extents.x * std::abs(m3d::dot(u[0], world_n)) + a.half_extents.y * std::abs(m3d::dot(u[1], world_n)) + a.half_extents.z * std::abs(m3d::dot(u[2], world_n));
            const m3d::scalar center_dist = m3d::dot(tf_a.pos, world_n) - world_d;
            if (center_dist - r > epsilon)
                return false;

            // 4. Test all 8 corners
            out.normal = -world_n;
            out.num_points = 0;

            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    for (int sz = -1; sz <= 1; sz += 2)
                    {
                        const m3d::vec3 corner = tf_a.pos + static_cast<m3d::scalar>(sx) * a.half_extents.x * u[0] + static_cast<m3d::scalar>(sy) * a.half_extents.y * u[1] + static_cast<m3d::scalar>(sz) * a.half_extents.z * u[2];

                        const m3d::scalar dist = m3d::dot(corner, world_n) - world_d;
                        if (dist <= epsilon && out.num_points < 4)
                        {
                            out.points[out.num_points].position = corner;
                            out.points[out.num_points].penetration_depth = -dist;
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

    // ── Cylinder vs Plane (Multi-point manifold) ───────────────────────────────
    //
    // A cylinder has three distinct contact configurations with a plane:
    //
    //  Case 1 – Flat cap facing the plane (axis ∥ plane normal, |cos θ| ≈ 1):
    //           The entire cap rim could be in contact. We emit the 4 cardinal
    //           points of the penetrating cap as the manifold (stable 4-point
    //           contact, consistent with Box and ConvexHull conventions).
    //
    //  Case 2 – Oblique (general case, 0 < |cos θ| < threshold):
    //           Both caps contribute a single support point each (the point on
    //           each cap rim deepest into the plane). We emit up to 2 points.
    //
    //  Case 3 – Side contact (axis ⊥ plane normal, |cos θ| ≈ 0):
    //           The deepest point is on the curved surface midway along the
    //           axis. We emit a single point at the support along -world_n.
    //
    // The threshold between Case 1 and Case 2 is |cos θ| > cos(10°) ≈ 0.985.
    // The threshold between Case 2 and Case 3 is |cos θ| < cos(80°) ≈ 0.174.
    //
    template <>
    struct CollisionAlgorithm<Cylinder, Plane>
    {
        static bool test(const Cylinder &a, const m3d::tf &tf_a,
                         const Plane &b, const m3d::tf &tf_b,
                         ContactManifold &out)
        {
            // ── 1. Plane in world space ──────────────────────────────────────
            const m3d::vec3 world_n = tf_b.rotate_vector(b.normal);
            const m3d::scalar world_d = b.d + m3d::dot(world_n, tf_b.pos);

            // ── 2. Cylinder axis in world space ──────────────────────────────
            // Local +Y is the cylinder axis.
            const m3d::vec3 world_axis = tf_a.rotate_vector(m3d::vec3(0, 1, 0));

            // cos  between cylinder axis and plane normal
            const m3d::scalar cos_theta = m3d::dot(world_axis, world_n);

            // ── 3. Early out via support point ───────────────────────────────
            // The support of the cylinder along -world_n gives the deepest
            // possible point. If it is above the plane, nothing penetrates.
            const m3d::vec3 local_neg_n = tf_a.inverse_rotate_vector(-world_n);
            const m3d::vec3 support_local = support(a, local_neg_n);
            const m3d::vec3 support_world = tf_a.transform_point(support_local);
            const m3d::scalar support_dist = m3d::dot(support_world, world_n) - world_d;

            constexpr m3d::scalar epsilon = 1e-4f;
            constexpr m3d::scalar parallel_threshold = 0.985f;      // cos(10°)
            constexpr m3d::scalar perpendicular_threshold = 0.174f; // cos(80°)

            if (support_dist > epsilon)
                return false;

            out.normal = -world_n;
            out.num_points = 0;

            const m3d::scalar abs_cos = m3d::abs(cos_theta);

            // ── CASE 1: Axis roughly parallel to plane normal ─────────────────
            // The facing cap is nearly flat against the plane.
            // Emit the 4 cardinal rim points of that cap for a stable manifold.
            if (abs_cos >= parallel_threshold)
            {
                // Which cap faces the plane? The one whose centre is deeper.
                // cos_theta < 0 means +Y cap faces toward the plane normal
                // (i.e. +Y cap is the lower one), and vice-versa.
                const m3d::scalar cap_sign = (cos_theta < 0.0f) ? 1.0f : -1.0f;

                // Cap centre in world space
                const m3d::vec3 cap_centre = tf_a.pos + cap_sign * a.half_height * world_axis;

                // Two orthogonal radial directions in the cap plane.
                // Build them from world_axis to stay numerically stable.
                m3d::vec3 radial1 = m3d::cross(world_axis, m3d::vec3(1, 0, 0));
                if (m3d::dot(radial1, radial1) < 1e-6f)
                    radial1 = m3d::cross(world_axis, m3d::vec3(0, 0, 1));
                radial1 = m3d::normalize(radial1);
                const m3d::vec3 radial2 = m3d::normalize(m3d::cross(world_axis, radial1));

                // 4 cardinal points on the cap rim
                const m3d::vec3 rim[4] = {
                    cap_centre + a.base_radius * radial1,
                    cap_centre - a.base_radius * radial1,
                    cap_centre + a.base_radius * radial2,
                    cap_centre - a.base_radius * radial2,
                };

                for (int i = 0; i < 4; ++i)
                {
                    const m3d::scalar dist = m3d::dot(rim[i], world_n) - world_d;
                    if (dist <= epsilon)
                    {
                        out.points[out.num_points].position = rim[i];
                        out.points[out.num_points].penetration_depth = -dist;
                        out.num_points++;
                    }
                }

                // Cap centre itself if it also penetrates (e.g. deep embedding)
                // — skip, 4 rim points are sufficient for the solver.
            }
            // ── CASE 2 & 3: Oblique and perpendicular ─────────────────────────────
            // In both cases: one contact candidate per cap rim, at the radial
            // position deepest into the plane. Emit whichever caps penetrate.
            else
            {
                const m3d::scalar radial_len = m3d::sqrt(
                    support_local.x * support_local.x +
                    support_local.z * support_local.z);

                m3d::vec3 radial_world(0, 0, 0);
                if (radial_len > m3d::EPSILON)
                {
                    const m3d::vec3 radial_local(
                        support_local.x / radial_len * a.base_radius,
                        0.0f,
                        support_local.z / radial_len * a.base_radius);
                    radial_world = tf_a.rotate_vector(radial_local);
                }

                const m3d::vec3 cap_pts[2] = {
                    tf_a.pos + a.half_height * world_axis + radial_world,
                    tf_a.pos - a.half_height * world_axis + radial_world,
                };

                for (int i = 0; i < 2; ++i)
                {
                    const m3d::scalar dist = m3d::dot(cap_pts[i], world_n) - world_d;
                    if (dist <= epsilon)
                    {
                        out.points[out.num_points].position = cap_pts[i];
                        out.points[out.num_points].penetration_depth = -dist;
                        out.num_points++;
                    }
                }
            }

            return out.num_points > 0;
        }
    };

    template <>
    struct CollisionAlgorithm<Plane, Cylinder> : CollisionAlgorithmSym<Plane, Cylinder>
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