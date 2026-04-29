#pragma once
#include <math3d/math3d.hpp>
#include "rbc/Contact.hpp"

/**
 * @file TriangleUtils.hpp
 * @brief Triangle/segment geometry helpers shared by mesh + heightmap collision.
 * @ingroup rbc
 * @ingroup internals
 */

namespace rbc::tri
{
    /**
     * @brief Closest point on triangle `ABC` to query point `P`.
     *
     * Barycentric / Voronoi-region method. Reference:
     * Ericson, *Real-Time Collision Detection*, §5.1.5.
     *
     * @ingroup internals
     */
    inline m3d::vec3 closest_point_on_triangle(const m3d::vec3 &P,
                                               const m3d::vec3 &A,
                                               const m3d::vec3 &B,
                                               const m3d::vec3 &C)
    {
        const m3d::vec3 AB = B - A, AC = C - A, AP = P - A;
        const m3d::scalar d1 = m3d::dot(AB, AP), d2 = m3d::dot(AC, AP);
        if (d1 <= 0.0 && d2 <= 0.0)
            return A; // vertex A

        const m3d::vec3 BP = P - B;
        const m3d::scalar d3 = m3d::dot(AB, BP), d4 = m3d::dot(AC, BP);
        if (d3 >= 0.0 && d4 <= d3)
            return B; // vertex B

        const m3d::scalar vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
            return A + AB * (d1 / (d1 - d3)); // edge AB

        const m3d::vec3 CP = P - C;
        const m3d::scalar d5 = m3d::dot(AB, CP), d6 = m3d::dot(AC, CP);
        if (d6 >= 0.0 && d5 <= d6)
            return C; // vertex C

        const m3d::scalar vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
            return A + AC * (d2 / (d2 - d6)); // edge AC

        const m3d::scalar va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
        {
            const m3d::scalar w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return B + (C - B) * w; // edge BC
        }

        // Inside face
        const m3d::scalar denom = 1.0 / (va + vb + vc);
        const m3d::scalar v = vb * denom, w = vc * denom;
        return A + AB * v + AC * w;
    }

    /**
     * @brief Squared distance from point `P` to triangle `ABC`.
     * @ingroup internals
     */
    inline m3d::scalar sq_dist_point_triangle(const m3d::vec3 &P,
                                              const m3d::vec3 &A,
                                              const m3d::vec3 &B,
                                              const m3d::vec3 &C)
    {
        const m3d::vec3 Q = closest_point_on_triangle(P, A, B, C);
        return m3d::length_sq(P - Q);
    }

    /**
     * @brief Closest point on segment `[P, Q]` to query point `X`.
     * @ingroup internals
     */
    inline m3d::vec3 closest_point_on_segment(const m3d::vec3 &P,
                                              const m3d::vec3 &Q,
                                              const m3d::vec3 &X)
    {
        const m3d::vec3 d = Q - P;
        const m3d::scalar len2 = m3d::length_sq(d);
        if (len2 < m3d::EPSILON)
            return P;
        const m3d::scalar t = m3d::clamp(m3d::dot(X - P, d) / len2, m3d::scalar(0), m3d::scalar(1));
        return P + d * t;
    }

    /**
     * @brief Squared distance from segment `[P, Q]` to triangle `ABC`.
     *
     * Tests every segment endpoint against the triangle plus every triangle
     * edge against the segment, returning the minimum.
     *
     * @ingroup internals
     */
    inline m3d::scalar sq_dist_segment_triangle(const m3d::vec3 &P, const m3d::vec3 &Q,
                                                const m3d::vec3 &A, const m3d::vec3 &B,
                                                const m3d::vec3 &C)
    {
        m3d::scalar best = sq_dist_point_triangle(P, A, B, C);
        best = m3d::min(best, sq_dist_point_triangle(Q, A, B, C));

        // Triangle edge AB vs segment
        auto seg_seg_sq = [](const m3d::vec3 &p1, const m3d::vec3 &p2,
                             const m3d::vec3 &q1, const m3d::vec3 &q2) -> m3d::scalar
        {
            const m3d::vec3 d1 = p2 - p1, d2 = q2 - q1, r = p1 - q1;
            const m3d::scalar a = m3d::length_sq(d1);
            const m3d::scalar e = m3d::length_sq(d2);
            const m3d::scalar f = m3d::dot(d2, r);
            m3d::scalar s, t;
            if (a < m3d::EPSILON && e < m3d::EPSILON)
            {
                s = t = 0.0;
            }
            else if (a < m3d::EPSILON)
            {
                s = 0.0;
                t = m3d::clamp(f / e, m3d::scalar(0), m3d::scalar(1));
            }
            else
            {
                const m3d::scalar c = m3d::dot(d1, r);
                if (e < m3d::EPSILON)
                {
                    t = 0.0;
                    s = m3d::clamp(-c / a, m3d::scalar(0), m3d::scalar(1));
                }
                else
                {
                    const m3d::scalar b = m3d::dot(d1, d2);
                    const m3d::scalar denom = a * e - b * b;
                    if (denom > m3d::EPSILON)
                        s = m3d::clamp((b * f - c * e) / denom, m3d::scalar(0), m3d::scalar(1));
                    else
                        s = 0.0;
                    t = (b * s + f) / e;
                    if (t < 0.0)
                    {
                        t = 0.0;
                        s = m3d::clamp(-c / a, m3d::scalar(0), m3d::scalar(1));
                    }
                    else if (t > 1.0)
                    {
                        t = 1.0;
                        s = m3d::clamp((b - c) / a, m3d::scalar(0), m3d::scalar(1));
                    }
                }
            }
            return m3d::length_sq((p1 + d1 * s) - (q1 + d2 * t));
        };

        best = m3d::min(best, seg_seg_sq(P, Q, A, B));
        best = m3d::min(best, seg_seg_sq(P, Q, B, C));
        best = m3d::min(best, seg_seg_sq(P, Q, C, A));
        return best;
    }

    /**
     * @brief Test a sphere against a single triangle.
     *
     * Returns `true` and fills `manifold` (1 contact point) if the sphere
     * penetrates the triangle. The contact normal points from the
     * triangle face toward the sphere.
     *
     * @param face_normal Precomputed unit triangle normal (avoids re-deriving).
     *
     * @ingroup internals
     */
    inline bool sphere_vs_triangle(const m3d::vec3 &centre, m3d::scalar radius,
                                   const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C,
                                   const m3d::vec3 &face_normal, // precomputed, unit
                                   ContactManifold &manifold)
    {
        const m3d::vec3 closest = closest_point_on_triangle(centre, A, B, C);
        const m3d::vec3 delta = centre - closest;
        const m3d::scalar dist2 = m3d::length_sq(delta);
        if (dist2 > radius * radius)
            return false;

        const m3d::scalar dist = m3d::sqrt(dist2);
        if (dist > m3d::EPSILON)
        {
            manifold.normal = delta / dist;
            manifold.points[0].penetration_depth = radius - dist;
            manifold.points[0].position = closest;
            manifold.num_points = 1;
        }
        else
        {
            // Centre is on or inside the face — use face normal
            const m3d::scalar side = m3d::dot(face_normal, centre - A);
            manifold.normal = (side >= 0.0) ? face_normal : -face_normal;
            manifold.points[0].penetration_depth = radius + m3d::abs(side);
            manifold.points[0].position = centre - manifold.normal * radius;
            manifold.num_points = 1;
        }
        return true;
    }

    /**
     * @brief Test a capsule against a single triangle.
     *
     * Returns `true` and fills `manifold` (1 contact point) if the capsule
     * penetrates the triangle.
     *
     * @ingroup internals
     */
    inline bool capsule_vs_triangle(const m3d::vec3 &cap_p1, const m3d::vec3 &cap_p2,
                                    m3d::scalar radius,
                                    const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C,
                                    const m3d::vec3 &face_normal,
                                    ContactManifold &manifold)
    {
        // Project segment onto triangle plane to find the "closest" segment point
        // Strategy: find closest dist between segment and triangle, handle as sphere from that point
        const m3d::scalar d1 = m3d::dot(face_normal, cap_p1 - A);
        const m3d::scalar d2 = m3d::dot(face_normal, cap_p2 - A);

        // If both endpoints on same side and beyond radius, early-out (rough check)
        if (d1 > radius && d2 > radius)
            return false;
        if (d1 < -radius && d2 < -radius)
            return false;

        const m3d::scalar sq_dist = sq_dist_segment_triangle(cap_p1, cap_p2, A, B, C);
        if (sq_dist > radius * radius)
            return false;

        const m3d::scalar dist = m3d::sqrt(sq_dist);

        // Find the actual closest points to determine contact position and normal
        // Use closest endpoint to triangle as reference
        const m3d::vec3 cp1 = closest_point_on_triangle(cap_p1, A, B, C);
        const m3d::vec3 cp2 = closest_point_on_triangle(cap_p2, A, B, C);
        const m3d::scalar d1sq = m3d::length_sq(cap_p1 - cp1);
        const m3d::scalar d2sq = m3d::length_sq(cap_p2 - cp2);
        const m3d::vec3 &ref_seg = (d1sq <= d2sq) ? cap_p1 : cap_p2;
        const m3d::vec3 &ref_tri = (d1sq <= d2sq) ? cp1 : cp2;
        const m3d::vec3 delta = ref_seg - ref_tri;

        if (dist > m3d::EPSILON)
        {
            manifold.normal = delta / dist;
            manifold.points[0].penetration_depth = radius - dist;
            manifold.points[0].position = ref_tri;
            manifold.num_points = 1;
        }
        else
        {
            // Segment lies on face
            manifold.normal = face_normal;
            manifold.points[0].penetration_depth = radius;
            manifold.points[0].position = ref_tri;
            manifold.num_points = 1;
        }
        return true;
    }

} // namespace rbc::tri