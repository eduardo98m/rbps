#pragma once
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/Capsule.hpp"
#include <algorithm>
#include <limits>

namespace rbc
{

    // ── Capsule vs Box ────────────────────────────────────────────────────────
    // SAT approach: Treat the capsule as a line segment with a radius.
    // We test 6 axes: 3 Box face normals and 3 Edge-Segment cross products.
    // Then, we generate a 1-point or 2-point manifold depending on the feature.
    template <>
    struct CollisionAlgorithm<Capsule, Box>
    {
        static bool test(const Capsule &capsule, const m3d::tf &tf_capsule,
                         const Box &box, const m3d::tf &tf_box,
                         ContactManifold &manifold)
        {
            m3d::vec3 p1, p2;
            capsule_endpoints(capsule, tf_capsule, p1, p2);

            const m3d::vec3 d = p2 - p1;
            const m3d::vec3 l = d * 0.5; // Segment half-vector
            const m3d::vec3 c = p1 + l;  // Segment centre

            const m3d::vec3 ax[3] = {
                tf_box.rotate_vector(m3d::vec3(1, 0, 0)),
                tf_box.rotate_vector(m3d::vec3(0, 1, 0)),
                tf_box.rotate_vector(m3d::vec3(0, 0, 1))};
            const m3d::scalar e[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};

            const m3d::vec3 diff = c - tf_box.pos; // Vector from Box centre to Capsule centre

            m3d::scalar best_depth = std::numeric_limits<m3d::scalar>::max();
            m3d::vec3 best_normal = m3d::vec3(1, 0, 0);
            int best_feat = -1; // 0-2: Box faces, 3-5: Edge-Segment crosses

            // 1. Test 3 Box Face Axes
            for (int i = 0; i < 3; ++i)
            {
                const m3d::vec3 &axis = ax[i];
                m3d::scalar r_box = e[i];
                m3d::scalar r_cap = m3d::abs(m3d::dot(l, axis)) + capsule.radius;
                m3d::scalar dist = m3d::abs(m3d::dot(diff, axis));

                m3d::scalar overlap = r_box + r_cap - dist;
                if (overlap < 0.0)
                    return false;

                if (overlap < best_depth)
                {
                    best_depth = overlap;
                    best_normal = (m3d::dot(diff, axis) >= 0.0) ? axis : -axis;
                    best_feat = i;
                }
            }

            // 2. Test 3 Edge-Segment Cross Axes
            for (int i = 0; i < 3; ++i)
            {
                m3d::vec3 axis = m3d::cross(ax[i], l);
                m3d::scalar len_sq = m3d::length_sq(axis);
                if (len_sq < m3d::EPSILON * m3d::EPSILON)
                    continue; // Parallel, handled by face tests

                axis = axis / m3d::sqrt(len_sq);

                // Project box onto axis
                m3d::scalar r_box = e[0] * m3d::abs(m3d::dot(ax[0], axis)) +
                                    e[1] * m3d::abs(m3d::dot(ax[1], axis)) +
                                    e[2] * m3d::abs(m3d::dot(ax[2], axis));

                m3d::scalar r_cap = capsule.radius; // l is perpendicular to the cross axis
                m3d::scalar dist = m3d::abs(m3d::dot(diff, axis));

                m3d::scalar overlap = r_box + r_cap - dist;
                if (overlap < 0.0)
                    return false;

                if (overlap < best_depth)
                {
                    best_depth = overlap;
                    best_normal = (m3d::dot(diff, axis) >= 0.0) ? axis : -axis;
                    best_feat = i + 3;
                }
            }

            // ── We have a collision! Generate the manifold ──────────────────────
            manifold.normal = - best_normal; // Points from Box to Capsule
            manifold.num_points = 0;

            if (best_feat < 3)
            {
                // ── Face Contact (Capsule Segment vs Box Face)
                // Clip the capsule segment against the 4 boundary planes of the box face
                m3d::scalar t0 = 0.0, t1 = 1.0;

                // Helper for Liang-Barsky line clipping
                auto clip_line = [](m3d::scalar p, m3d::scalar q, m3d::scalar &tMin, m3d::scalar &tMax) -> bool
                {
                    if (m3d::abs(p) < m3d::EPSILON)
                        return q >= 0.0;
                    m3d::scalar t = q / p;
                    if (p > 0.0)
                    {
                        if (t < tMax)
                            tMax = t;
                    }
                    else
                    {
                        if (t > tMin)
                            tMin = t;
                    }
                    return tMin <= tMax;
                };

                int iA = (best_feat + 1) % 3;
                int iB = (best_feat + 2) % 3;

                m3d::vec3 p1_rel = p1 - tf_box.pos;

                // Clip against Axis A boundaries
                m3d::scalar pA = m3d::dot(d, ax[iA]), valA = m3d::dot(p1_rel, ax[iA]);
                if (!clip_line(pA, e[iA] - valA, t0, t1))
                    return true;
                if (!clip_line(-pA, e[iA] + valA, t0, t1))
                    return true;

                // Clip against Axis B boundaries
                m3d::scalar pB = m3d::dot(d, ax[iB]), valB = m3d::dot(p1_rel, ax[iB]);
                if (!clip_line(pB, e[iB] - valB, t0, t1))
                    return true;
                if (!clip_line(-pB, e[iB] + valB, t0, t1))
                    return true;

                // Evaluate depths of clipped segment endpoints
                m3d::vec3 pt0 = p1 + d * t0;
                m3d::scalar depth0 = capsule.radius + e[best_feat] - m3d::dot(pt0 - tf_box.pos, best_normal);
                if (depth0 > 0.0)
                {
                    manifold.points[manifold.num_points].position = pt0 - best_normal * capsule.radius;
                    manifold.points[manifold.num_points].penetration_depth = depth0;
                    manifold.num_points++;
                }

                // If the segment is long enough inside the box, add the second point
                if (t1 - t0 > m3d::EPSILON)
                {
                    m3d::vec3 pt1 = p1 + d * t1;
                    m3d::scalar depth1 = capsule.radius + e[best_feat] - m3d::dot(pt1 - tf_box.pos, best_normal);
                    if (depth1 > 0.0)
                    {
                        manifold.points[manifold.num_points].position = pt1 - best_normal * capsule.radius;
                        manifold.points[manifold.num_points].penetration_depth = depth1;
                        manifold.num_points++;
                    }
                }
            }
            else
            {
                // ── Edge Contact (Capsule Segment vs Box Edge)
                // Single point contact. Find the closest point on the segment to the box edge.
                int edge_axis = best_feat - 3;
                m3d::vec3 v = ax[edge_axis]; // Direction of the box edge

                // Find a point lying on the exact edge we are hitting
                m3d::vec3 support_box = tf_box.pos +
                                        ax[0] * ((m3d::dot(ax[0], best_normal) >= 0.0) ? e[0] : -e[0]) +
                                        ax[1] * ((m3d::dot(ax[1], best_normal) >= 0.0) ? e[1] : -e[1]) +
                                        ax[2] * ((m3d::dot(ax[2], best_normal) >= 0.0) ? e[2] : -e[2]);

                // Closest point calculation from segment `p1 + t*d` to line `support_box + s*v`
                m3d::vec3 r = p1 - support_box;
                m3d::scalar a = m3d::length_sq(d);
                m3d::scalar b = m3d::dot(d, v);
                m3d::scalar f = m3d::dot(v, r);
                m3d::scalar c_dot = m3d::dot(d, r);

                m3d::scalar denom = a - b * b;
                m3d::scalar t = (denom > m3d::EPSILON) ? m3d::clamp((b * f - c_dot) / denom, m3d::scalar(0), m3d::scalar(1)) : 0.5;

                m3d::vec3 pt = p1 + d * t;

                manifold.num_points = 1;
                manifold.points[0].position = pt - best_normal * capsule.radius;
                manifold.points[0].penetration_depth = best_depth;
            }

            // Failsafe if clipping accidentally discarded valid points
            if (manifold.num_points == 0)
            {
                manifold.num_points = 1;
                manifold.points[0].position = c - best_normal * capsule.radius;
                manifold.points[0].penetration_depth = best_depth;
            }

            return true;
        }
    };

    // Box vs Capsule — symmetric shim
    template <>
    struct CollisionAlgorithm<Box, Capsule> : CollisionAlgorithmSym<Box, Capsule>
    {
    };

} // namespace rbc