#pragma once
#include "rbc/Dispatcher.hpp"
#include <algorithm> // std::min, std::abs
#include <limits>    // std::numeric_limits

namespace rbc
{

    // ── Box vs Box — Separating Axis Theorem (SAT) ───────────────────────────────
    //
    // For two Oriented Bounding Boxes (OBBs), SAT requires testing 15 axes:
    //   • 3 face normals of box A  (A's local X, Y, Z axes in world space)
    //   • 3 face normals of box B  (B's local X, Y, Z axes in world space)
    //   • 9 edge cross products    (A_i × B_j for i,j in {X,Y,Z})
    //
    // For each axis:
    //   1. Project both boxes onto it (yields an interval [−r, +r] for each)
    //   2. Compute overlap = rA + rB − |dot(centre_diff, axis)|
    //   3. If overlap < 0 → separating axis found → no collision
    //   4. Track the axis with the MINIMUM overlap → that becomes the contact normal
    //
    // Time complexity: O(1) — exactly 15 axis tests, all branchless arithmetic.

    namespace detail
    {

        // Returns the "support radius" of an OBB projected onto a world-space axis.
        // This is the half-width of the OBB's shadow on that axis.
        inline m3d::scalar obb_radius(const Box &box, const m3d::tf &tf,
                                      const m3d::vec3 &axis)
        {
            // Columns of the rotation matrix = local axes in world space
            const m3d::vec3 ax = tf.rotate_vector(m3d::vec3(1, 0, 0));
            const m3d::vec3 ay = tf.rotate_vector(m3d::vec3(0, 1, 0));
            const m3d::vec3 az = tf.rotate_vector(m3d::vec3(0, 0, 1));

            return m3d::abs(m3d::dot(ax, axis)) * box.half_extents.x + m3d::abs(m3d::dot(ay, axis)) * box.half_extents.y + m3d::abs(m3d::dot(az, axis)) * box.half_extents.z;
        }

        // Returns the world-space support point of an OBB in a given direction.
        inline m3d::vec3 obb_support(const Box &box, const m3d::tf &tf,
                                     const m3d::vec3 &dir)
        {
            const m3d::vec3 ax = tf.rotate_vector(m3d::vec3(1, 0, 0));
            const m3d::vec3 ay = tf.rotate_vector(m3d::vec3(0, 1, 0));
            const m3d::vec3 az = tf.rotate_vector(m3d::vec3(0, 0, 1));

            return tf.pos + (m3d::dot(ax, dir) >= 0 ? ax : -ax) * box.half_extents.x + (m3d::dot(ay, dir) >= 0 ? ay : -ay) * box.half_extents.y + (m3d::dot(az, dir) >= 0 ? az : -az) * box.half_extents.z;
        }

        // Tests one axis. Returns the signed overlap (positive = penetrating).
        // Updates best_depth / best_normal if this axis has the smallest penetration.
        // Returns false immediately if the axis is a separating one.
        inline bool test_axis(const m3d::vec3 &axis,
                              const Box &a, const m3d::tf &tf_a,
                              const Box &b, const m3d::tf &tf_b,
                              const m3d::vec3 &centre_diff,
                              m3d::scalar &best_depth,
                              m3d::vec3 &best_normal)
        {
            const m3d::scalar len_sq = m3d::length_sq(axis);

            // Skip degenerate axes (can happen with parallel edges in cross products)
            if (len_sq < m3d::EPSILON * m3d::EPSILON)
                return true; // not a useful axis — don't reject, just skip

            const m3d::vec3 norm_axis = axis / m3d::sqrt(len_sq);

            const m3d::scalar rA = obb_radius(a, tf_a, norm_axis);
            const m3d::scalar rB = obb_radius(b, tf_b, norm_axis);
            const m3d::scalar dist = m3d::abs(m3d::dot(centre_diff, norm_axis));
            const m3d::scalar overlap = rA + rB - dist;

            if (overlap < 0.0)
                return false; // separating axis found

            if (overlap < best_depth)
            {
                best_depth = overlap;
                // Normal points from A to B (i.e. from A's perspective, push direction)
                best_normal = m3d::dot(centre_diff, norm_axis) >= 0.0
                                  ? norm_axis
                                  : -norm_axis;
            }

            return true;
        }

    } // namespace detail

    template <>
    struct CollisionAlgorithm<Box, Box>
    {
        static bool test(const Box &a, const m3d::tf &tf_a,
                         const Box &b, const m3d::tf &tf_b,
                         Contact &out)
        {
            // ── Setup ─────────────────────────────────────────────────────────────
            const m3d::vec3 centre_diff = tf_b.pos - tf_a.pos;

            // Local axes of A and B in world space
            const m3d::vec3 ax[3] = {
                tf_a.rotate_vector(m3d::vec3(1, 0, 0)),
                tf_a.rotate_vector(m3d::vec3(0, 1, 0)),
                tf_a.rotate_vector(m3d::vec3(0, 0, 1))};
            const m3d::vec3 bx[3] = {
                tf_b.rotate_vector(m3d::vec3(1, 0, 0)),
                tf_b.rotate_vector(m3d::vec3(0, 1, 0)),
                tf_b.rotate_vector(m3d::vec3(0, 0, 1))};

            m3d::scalar best_depth = std::numeric_limits<m3d::scalar>::max();
            m3d::vec3 best_normal = m3d::vec3(1, 0, 0);

            // ── 3 face axes of A ─────────────────────────────────────────────────
            for (int i = 0; i < 3; ++i)
                if (!detail::test_axis(ax[i], a, tf_a, b, tf_b,
                                       centre_diff, best_depth, best_normal))
                    return false;

            // ── 3 face axes of B ─────────────────────────────────────────────────
            for (int i = 0; i < 3; ++i)
                if (!detail::test_axis(bx[i], a, tf_a, b, tf_b,
                                       centre_diff, best_depth, best_normal))
                    return false;

            // ── 9 edge cross-product axes (A_i × B_j) ────────────────────────────
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    if (!detail::test_axis(m3d::cross(ax[i], bx[j]),
                                           a, tf_a, b, tf_b,
                                           centre_diff, best_depth, best_normal))
                        return false;

            // ── All 15 axes passed — boxes are overlapping ───────────────────────
            out.normal = best_normal;
            out.penetration_depth = best_depth;

            // Contact point: support point of A in the direction of the normal,
            // then pulled back half the penetration depth so it sits at the
            // contact surface between the two boxes.
            out.pos = tf_a.pos + detail::obb_support(a, tf_a, best_normal) - best_normal * (best_depth * 0.5);

            return true;
        }
    };

} // namespace rbc