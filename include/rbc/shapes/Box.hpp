#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

/**
 * @file Box.hpp
 * @brief Oriented box (OBB) collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Oriented box, defined by its half-extents along the body's local axes.
     *
     * Box is the only shape whose `face_corners` returns a true face polygon
     * (the 4 corners of the most-aligned face) instead of falling back to
     * the disc approximation. This is what enables the high-quality face/face
     * contact manifold for box–box pairs.
     *
     * @ingroup rbc
     */
    struct Box
    {
        m3d::vec3 half_extents; ///< Half-sizes along local X, Y, Z.

        /** @brief Default box of half-extents (0.5, 0.5, 0.5) (a unit cube). */
        Box() : half_extents(0.5, 0.5, 0.5) {}
        /** @brief Construct with explicit half-extents. */
        Box(const m3d::vec3 &half_extents) : half_extents(half_extents) {}

        /** @brief Equality on half-extents. */
        inline bool operator==(const Box &other) const
        {
            return half_extents == other.half_extents;
        }

        /** @brief Inequality. */
        inline bool operator!=(const Box &other) const
        {
            return !(*this == other);
        }
    };

    /**
     * @brief Local-space support: corner whose components match the sign of `dir`.
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Box &box, const m3d::vec3 &dir)
    {
        return m3d::vec3(
            (dir.x > 0) ? box.half_extents.x : -box.half_extents.x,
            (dir.y > 0) ? box.half_extents.y : -box.half_extents.y,
            (dir.z > 0) ? box.half_extents.z : -box.half_extents.z);
    }

    /** @brief Box volume `8·hx·hy·hz`. @ingroup rbc */
    inline m3d::scalar compute_volume(const Box &box)
    {
        return 8.0 * box.half_extents.x * box.half_extents.y * box.half_extents.z;
    }

    /**
     * @brief Inertia tensor of a uniform-density box about its centre.
     *
     * Diagonal entries follow the standard `(1/12)·m·(side_a² + side_b²)`
     * formula; off-diagonals are zero. Mass is taken as the volume
     * (unit density).
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const Box &box)
    {
        m3d::scalar x2 = 4.0 * box.half_extents.x * box.half_extents.x;
        m3d::scalar y2 = 4.0 * box.half_extents.y * box.half_extents.y;
        m3d::scalar z2 = 4.0 * box.half_extents.z * box.half_extents.z;
        m3d::scalar mass = compute_volume(box);
        return m3d::smat3(
            (1.0 / 12.0) * mass * (y2 + z2), // Ixx
            (1.0 / 12.0) * mass * (x2 + z2), // Iyy
            (1.0 / 12.0) * mass * (x2 + y2), // Izz
            0,                               // Ixy
            0,                               // Ixz
            0                                // Iyz
        );
    }

    /**
     * @brief Tight world AABB of an oriented box.
     *
     * Computed by projecting the OBB onto each world axis: the world-space
     * half-extent along axis `i` is `Σ_j |R_ij| · h_j`. Standard derivation
     * for a rotated box; cheaper than evaluating all 8 corners.
     *
     * @note Assumes `m3d::mat3_cast(quat)` returns a column-major mat3 where
     *       `mat[col][row]` indexes the element. Adjust the indexing if your
     *       math3d uses a different convention.
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Box &b, const m3d::tf &tf)
    {
        const m3d::mat3 R = m3d::mat3_cast(tf.rot);
        const m3d::vec3 &h = b.half_extents;

        const m3d::vec3 extent(
            m3d::abs(R[0][0]) * h.x + m3d::abs(R[1][0]) * h.y + m3d::abs(R[2][0]) * h.z,
            m3d::abs(R[0][1]) * h.x + m3d::abs(R[1][1]) * h.y + m3d::abs(R[2][1]) * h.z,
            m3d::abs(R[0][2]) * h.x + m3d::abs(R[1][2]) * h.y + m3d::abs(R[2][2]) * h.z);

        return {tf.pos - extent, tf.pos + extent};
    }

    /** @brief Tag-dispatched marker: Box is a convex bounded shape (true). @ingroup rbc */
    constexpr bool is_gjk_convex(const Box *) { return true; }

    /** @brief Representative size = the largest half-extent. @ingroup rbc */
    inline m3d::scalar representative_radius(const Box &b)
    {
        return m3d::max(m3d::max(b.half_extents.x, b.half_extents.y), b.half_extents.z);
    }

    /**
     * @brief 4 corners of the box face whose outward normal is most aligned with `dir`.
     *
     * This is the only shape that returns an exact face polygon (rather than
     * the disc approximation in `FaceHelpers`). The contact manifold
     * generator uses it as the reference / incident polygon for box pairs,
     * which is what gives box–box contacts their stable 4-point manifold.
     *
     * @return Always 4.
     *
     * @ingroup rbc
     */
    inline int face_corners(const Box &b, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 *corners, int capacity)
    {
        if (capacity < 4) return 0;
        const m3d::vec3 axes[3] = {
            tf.rotate_vector(m3d::vec3(1, 0, 0)),
            tf.rotate_vector(m3d::vec3(0, 1, 0)),
            tf.rotate_vector(m3d::vec3(0, 0, 1)),
        };
        const m3d::scalar half[3] = {b.half_extents.x, b.half_extents.y, b.half_extents.z};

        int best = 0;
        m3d::scalar best_d = m3d::abs(m3d::dot(axes[0], dir));
        for (int i = 1; i < 3; ++i)
        {
            const m3d::scalar d = m3d::abs(m3d::dot(axes[i], dir));
            if (d > best_d)
            {
                best_d = d;
                best = i;
            }
        }

        const m3d::scalar sign = (m3d::dot(axes[best], dir) >= 0.0) ? 1.0 : -1.0;
        const m3d::vec3 centre = tf.pos + axes[best] * (sign * half[best]);
        const int u = (best + 1) % 3, v = (best + 2) % 3;

        corners[0] = centre + axes[u] * half[u] + axes[v] * half[v];
        corners[1] = centre - axes[u] * half[u] + axes[v] * half[v];
        corners[2] = centre - axes[u] * half[u] - axes[v] * half[v];
        corners[3] = centre + axes[u] * half[u] - axes[v] * half[v];
        return 4;
    }
}
