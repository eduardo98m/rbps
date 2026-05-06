#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

/**
 * @file Ellipsoid.hpp
 * @brief Ellipsoid collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Axis-aligned ellipsoid in local space.
     *
     * Local-space surface: `(x/a)² + (y/b)² + (z/c)² = 1` with semi-axes
     * `(a, b, c)` along local X, Y, Z. Body rotation is handled by `m3d::tf`.
     *
     * @ingroup rbc
     */
    struct Ellipsoid
    {
        m3d::vec3 half_extents; ///< Semi-axes (a, b, c) along local X, Y, Z.

        /** @brief Default ellipsoid: semi-axes (0.5, 0.5, 0.5) (unit sphere). */
        Ellipsoid() : half_extents(0.5, 0.5, 0.5) {}
        /** @brief Construct with explicit semi-axes. */
        explicit Ellipsoid(const m3d::vec3 &half_extents) : half_extents(half_extents) {}

        /** @brief Equality on semi-axes. */
        inline bool operator==(const Ellipsoid &o) const { return half_extents == o.half_extents; }
        /** @brief Inequality. */
        inline bool operator!=(const Ellipsoid &o) const { return !(*this == o); }
    };

    /**
     * @brief Local-space support via Lagrange multipliers.
     *
     * For the constraint `(x/a)² + (y/b)² + (z/c)² ≤ 1`, the support point
     * along `dir` is:
     * `p_k = a_k² · d_k / sqrt(Σ a_k² · d_k²)`.
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Ellipsoid &e, const m3d::vec3 &dir)
    {
        const m3d::vec3 &a = e.half_extents;
        const m3d::vec3 scaled(a.x * a.x * dir.x,
                               a.y * a.y * dir.y,
                               a.z * a.z * dir.z);
        const m3d::scalar denom = m3d::sqrt(a.x * a.x * dir.x * dir.x +
                                            a.y * a.y * dir.y * dir.y +
                                            a.z * a.z * dir.z * dir.z);
        if (denom < m3d::EPSILON)
            return m3d::vec3(a.x, 0.0, 0.0);
        return scaled / denom;
    }

    /** @brief Volume `(4/3)·π·a·b·c`. @ingroup rbc */
    inline m3d::scalar compute_volume(const Ellipsoid &e)
    {
        return (4.0 / 3.0) * m3d::PI * e.half_extents.x * e.half_extents.y * e.half_extents.z;
    }

    /**
     * @brief Inertia tensor of a uniform-density ellipsoid about its centre.
     *
     * Diagonal entries `Iii = (2/5)·m·(j² + k²)` cycled over the three axes.
     * Mass is taken as the volume (unit density).
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const Ellipsoid &e)
    {
        const m3d::scalar mass = compute_volume(e);
        const m3d::scalar ax2 = e.half_extents.x * e.half_extents.x;
        const m3d::scalar ay2 = e.half_extents.y * e.half_extents.y;
        const m3d::scalar az2 = e.half_extents.z * e.half_extents.z;
        return m3d::smat3(
            (2.0 / 5.0) * mass * (ay2 + az2),
            (2.0 / 5.0) * mass * (ax2 + az2),
            (2.0 / 5.0) * mass * (ax2 + ay2),
            0.0, 0.0, 0.0);
    }

    /**
     * @brief Tight world AABB.
     *
     * World-space half-extent along axis `i` is
     * `sqrt(R[0][i]²·a² + R[1][i]²·b² + R[2][i]²·c²)`. Same derivation
     * as `Box::compute_aabb` but with sum-of-squares instead of sum-of-`|·|`.
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Ellipsoid &e, const m3d::tf &tf)
    {
        const m3d::mat3 R = m3d::mat3_cast(tf.rot);
        const m3d::vec3 &a = e.half_extents;
        const m3d::vec3 extent(
            m3d::sqrt(R[0][0] * R[0][0] * a.x * a.x + R[1][0] * R[1][0] * a.y * a.y + R[2][0] * R[2][0] * a.z * a.z),
            m3d::sqrt(R[0][1] * R[0][1] * a.x * a.x + R[1][1] * R[1][1] * a.y * a.y + R[2][1] * R[2][1] * a.z * a.z),
            m3d::sqrt(R[0][2] * R[0][2] * a.x * a.x + R[1][2] * R[1][2] * a.y * a.y + R[2][2] * R[2][2] * a.z * a.z));
        return {tf.pos - extent, tf.pos + extent};
    }

    /** @brief Tag-dispatched marker: Ellipsoid is a convex bounded shape (true). @ingroup rbc */
    constexpr bool is_gjk_convex(const Ellipsoid *) { return true; }

    /** @brief Representative size = the largest semi-axis. @ingroup rbc */
    inline m3d::scalar representative_radius(const Ellipsoid &e)
    {
        return m3d::max(m3d::max(e.half_extents.x, e.half_extents.y), e.half_extents.z);
    }

    /** @brief Disc-approximation face polygon (ellipsoid has no flat face). @ingroup rbc */
    inline int face_corners(const Ellipsoid &e, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 *out, int capacity)
    {
        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        const m3d::vec3 sup_world = tf.transform_point(support(e, local_dir));
        return get_generic_face_corners(sup_world, dir, representative_radius(e), out, capacity);
    }
} // namespace rbc
