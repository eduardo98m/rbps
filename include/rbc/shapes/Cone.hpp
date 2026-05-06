#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

/**
 * @file Cone.hpp
 * @brief Right circular cone collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Right circular cone aligned with local +Y.
     *
     * - Apex = `(0, +half_height, 0)`.
     * - Base centre = `(0, -half_height, 0)`, radius = `base_radius`.
     * - Total height = `2 * half_height`.
     *
     * @ingroup rbc
     */
    struct Cone
    {
        m3d::scalar half_height; ///< Distance from centre to apex (and to base centre).
        m3d::scalar base_radius; ///< Radius of the base circle.

        /** @brief Default cone of half-height 0.5 and base radius 0.5. */
        Cone() : half_height(0.5), base_radius(0.5) {}
        /** @brief Construct with explicit dimensions. */
        Cone(m3d::scalar half_height, m3d::scalar base_radius)
            : half_height(half_height), base_radius(base_radius) {}

        /** @brief Equality on both fields. */
        inline bool operator==(const Cone &o) const
        {
            return half_height == o.half_height && base_radius == o.base_radius;
        }
        /** @brief Inequality. */
        inline bool operator!=(const Cone &o) const { return !(*this == o); }
    };

    /**
     * @brief Local-space support — pick the apex or a point on the base circle.
     *
     * Two candidates compete: the apex `(0, h, 0)` and the base-edge point
     * in the radial direction of `dir`. The winner is whichever has the
     * larger inner product with `dir`. Closed-form, no iteration.
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Cone &c, const m3d::vec3 &dir)
    {
        const m3d::scalar d_radial = m3d::sqrt(dir.x * dir.x + dir.z * dir.z);

        if (d_radial < m3d::EPSILON)
        {
            // Direction is purely axial.
            return (dir.y >= 0.0)
                       ? m3d::vec3(0.0, c.half_height, 0.0)
                       : m3d::vec3(c.base_radius, -c.half_height, 0.0);
        }

        const m3d::scalar apex_dot = dir.y * c.half_height;
        const m3d::scalar base_edge_dot = d_radial * c.base_radius - dir.y * c.half_height;

        if (apex_dot >= base_edge_dot)
            return m3d::vec3(0.0, c.half_height, 0.0);

        const m3d::scalar scale = c.base_radius / d_radial;
        return m3d::vec3(dir.x * scale, -c.half_height, dir.z * scale);
    }

    /** @brief Volume `(1/3)·π·r²·h_total`. @ingroup rbc */
    inline m3d::scalar compute_volume(const Cone &c)
    {
        return (1.0 / 3.0) * m3d::PI * c.base_radius * c.base_radius * (2.0 * c.half_height);
    }

    /**
     * @brief Inertia tensor of a uniform-density cone about its centre.
     *
     * The transverse moment is computed about the apex and shifted to the
     * centroid (which sits at `3H/8` from the base, i.e. `h/4` from the
     * local origin) using the parallel-axis theorem.
     *
     * - `Iyy (axial)     = (3/10) · m · R²`
     * - `Ixx (transverse)` derived as above.
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const Cone &c)
    {
        const m3d::scalar mass = compute_volume(c);
        const m3d::scalar R2 = c.base_radius * c.base_radius;
        const m3d::scalar H = 2.0 * c.half_height;
        const m3d::scalar Iyy = (3.0 / 10.0) * mass * R2;
        const m3d::scalar Ixx_apex = (3.0 / 5.0) * mass * (R2 / 4.0 + H * H);
        const m3d::scalar d_centroid = 3.0 * H / 8.0 - c.half_height;
        const m3d::scalar Ixx = Ixx_apex - mass * d_centroid * d_centroid;
        return m3d::smat3(Ixx, Iyy, Ixx, 0.0, 0.0, 0.0);
    }

    /**
     * @brief Tight world AABB via 6-direction support sweep.
     *
     * More accurate than a bounding sphere, still O(1).
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Cone &c, const m3d::tf &tf)
    {
        const m3d::vec3 world_axes[6] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

        m3d::vec3 mn = tf.pos, mx = tf.pos;
        for (const auto &axis : world_axes)
        {
            const m3d::vec3 local_dir = tf.inverse_rotate_vector(axis);
            const m3d::vec3 world_sup = tf.pos + tf.rotate_vector(support(c, local_dir));
            mn = m3d::vec3(m3d::min(mn.x, world_sup.x), m3d::min(mn.y, world_sup.y), m3d::min(mn.z, world_sup.z));
            mx = m3d::vec3(m3d::max(mx.x, world_sup.x), m3d::max(mx.y, world_sup.y), m3d::max(mx.z, world_sup.z));
        }
        return {mn, mx};
    }

    /** @brief Tag-dispatched marker: Cone is a convex bounded shape (true). @ingroup rbc */
    constexpr bool is_gjk_convex(const Cone *) { return true; }

    /** @brief Representative size = max(base_radius, half_height). @ingroup rbc */
    inline m3d::scalar representative_radius(const Cone &c)
    {
        return m3d::max(c.base_radius, c.half_height);
    }

    /** @brief Disc-approximation face polygon (cone has no flat side face). @ingroup rbc */
    inline int face_corners(const Cone &c, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 *out, int capacity)
    {
        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        const m3d::vec3 sup_world = tf.transform_point(support(c, local_dir));
        return get_generic_face_corners(sup_world, dir, representative_radius(c), out, capacity);
    }
} // namespace rbc
