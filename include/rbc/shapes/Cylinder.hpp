#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

/**
 * @file Cylinder.hpp
 * @brief Cylinder collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Cylinder aligned with local +Y.
     *
     * - Top cap centre  = `(0, +half_height, 0)`, radius = `base_radius`.
     * - Bottom cap centre = `(0, -half_height, 0)`, radius = `base_radius`.
     * - Total height = `2 * half_height`.
     *
     * @ingroup rbc
     */
    struct Cylinder
    {
        m3d::scalar half_height; ///< Distance from centre to each cap.
        m3d::scalar base_radius; ///< Radius of both circular caps.

        /** @brief Default cylinder of half-height 0.5 and base radius 0.5. */
        Cylinder() : half_height(0.5), base_radius(0.5) {}
        /** @brief Construct with explicit dimensions. */
        Cylinder(m3d::scalar half_height, m3d::scalar base_radius)
            : half_height(half_height), base_radius(base_radius) {}

        /** @brief Equality on both fields. */
        inline bool operator==(const Cylinder &o) const
        {
            return half_height == o.half_height && base_radius == o.base_radius;
        }
        /** @brief Inequality. */
        inline bool operator!=(const Cylinder &o) const { return !(*this == o); }
    };

    /**
     * @brief Local-space support function for a cylinder.
     *
     * A cylinder's support decomposes cleanly into two independent parts:
     *
     * - **Y axis**: the support is always `±half_height` depending on `sign(dir.y)`.
     * - **XZ plane**: the support is the point on the rim in the direction of
     *   the XZ projection of `dir`, scaled to `base_radius`.
     *
     * When the XZ projection of `dir` is degenerate (purely axial direction),
     * any point on the rim is equally valid — we pick `(base_radius, ±h, 0)`.
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Cylinder &c, const m3d::vec3 &dir)
    {
        // Y component: whichever cap is in the direction of dir.y
        const m3d::scalar sy = (dir.y >= 0.0) ? c.half_height : -c.half_height;

        // XZ component: project dir onto the XZ plane and normalize to radius
        const m3d::scalar d_radial = m3d::sqrt(dir.x * dir.x + dir.z * dir.z);

        if (d_radial < m3d::EPSILON)
        {
            // Purely axial direction — XZ position is arbitrary, pick (r, 0)
            return m3d::vec3(c.base_radius, sy, 0.0);
        }

        const m3d::scalar scale = c.base_radius / d_radial;
        return m3d::vec3(dir.x * scale, sy, dir.z * scale);
    }

    /** @brief Volume `π · r² · 2h`. @ingroup rbc */
    inline m3d::scalar compute_volume(const Cylinder &c)
    {
        return m3d::PI * c.base_radius * c.base_radius * (2.0 * c.half_height);
    }

    /**
     * @brief Inertia tensor of a uniform-density cylinder about its centre.
     *
     * - `Iyy (axial)      = (1/2) · m · R²`
     * - `Ixx = Izz (transverse) = (1/12) · m · (3R² + H_total²)`
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const Cylinder &c)
    {
        const m3d::scalar mass  = compute_volume(c);
        const m3d::scalar R2    = c.base_radius * c.base_radius;
        const m3d::scalar H2    = (2.0 * c.half_height) * (2.0 * c.half_height);
        const m3d::scalar Iyy   = 0.5  * mass * R2;
        const m3d::scalar Ixx   = (1.0 / 12.0) * mass * (3.0 * R2 + H2);
        return m3d::smat3(Ixx, Iyy, Ixx, 0.0, 0.0, 0.0);
    }

    /**
     * @brief Tight world AABB via 6-direction support sweep.
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Cylinder &c, const m3d::tf &tf)
    {
        const m3d::vec3 world_axes[6] = {
            {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};

        m3d::vec3 mn = tf.pos, mx = tf.pos;
        for (const auto &axis : world_axes)
        {
            const m3d::vec3 local_dir  = tf.inverse_rotate_vector(axis);
            const m3d::vec3 world_sup  = tf.pos + tf.rotate_vector(support(c, local_dir));
            mn = m3d::vec3(m3d::min(mn.x, world_sup.x),
                           m3d::min(mn.y, world_sup.y),
                           m3d::min(mn.z, world_sup.z));
            mx = m3d::vec3(m3d::max(mx.x, world_sup.x),
                           m3d::max(mx.y, world_sup.y),
                           m3d::max(mx.z, world_sup.z));
        }
        return {mn, mx};
    }

    /** @brief Tag-dispatched marker: cylinder is a convex bounded shape. @ingroup rbc */
    constexpr bool is_gjk_convex(const Cylinder *) { return true; }

    /** @brief Representative size = max(base_radius, half_height). @ingroup rbc */
    inline m3d::scalar representative_radius(const Cylinder &c)
    {
        return m3d::max(c.base_radius, c.half_height);
    }

    /** @brief Disc-approximation face polygon. @ingroup rbc */
    inline int face_corners(const Cylinder &c, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 *out, int capacity)
    {
        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        const m3d::vec3 sup_world = tf.transform_point(support(c, local_dir));
        return get_generic_face_corners(sup_world, dir, representative_radius(c), out, capacity);
    }

} // namespace rbc