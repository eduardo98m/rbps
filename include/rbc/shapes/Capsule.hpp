#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

/**
 * @file Capsule.hpp
 * @brief Capsule (line segment with spherical caps) collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Capsule = a Y-axis cylinder of radius `radius` capped with two hemispheres.
     *
     * In local space the principal axis is +Y:
     * - Top hemisphere centre = `(0, +half_height, 0)`.
     * - Bottom hemisphere centre = `(0, -half_height, 0)`.
     * - Total height = `2 * (half_height + radius)`.
     *
     * Algorithmically a Minkowski sum of a segment and a sphere, which makes
     * the support function and AABB cheap.
     *
     * @ingroup rbc
     */
    struct Capsule
    {
        m3d::scalar half_height; ///< Distance from centre to each cap centre.
        m3d::scalar radius;      ///< Cap and cylinder radius.

        /** @brief Default capsule of half-height 0.5 and radius 0.25. */
        Capsule() : half_height(0.5), radius(0.25) {}
        /** @brief Construct with explicit dimensions. */
        Capsule(m3d::scalar half_height, m3d::scalar radius)
            : half_height(half_height), radius(radius) {}

        /** @brief Equality on both fields. */
        inline bool operator==(const Capsule &o) const
        { return half_height == o.half_height && radius == o.radius; }
        /** @brief Inequality. */
        inline bool operator!=(const Capsule &o) const { return !(*this == o); }
    };

    /**
     * @brief World-space endpoints of the capsule's central segment.
     *
     * @param c        Capsule shape.
     * @param tf       Body transform.
     * @param[out] p1_out  Endpoint on the +Y side of local space.
     * @param[out] p2_out  Endpoint on the −Y side of local space.
     *
     * @ingroup rbc
     */
    inline void capsule_endpoints(const Capsule &c, const m3d::tf &tf,
                                  m3d::vec3 &p1_out, m3d::vec3 &p2_out)
    {
        const m3d::vec3 axis = tf.rotate_vector(m3d::vec3(0, c.half_height, 0));
        p1_out = tf.pos + axis;
        p2_out = tf.pos - axis;
    }

    /**
     * @brief Local-space support: pick the cap centre on the side `dir.y` points to,
     *        then nudge by `radius * normalize(dir)`.
     *
     * The Minkowski-sum identity: capsule = segment ⊕ sphere, so
     * `support_capsule = support_segment + support_sphere`.
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Capsule &c, const m3d::vec3 &dir)
    {
        const m3d::vec3 endpoint(0.0,
                                  (dir.y >= 0.0 ? c.half_height : -c.half_height),
                                  0.0);
        const m3d::scalar len = m3d::length(dir);
        if (len < m3d::EPSILON) return endpoint;
        return endpoint + (dir / len) * c.radius;
    }

    /** @brief Volume = cylinder + two hemispheres = `π·r²·(2h + 4r/3)`. @ingroup rbc */
    inline m3d::scalar compute_volume(const Capsule &c)
    {
        const m3d::scalar r = c.radius, h = c.half_height;
        return m3d::PI * r * r * (2.0 * h + (4.0 / 3.0) * r);
    }

    /**
     * @brief Inertia tensor of a uniform-density capsule about its centre.
     *
     * Built from the cylinder + two hemispheres composition; the hemisphere
     * contribution to the transverse axis uses the parallel-axis theorem
     * with offset `h + 3r/8`. Reference: Eberly, *Game Physics*, capsule
     * moment integrals.
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const Capsule &c)
    {
        const m3d::scalar r  = c.radius, h = c.half_height;
        const m3d::scalar r2 = r * r;
        const m3d::scalar h2 = h * h;
        const m3d::scalar mass = compute_volume(c);

        const m3d::scalar m_cyl  = m3d::PI * r2 * 2.0 * h;
        const m3d::scalar m_hemi = mass - m_cyl;

        const m3d::scalar Iyy = (m_cyl * r2 / 2.0) +
                                  (m_hemi * 2.0 * r2 / 5.0);

        const m3d::scalar Ixx_cyl = m_cyl * (r2 / 4.0 + h2 / 3.0);
        const m3d::scalar d_hemi  = h + 3.0 * r / 8.0;
        const m3d::scalar Ixx_hemi = m_hemi * (2.0 * r2 / 5.0 + d_hemi * d_hemi);
        const m3d::scalar Ixx = Ixx_cyl + Ixx_hemi;

        return m3d::smat3(Ixx, Iyy, Ixx, 0.0, 0.0, 0.0);
    }

    /**
     * @brief Tight world AABB.
     *
     * Computed as the union of two `radius`-sized boxes around the world
     * endpoints — exact for a capsule and cheaper than a 6-direction
     * support sweep.
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Capsule &c, const m3d::tf &tf)
    {
        const m3d::vec3 axis = tf.rotate_vector(m3d::vec3(0, c.half_height, 0));
        const m3d::vec3 p1   = tf.pos + axis;
        const m3d::vec3 p2   = tf.pos - axis;
        const m3d::vec3 r(c.radius, c.radius, c.radius);
        return {
            m3d::vec3(m3d::min(p1.x, p2.x), m3d::min(p1.y, p2.y), m3d::min(p1.z, p2.z)) - r,
            m3d::vec3(m3d::max(p1.x, p2.x), m3d::max(p1.y, p2.y), m3d::max(p1.z, p2.z)) + r
        };
    }

    /** @brief Tag-dispatched marker: Capsule is a convex bounded shape (true). @ingroup rbc */
    constexpr bool is_gjk_convex(const Capsule *) { return true; }

    /** @brief Representative size = the cap radius. @ingroup rbc */
    inline m3d::scalar representative_radius(const Capsule &c) { return c.radius; }

    /** @brief Disc-approximation face polygon (capsule has no flat face). @ingroup rbc */
    inline int face_corners(const Capsule &c, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 *out, int capacity)
    {
        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        const m3d::vec3 sup_world = tf.transform_point(support(c, local_dir));
        return get_generic_face_corners(sup_world, dir, representative_radius(c), out, capacity);
    }
} // namespace rbc
