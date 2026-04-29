#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

/**
 * @file Sphere.hpp
 * @brief Sphere collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Sphere of given `radius`, centred on the body's transform origin.
     *
     * The simplest convex collision shape. AABB and inertia tensor are exact;
     * `face_corners` falls back to the disc approximation.
     *
     * @ingroup rbc
     */
    struct Sphere
    {
        m3d::scalar radius; ///< Sphere radius.

        /** @brief Default sphere of radius 0.5. */
        Sphere() : radius(0.5) {}
        /** @brief Construct with the given radius. */
        Sphere(const m3d::scalar &radius) : radius(radius) {}

        /** @brief Equality on radius. */
        inline bool operator==(const Sphere &other) const
        {
            return radius == other.radius;
        }

        /** @brief Inequality. */
        inline bool operator!=(const Sphere &other) const
        {
            return !(*this == other);
        }
    };

    /**
     * @brief Local-space support: `radius * normalize(direction)`.
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Sphere &s, const m3d::vec3 &direction)
    {
        return m3d::normalize(direction) * s.radius;
    }

    /** @brief Sphere volume `(4/3)·π·r³`. @ingroup rbc */
    inline m3d::scalar compute_volume(const Sphere &s)
    {
        return (4.0 / 3.0) * m3d::PI * s.radius * s.radius * s.radius;
    }

    /**
     * @brief Inertia tensor of a uniform-density sphere about its centre.
     *
     * `Ixx = Iyy = Izz = (2/5)·m·r²`, off-diagonals zero. Mass is taken as
     * the volume (unit density) — multiply the result by your body's
     * density at construction time.
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const Sphere &s)
    {
        m3d::scalar mass = compute_volume(s);
        m3d::scalar inertia = (2.0 / 5.0) * mass * s.radius * s.radius;
        return m3d::smat3(
            inertia, // Ixx
            inertia, // Iyy
            inertia, // Izz
            0,       // Ixy
            0,       // Ixz
            0        // Iyz
        );
    }

    /** @brief World AABB: `[tf.pos - r, tf.pos + r]`. @ingroup rbc */
    inline AABB compute_aabb(const Sphere &s, const m3d::tf &tf)
    {
        const m3d::vec3 r(s.radius, s.radius, s.radius);
        return {tf.pos - r, tf.pos + r};
    }

    /**
     * @brief Tag-dispatched marker: Sphere is a convex bounded shape (true).
     *
     * The pointer is never dereferenced; this is a compile-time predicate.
     *
     * @ingroup rbc
     */
    constexpr bool is_gjk_convex(const Sphere *) { return true; }

    /** @brief Representative size = the radius. @ingroup rbc */
    inline m3d::scalar representative_radius(const Sphere &s) { return s.radius; }

    /**
     * @brief Disc-approximation face polygon at the support point along `dir`.
     *
     * Spheres have no flat face; this routes through `get_generic_face_corners`
     * so the contact manifold generator gets a usable 4-corner patch.
     *
     * @ingroup rbc
     */
    inline int face_corners(const Sphere &s, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 out[4])
    {
        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        const m3d::vec3 sup_world = tf.transform_point(support(s, local_dir));
        return get_generic_face_corners(sup_world, dir, representative_radius(s), out);
    }

}
