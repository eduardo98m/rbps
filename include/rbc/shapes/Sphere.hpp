#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

namespace rbc
{
    struct Sphere
    {
        m3d::scalar radius;
        // Constructors
        Sphere() : radius(0.5) {}
        Sphere(const m3d::scalar &radius) : radius(radius) {}

        inline bool operator==(const Sphere &other) const
        {
            return radius == other.radius;
        }

        inline bool operator!=(const Sphere &other) const
        {
            return !(*this == other);
        }
    };

    inline m3d::vec3 support(const Sphere &s, const m3d::vec3 &direction)
    {
        // The furthest point of a sphere in any direction is just the radius along that direction
        return m3d::normalize(direction) * s.radius;
    }

    inline m3d::scalar compute_volume(const Sphere &s)
    {
        return (4.0 / 3.0) * m3d::PI * s.radius * s.radius * s.radius; // Volume of the Sphere
    }

    inline m3d::smat3 compute_inertia_tensor(const Sphere &s)
    {
        m3d::scalar mass = compute_volume(s); // Assuming unit density for simplicity (How we should ahndle this? Should we add a density parameter to the Spehere struct?)
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

    // Sphere: trivially radius in all directions from center.
    inline AABB compute_aabb(const Sphere &s, const m3d::tf &tf)
    {
        const m3d::vec3 r(s.radius, s.radius, s.radius);
        return {tf.pos - r, tf.pos + r};
    }

    // Marker for the dispatcher: Sphere is a convex bounded shape.
    // Tag-dispatched (pointer arg, never dereferenced) so callers can query
    // convexity by type without constructing a Sphere.
    constexpr bool is_gjk_convex(const Sphere *) { return true; }

    inline m3d::scalar representative_radius(const Sphere &s) { return s.radius; }

    inline int face_corners(const Sphere &s, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 out[4])
    {
        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        const m3d::vec3 sup_world = tf.transform_point(support(s, local_dir));
        return get_generic_face_corners(sup_world, dir, representative_radius(s), out);
    }

}