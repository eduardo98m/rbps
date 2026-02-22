#pragma once
#include <math3d/math3d.hpp>

namespace rbc
{
    struct Ellipsoid
    {
        m3d::vec3 half_extents;
        // Constructors
        Ellipsoid() : half_extents(0.5, 0.5, 0.5) {}
        Ellipsoid(const m3d::vec3 &half_extents) : half_extents(half_extents) {}

        inline bool operator==(const Ellipsoid &other) const
        {
            return half_extents == other.half_extents;
        }

        inline bool operator!=(const Ellipsoid &other) const
        {
            return !(*this == other);
        }

        inline m3d::vec3 support(const m3d::vec3 &direction) const
        {
            return m3d::vec3(); // Placeholder: Implement the support function for the Ellipsoid shape
        }

        inline m3d::scalar compute_volume() const
        {
            return (4.0 / 3.0) * m3d::PI * half_extents.x * half_extents.y * half_extents.z; // Volume of the Ellipsoid
        }

        inline m3d::smat3 compute_inertia_tensor() const
        {
            m3d::scalar x2 = 4.0 * half_extents.x * half_extents.x;
            m3d::scalar y2 = 4.0 * half_extents.y * half_extents.y;
            m3d::scalar z2 = 4.0 * half_extents.z * half_extents.z;
            m3d::scalar mass = compute_volume(); // Assuming unit density for simplicity (How we should ahndle this? Should we add a density parameter to the Ellipsoid struct?)
            return m3d::smat3(
                (1.0 / 5.0) * mass * (y2 + z2), // Ixx
                (1.0 / 5.0) * mass * (x2 + z2), // Iyy
                (1.0 / 5.0) * mass * (x2 + y2), // Izz
                0,                                // Ixy
                0,                                // Ixz
                0                                 // Iyz
            );
        }
    };
}