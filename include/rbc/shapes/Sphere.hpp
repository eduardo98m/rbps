#pragma once
#include <math3d/math3d.hpp>
#include "CollisionShape.hpp"

namespace rbc
{
    struct Sphere : public CollisionShape
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

        inline m3d::vec3 support(const m3d::vec3 &direction) const override
        {
            // The furthest point of a sphere in any direction is just the radius along that direction
            return m3d::normalize(direction) * radius;
        }

        inline m3d::scalar compute_volume() const
        {
            return (4.0 / 3.0) * m3d::PI * radius * radius * radius; // Volume of the Sphere
        }

        inline m3d::smat3 compute_inertia_tensor() const
        {
            m3d::scalar mass = compute_volume(); // Assuming unit density for simplicity (How we should ahndle this? Should we add a density parameter to the Spehere struct?)
            m3d::scalar inertia = (2.0 / 5.0) * mass * radius * radius;
            return m3d::smat3(
                inertia, // Ixx
                inertia, // Iyy
                inertia, // Izz
                0,       // Ixy
                0,       // Ixz
                0        // Iyz
            );
        }
    };
}