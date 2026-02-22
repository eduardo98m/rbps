#pragma once
#include <math3d/math3d.hpp>

namespace rbc
{
    struct Plane
    {
        m3d::vec3 normal;
        m3d::scalar distance;
        // Constructors
        Plane() : normal(0.0, 1.0, 0.0), distance(0.0) {}
        Plane(const m3d::vec3 &normal, const m3d::scalar &distance) : normal(normal), distance(distance) {}

        inline bool operator==(const Plane &other) const
        {
            return normal == other.normal && distance == other.distance;
        }

        inline bool operator!=(const Plane &other) const
        {
            return !(*this == other);
        }

        inline m3d::vec3 support(const m3d::vec3 &direction) const
        {
            return m3d::vec3(); // Placeholder: Implement the support function for the Plane shape
        }

    };
}