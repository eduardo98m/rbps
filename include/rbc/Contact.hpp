#pragma once
#include <math3d/math3d.hpp>

namespace rbc
{

    struct ContactPoint
    {
        m3d::vec3 position;
        m3d::scalar penetration_depth;
    };

    struct ContactManifold
    {
        m3d::vec3 normal; // from A toward B
        ContactPoint points[4];
        int num_points = 0;

        // Convenience: single-point access (backwards-compat for GJK/EPA path)
        const ContactPoint &first() const { return points[0]; }
 
        // Reset
        void clear() { num_points = 0; }
    };

} // namespace rbc