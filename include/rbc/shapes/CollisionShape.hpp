#pragma once
#include <math3d/math3d.hpp>

namespace rbc {
    // Abstract base class for all collision shapes. GJK only needs the support function, so that's the only thing we require.
    struct CollisionShape {
        virtual ~CollisionShape() = default;
        
        // This is the core of GJK/EPA: returns the furthest point in a given direction (in local space).
        virtual m3d::vec3 support(const m3d::vec3& direction) const = 0;
    };
}