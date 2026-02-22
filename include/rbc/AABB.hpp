#pragma once
#include <math3d/math3d.hpp>

namespace rbc {
    struct AABB{
        m3d::vec3 min;
        m3d::vec3 max;

        AABB() : min(m3d::vec3(std::numeric_limits<m3d::scalar>::infinity())),
                 max(m3d::vec3(-std::numeric_limits<m3d::scalar>::infinity())) {}

        AABB(const m3d::vec3 &min, const m3d::vec3 &max) : min(min), max(max) {}

        bool operator==(const AABB &other) const {
            return min == other.min && max == other.max;
        }

        bool operator!=(const AABB &other) const {
            return !(*this == other);
        }
    };
} // namespace rbc