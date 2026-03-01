#pragma once
#include <math3d/math3d.hpp>
namespace rbc
{
    // -------------------------------------------------------------------------
    //  AABB - Axis-Aligned Bounding Box
    //  Stored as min/max corners in world space.
    //  Fattenned by a small margin during broad phase for temporal coherence.
    // -------------------------------------------------------------------------
    struct AABB
    {
        m3d::vec3 min;
        m3d::vec3 max;

        AABB() : min(m3d::vec3(std::numeric_limits<m3d::scalar>::infinity())),
                 max(m3d::vec3(-std::numeric_limits<m3d::scalar>::infinity())) {}

        AABB(const m3d::vec3 &min, const m3d::vec3 &max) : min(min), max(max) {}

        bool operator==(const AABB &other) const
        {
            return min == other.min && max == other.max;
        }

        bool operator!=(const AABB &other) const
        {
            return !(*this == other);
        }
    };

    // -------------------------------------------------------------------------
    //  Basic AABB operations
    // -------------------------------------------------------------------------
    inline bool aabb_overlap(const AABB &a, const AABB &b)
    {
        // Separating axis test on all 3 axes
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    inline AABB aabb_union(const AABB &a, const AABB &b)
    {
        return {
            m3d::vec3(m3d::min(a.min.x, b.min.x),
                      m3d::min(a.min.y, b.min.y),
                      m3d::min(a.min.z, b.min.z)),
            m3d::vec3(m3d::max(a.max.x, b.max.x),
                      m3d::max(a.max.y, b.max.y),
                      m3d::max(a.max.z, b.max.z))};
    }

    // Expand the AABB by a uniform margin on all sides.
    // Used to "fatten" AABBs for temporal coherence (avoids rebuilding the
    // sorted list every frame for objects that barely moved).
    inline AABB aabb_expand(const AABB &a, m3d::scalar margin)
    {
        const m3d::vec3 m(margin, margin, margin);
        return {a.min - m, a.max + m};
    }

    inline m3d::vec3 aabb_center(const AABB &a)
    {
        return (a.min + a.max) * 0.5;
    }

    inline m3d::vec3 aabb_half_extents(const AABB &a)
    {
        return (a.max - a.min) * 0.5;
    }

    // Surface area — used by BVH cost heuristics if you ever switch to one.
    inline m3d::scalar aabb_surface_area(const AABB &a)
    {
        const m3d::vec3 d = a.max - a.min;
        return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

} // namespace rbc