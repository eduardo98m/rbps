#pragma once
#include <math3d/math3d.hpp>

/**
 * @file AABB.hpp
 * @brief Axis-aligned bounding box and utility free functions.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Axis-Aligned Bounding Box stored as min/max corners in world space.
     *
     * The default constructor produces an *inverted* AABB (`min = +inf`,
     * `max = -inf`) so that successive `aabb_union` calls correctly grow
     * from an empty starting box.
     *
     * Used by the broad phase as the per-object bounding volume; routinely
     * "fattened" by a small margin (`aabb_expand`) to give temporal
     * coherence when objects move slightly between frames.
     *
     * @ingroup rbc
     */
    struct AABB
    {
        m3d::vec3 min; ///< Min corner (inclusive).
        m3d::vec3 max; ///< Max corner (inclusive).

        /** @brief Inverted ("empty") AABB: `min = +inf`, `max = -inf`. */
        AABB() : min(m3d::vec3(std::numeric_limits<m3d::scalar>::infinity())),
                 max(m3d::vec3(-std::numeric_limits<m3d::scalar>::infinity())) {}

        /** @brief Construct from explicit corners. */
        AABB(const m3d::vec3 &min, const m3d::vec3 &max) : min(min), max(max) {}

        /** @brief Equality on both corners. */
        bool operator==(const AABB &other) const
        {
            return min == other.min && max == other.max;
        }

        /** @brief Inequality. */
        bool operator!=(const AABB &other) const
        {
            return !(*this == other);
        }
    };

    /**
     * @brief Test whether two AABBs overlap (separating-axis test on all 3 axes).
     * @ingroup rbc
     */
    inline bool aabb_overlap(const AABB &a, const AABB &b)
    {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    /**
     * @brief Smallest AABB enclosing both `a` and `b`.
     * @ingroup rbc
     */
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

    /**
     * @brief Expand the AABB by a uniform `margin` on all sides.
     *
     * Used to "fatten" AABBs for broad-phase temporal coherence: a moving
     * object only triggers a re-sort when its tight AABB escapes its
     * fattened wrapper.
     *
     * @ingroup rbc
     */
    inline AABB aabb_expand(const AABB &a, m3d::scalar margin)
    {
        const m3d::vec3 m(margin, margin, margin);
        return {a.min - m, a.max + m};
    }

    /** @brief Geometric centre of the AABB. @ingroup rbc */
    inline m3d::vec3 aabb_center(const AABB &a)
    {
        return (a.min + a.max) * 0.5;
    }

    /** @brief Half-extents `(max - min) / 2`. @ingroup rbc */
    inline m3d::vec3 aabb_half_extents(const AABB &a)
    {
        return (a.max - a.min) * 0.5;
    }

    /**
     * @brief Surface area of the AABB.
     *
     * Used by SAH-style cost heuristics — handy if the broad phase ever
     * switches from sweep-and-prune to a BVH.
     *
     * @ingroup rbc
     */
    inline m3d::scalar aabb_surface_area(const AABB &a)
    {
        const m3d::vec3 d = a.max - a.min;
        return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

} // namespace rbc
