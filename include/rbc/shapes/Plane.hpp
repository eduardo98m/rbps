#pragma once
#include <limits>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

/**
 * @file Plane.hpp
 * @brief Infinite half-space (plane) collision shape.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Infinite half-space defined by `normal · x = d` in local space.
     *
     * The solid region is `normal · x ≥ d` ("above" the plane).
     *
     * Applying transform `tf`, the world-space plane parameters are:
     * - `world_normal = tf.rotate_vector(normal)`
     * - `world_d      = d + dot(world_normal, tf.pos)`
     *
     * @warning Plane is an *infinite* shape — GJK cannot handle it. Every
     *          `*–Plane` collision pair must use an analytic algorithm
     *          (see [analytic/PlaneCollision.hpp](analytic/PlaneCollision.hpp)).
     *          The `support` and `face_corners` overloads here are stubs
     *          that exist only because the variant-level dispatchers need
     *          one overload per shape kind.
     *
     * @ingroup rbc
     */
    struct Plane
    {
        m3d::vec3 normal = {0.0, 1.0, 0.0}; ///< Unit local-space normal (default: +Y up).
        m3d::scalar d = 0.0;                ///< Local-space offset: `normal·x = d` on the surface.

        /** @brief Default plane: y = 0, normal +Y (a floor). */
        Plane() = default;
        /** @brief Construct from a normal and offset. */
        Plane(const m3d::vec3 &normal, m3d::scalar d) : normal(normal), d(d) {}

        /** @brief Equality. */
        inline bool operator==(const Plane &o) const { return normal == o.normal && d == o.d; }
        /** @brief Inequality. */
        inline bool operator!=(const Plane &o) const { return !(*this == o); }
    };

    /** @brief World-space plane normal: `tf.rot * p.normal`. @ingroup rbc */
    inline m3d::vec3 plane_world_normal(const Plane &p, const m3d::tf &tf)
    {
        return tf.rotate_vector(p.normal);
    }

    /** @brief World-space plane offset: `p.d + dot(world_normal, tf.pos)`. @ingroup rbc */
    inline m3d::scalar plane_world_d(const Plane &p, const m3d::tf &tf)
    {
        return p.d + m3d::dot(plane_world_normal(p, tf), tf.pos);
    }

    /**
     * @brief Stub support function — returns zero.
     *
     * Plane is non-convex (infinite). `is_gjk_convex(const Plane*) == false`,
     * so the dispatcher's compile-time guard prevents GJK from ever calling
     * this. The overload exists only so the variant-level `shape_support`
     * dispatch compiles.
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Plane &, const m3d::vec3 &) { return m3d::vec3(); }

    /** @brief AABB is infinite in all directions. @ingroup rbc */
    inline AABB compute_aabb(const Plane & /*p*/, const m3d::tf & /*tf*/)
    {
        constexpr m3d::scalar INF = std::numeric_limits<m3d::scalar>::infinity();
        return {m3d::vec3(-INF, -INF, -INF), m3d::vec3(INF, INF, INF)};
    }

    /** @brief Tag-dispatched marker: Plane is non-convex / unbounded (false). @ingroup rbc */
    constexpr bool is_gjk_convex(const Plane *) { return false; }
    /** @brief Returns 0 — unbounded shape has no representative size. @ingroup rbc */
    inline m3d::scalar representative_radius(const Plane &) { return 0.0; }
    /** @brief Stub — Plane pairs go through analytic algorithms. @ingroup rbc */
    inline int face_corners(const Plane &, const m3d::tf &,
                            const m3d::vec3 &, m3d::vec3 *, int) { return 0; }
} // namespace rbc
