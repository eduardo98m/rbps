#pragma once
#include <limits>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    // ── Plane (infinite half-space) ───────────────────────────────────────────
    // The plane surface in local space: normal · x = d
    // The solid half-space is  normal · x >= d  ("above" the plane).
    //
    // World-space plane equation (applying transform tf):
    //   world_normal = tf.rotate_vector(normal)
    //   world_d      = d + dot(world_normal, tf.pos)
    //
    // IMPORTANT: Plane is an *infinite* shape. GJK cannot be used against it.
    //   All CollisionAlgorithm<*, Plane> specialisations must be analytic.
    //   They live in rbc/analytic/PlaneCollision.hpp.
    struct Plane
    {
        m3d::vec3 normal = {0.0, 1.0, 0.0}; // unit normal (local space)
        m3d::scalar d = 0.0;                // offset: normal·x = d on surface

        Plane() = default;
        Plane(const m3d::vec3 &normal, m3d::scalar d) : normal(normal), d(d) {}

        inline bool operator==(const Plane &o) const { return normal == o.normal && d == o.d; }
        inline bool operator!=(const Plane &o) const { return !(*this == o); }
    };

    // ── Helpers: world-space plane parameters ────────────────────────────────
    inline m3d::vec3 plane_world_normal(const Plane &p, const m3d::tf &tf)
    {
        return tf.rotate_vector(p.normal);
    }

    inline m3d::scalar plane_world_d(const Plane &p, const m3d::tf &tf)
    {
        return p.d + m3d::dot(plane_world_normal(p, tf), tf.pos);
    }

    // ── Support function ──────────────────────────────────────────────────────
    // Plane is non-convex (an infinite half-space) and is_gjk_convex<Plane>
    // returns false, so the dispatcher's compile-time guard ensures GJK never
    // calls this. The stub exists only because the variant-level shape_support
    // dispatch needs an overload for every alternative.
    inline m3d::vec3 support(const Plane &, const m3d::vec3 &) { return m3d::vec3(); }

    // ── AABB: infinite in all directions ─────────────────────────────────────
    inline AABB compute_aabb(const Plane & /*p*/, const m3d::tf & /*tf*/)
    {
        constexpr m3d::scalar INF = std::numeric_limits<m3d::scalar>::infinity();
        return {m3d::vec3(-INF, -INF, -INF), m3d::vec3(INF, INF, INF)};
    }

    // Marker for the dispatcher: Plane is non-convex (infinite half-space).
    // Plane pairs are handled by analytic specialisations exclusively;
    // these stubs exist only so the variant-level visit/table compiles.
    constexpr bool is_gjk_convex(const Plane *) { return false; }
    inline m3d::scalar representative_radius(const Plane &) { return 0.0; }
    inline int face_corners(const Plane &, const m3d::tf &,
                            const m3d::vec3 &, m3d::vec3[4]) { return 0; }
} // namespace rbc