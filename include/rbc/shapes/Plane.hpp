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
    // A half-space is unbounded in the normal direction — support returns
    // a very large (but finite) value so we can still call it safely.
    // DO NOT feed Plane to GJK; use analytic specialisations instead.
    inline m3d::vec3 support(const Plane &p, const m3d::vec3 &dir)
    {
        // If dir points into the half-space, support is "at infinity" — clamp to large value.
        constexpr m3d::scalar BIG = 1.0e15;
        if (m3d::dot(dir, p.normal) > 0.0)
            return p.normal * BIG;
        // Otherwise, any point on the plane surface; use origin projected onto plane.
        return p.normal * p.d;
    }

    // ── AABB: infinite in all directions ─────────────────────────────────────
    inline AABB compute_aabb(const Plane & /*p*/, const m3d::tf & /*tf*/)
    {
        constexpr m3d::scalar INF = std::numeric_limits<m3d::scalar>::infinity();
        return {m3d::vec3(-INF, -INF, -INF), m3d::vec3(INF, INF, INF)};
    }
} // namespace rbc