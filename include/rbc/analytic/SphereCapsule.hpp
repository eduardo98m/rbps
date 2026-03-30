#pragma once
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Capsule.hpp"
#include "rbc/shapes/Sphere.hpp"

namespace rbc
{
    // ── Sphere vs Capsule ─────────────────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Sphere, Capsule>
    {
        static bool test(const Sphere &sphere, const m3d::tf &tf_sphere,
                         const Capsule &capsule, const m3d::tf &tf_capsule,
                         ContactManifold &manifold)
        {
            // World-space capsule endpoints
            m3d::vec3 p1, p2;
            capsule_endpoints(capsule, tf_capsule, p1, p2);

            // Closest point on segment [p1,p2] to sphere centre
            const m3d::vec3 d = p2 - p1;
            const m3d::scalar len2 = m3d::length_sq(d);
            
            m3d::scalar t = 0.0;
            if (len2 > m3d::EPSILON)
                t = m3d::clamp(m3d::dot(tf_sphere.pos - p1, d) / len2,
                               m3d::scalar(0), m3d::scalar(1));

            const m3d::vec3 closest = p1 + d * t;
            const m3d::vec3 delta = tf_sphere.pos - closest;
            const m3d::scalar dist = m3d::length(delta);
            const m3d::scalar rsum = sphere.radius + capsule.radius;

            if (dist >= rsum)
                return false;

            manifold.num_points = 1;
            manifold.normal = (dist > m3d::EPSILON)
                             ? - delta / dist
                             : m3d::vec3(1.0, 0.0, 0.0);
                             
            manifold.points[0].penetration_depth = rsum - dist;
            manifold.points[0].position = closest - manifold.normal * capsule.radius;
            
            return true;
        }
    };

    template <>
    struct CollisionAlgorithm<Capsule, Sphere> : CollisionAlgorithmSym<Capsule, Sphere> {};
} // namespace rbc