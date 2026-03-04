#pragma once
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Capsule.hpp"

namespace rbc
{
    // ── Sphere vs Capsule ─────────────────────────────────────────────────────
    // A capsule = segment + sphere of radius r_c. So sphere(r_s) vs capsule(r_c):
    //   1. Find closest point on capsule axis to sphere centre.
    //   2. Test distance against (r_s + r_c).
    template <>
    struct CollisionAlgorithm<Sphere, Capsule>
    {
        static bool test(const Sphere &sphere, const m3d::tf &tf_sphere,
                         const Capsule &capsule, const m3d::tf &tf_capsule,
                         Contact &out)
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

            out.normal = (dist > m3d::EPSILON)
                             ? delta / dist
                             : m3d::vec3(1.0, 0.0, 0.0); // coincident centres
            out.penetration_depth = rsum - dist;
            out.pos = closest + out.normal * capsule.radius; // surface of capsule
            return true;
        }
    };

    // Capsule vs Sphere — symmetric shim
    template <>
    struct CollisionAlgorithm<Capsule, Sphere> : CollisionAlgorithmSym<Capsule, Sphere>
    {
    };

} // namespace rbc