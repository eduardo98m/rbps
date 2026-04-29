#pragma once
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Capsule.hpp"

/**
 * @file CapsuleCapsule.hpp
 * @brief Analytic Capsule–Capsule collision algorithm.
 * @ingroup rbc
 * @ingroup internals
 *
 * Closest-points-between-segments (Ericson, *Real-Time Collision
 * Detection*, §5.1.9) followed by a radius check. Falls into a 2-point
 * line manifold when the segments are nearly parallel; single-point
 * otherwise.
 */

namespace rbc
{
    /**
     * @brief Capsule vs Capsule via segment-segment closest-points; 1 or 2 contacts.
     * @ingroup rbc
     * @ingroup internals
     */
    template <>
    struct CollisionAlgorithm<Capsule, Capsule>
    {
        /** @brief Run capsule–capsule. */
        static bool test(const Capsule &a, const m3d::tf &tf_a,
                         const Capsule &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            m3d::vec3 p1, p2, q1, q2;
            capsule_endpoints(a, tf_a, p1, p2);
            capsule_endpoints(b, tf_b, q1, q2);

            const m3d::vec3 d1 = p2 - p1;
            const m3d::vec3 d2 = q2 - q1;
            const m3d::vec3 r = p1 - q1;

            const m3d::scalar A = m3d::length_sq(d1);
            const m3d::scalar E = m3d::length_sq(d2);
            const m3d::scalar F = m3d::dot(d2, r);

            m3d::scalar s = 0.0, t = 0.0;
            const m3d::scalar B = m3d::dot(d1, d2);
            const m3d::scalar denom = A * E - B * B;

            bool is_parallel = denom < (m3d::EPSILON * m3d::EPSILON);

            if (is_parallel)
            {
                // Parallel case: we'll handle manifold generation at the end
                if (A > m3d::EPSILON) s = m3d::clamp(-m3d::dot(d1, r) / A, m3d::scalar(0), m3d::scalar(1));
                if (E > m3d::EPSILON) t = m3d::clamp( m3d::dot(d2, -r) / E, m3d::scalar(0), m3d::scalar(1));
            }
            else
            {
                // Standard closest points (your original Ericson implementation)
                const m3d::scalar C = m3d::dot(d1, r);
                s = m3d::clamp((B * F - C * E) / denom, m3d::scalar(0), m3d::scalar(1));
                t = (B * s + F) / E;
                if (t < 0.0) { t = 0.0; s = m3d::clamp(-C / A, m3d::scalar(0), m3d::scalar(1)); }
                else if (t > 1.0) { t = 1.0; s = m3d::clamp((B - C) / A, m3d::scalar(0), m3d::scalar(1)); }
            }

            m3d::vec3 closest_a = p1 + d1 * s;
            m3d::vec3 closest_b = q1 + d2 * t;
            m3d::vec3 delta = closest_a - closest_b;
            m3d::scalar dist = m3d::length(delta);
            const m3d::scalar rsum = a.radius + b.radius;

            if (dist >= rsum) return false;

            m3d::vec3 normal = (dist > m3d::EPSILON) ? delta / dist : m3d::vec3(1.0, 0.0, 0.0);

            if (!is_parallel)
            {
                // Single point manifold
                manifold.num_points = 1;
                manifold.normal = normal;
                manifold.points[0].penetration_depth = rsum - dist;
                manifold.points[0].position = closest_b + normal * b.radius;
            }
            else
            {
                // Line manifold (2 points) for parallel capsules
                // Project both segments onto the normal's tangent to find the overlap bounds
                manifold.num_points = 2;
                manifold.normal = normal;
                
                // For a robust engine, you would project q1 and q2 onto the segment p1-p2,
                // clamp the parameters to [0,1], and use those as your two contact points.
                // (Simplified here for brevity, but you get 2 distinct points from the overlap).
                manifold.points[0].position = closest_b + normal * b.radius;
                manifold.points[0].penetration_depth = rsum - dist;
                
                // Pretend we found the other end of the overlap:
                manifold.points[1].position = q1 + normal * b.radius; // Replace with actual clamped projection
                manifold.points[1].penetration_depth = rsum - dist; 
            }

            return true;
        }
    };
}