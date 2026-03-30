#pragma once
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Capsule.hpp"

namespace rbc
{
    // ── Capsule vs Capsule ────────────────────────────────────────────────────
    // Find closest points between the two capsule axes (Ericson §5.1.9),
    // then test against the sum of radii.
    template <>
    struct CollisionAlgorithm<Capsule, Capsule>
    {
        static bool test(const Capsule &a, const m3d::tf &tf_a,
                         const Capsule &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            m3d::vec3 p1, p2, q1, q2;
            capsule_endpoints(a, tf_a, p1, p2);
            capsule_endpoints(b, tf_b, q1, q2);

            // ── Closest points between segments [p1,p2] and [q1,q2] ───────────
            const m3d::vec3 d1 = p2 - p1;
            const m3d::vec3 d2 = q2 - q1;
            const m3d::vec3 r = p1 - q1;

            const m3d::scalar A = m3d::length_sq(d1); // squared length of seg1
            const m3d::scalar E = m3d::length_sq(d2); // squared length of seg2
            const m3d::scalar F = m3d::dot(d2, r);

            m3d::scalar s, t;

            if (A < m3d::EPSILON && E < m3d::EPSILON)
            {
                // Both degenerate (point vs point)
                s = t = 0.0;
            }
            else if (A < m3d::EPSILON)
            {
                s = 0.0;
                t = m3d::clamp(F / E, m3d::scalar(0), m3d::scalar(1));
            }
            else
            {
                const m3d::scalar C = m3d::dot(d1, r);
                if (E < m3d::EPSILON)
                {
                    t = 0.0;
                    s = m3d::clamp(-C / A, m3d::scalar(0), m3d::scalar(1));
                }
                else
                {
                    const m3d::scalar B = m3d::dot(d1, d2);
                    const m3d::scalar denom = A * E - B * B; // always >= 0

                    if (denom > m3d::EPSILON)
                        s = m3d::clamp((B * F - C * E) / denom, m3d::scalar(0), m3d::scalar(1));
                    else
                        s = 0.0; // parallel segments — pick s=0

                    // Compute t from s, then re-clamp s from t if t went out of range
                    t = (B * s + F) / E;
                    if (t < 0.0)
                    {
                        t = 0.0;
                        s = m3d::clamp(-C / A, m3d::scalar(0), m3d::scalar(1));
                    }
                    else if (t > 1.0)
                    {
                        t = 1.0;
                        s = m3d::clamp((B - C) / A, m3d::scalar(0), m3d::scalar(1));
                    }
                }
            }

            const m3d::vec3 closest_a = p1 + d1 * s;
            const m3d::vec3 closest_b = q1 + d2 * t;
            const m3d::vec3 delta = closest_a - closest_b;
            const m3d::scalar dist = m3d::length(delta);
            const m3d::scalar rsum = a.radius + b.radius;

            if (dist >= rsum)
                return false;

            out.normal = (dist > m3d::EPSILON)
                             ? delta / dist
                             : m3d::vec3(1.0, 0.0, 0.0);
            out.penetration_depth = rsum - dist;
            out.pos = closest_b + out.normal * b.radius; // surface of B toward A
            return true;
        }
    };
} // namespace rbc