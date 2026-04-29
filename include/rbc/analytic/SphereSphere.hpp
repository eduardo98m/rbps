#pragma once
#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/Contact.hpp"
#include <math3d/math3d.hpp>

/**
 * @file SphereSphere.hpp
 * @brief Analytic Sphere–Sphere collision algorithm.
 * @ingroup rbc
 * @ingroup internals
 *
 * Pure distance check — no SAT, no GJK needed. The only axis that matters
 * is the vector between the two centres.
 */

namespace rbc
{

    /**
     * @brief Distance test: overlap iff `|center_b - center_a| < r_a + r_b`.
     * @ingroup rbc
     * @ingroup internals
     */
    template <>
    struct CollisionAlgorithm<Sphere, Sphere>
    {
        /** @brief Run sphere–sphere; produces 1 contact point on overlap. */
        static bool test(const Sphere &a, const m3d::tf &tf_a,
                         const Sphere &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            const m3d::vec3 delta = tf_b.pos - tf_a.pos;
            const m3d::scalar dist = m3d::length(delta);
            const m3d::scalar rsum = a.radius + b.radius;

            if (dist >= rsum)
                return false; // separated


            // Normal points from A to B (the direction A needs to move to separate)
            manifold.normal = (dist > m3d::EPSILON)
                             ? delta / dist
                             : m3d::vec3(1.0, 0.0, 0.0); // coincident centres — pick arbitrary axis

            manifold.num_points = 1;
            manifold.points[0].penetration_depth = rsum - dist;
            manifold.points[0].position = tf_a.pos + manifold.normal * a.radius;

            return true;
        }
    };

} // namespace rbc