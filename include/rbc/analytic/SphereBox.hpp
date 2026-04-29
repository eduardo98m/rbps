#pragma once
#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/Contact.hpp"
#include <math3d/math3d.hpp>

/**
 * @file SphereBox.hpp
 * @brief Analytic Sphere–Box collision algorithm.
 * @ingroup rbc
 * @ingroup internals
 *
 * Strategy: find the closest point on the OBB surface to the sphere centre.
 * If the distance is less than the sphere's radius, the pair overlaps.
 * Two cases are handled: sphere centre outside the box (closest-point
 * clamp on the local AABB) and sphere centre inside the box (find the
 * face with smallest exit penetration).
 *
 * @note **Normal convention:** A → B (sphere → box). The XPBD solver
 *       expects this orientation so that the impulse pushes the sphere
 *       away from the box.
 */

namespace rbc
{

    /**
     * @brief Closest-point analytic test for sphere vs OBB; produces 1 contact point.
     * @ingroup rbc
     * @ingroup internals
     */
    template <>
    struct CollisionAlgorithm<Sphere, Box>
    {
        /** @brief Run sphere–box. */
        static bool test(const Sphere &sphere, const m3d::tf &tf_sphere,
                         const Box    &box,    const m3d::tf &tf_box,
                         ContactManifold &out)
        {
            // Sphere centre expressed in the box's local frame.
            const m3d::vec3 local_centre =
                tf_box.inverse_rotate_vector(tf_sphere.pos - tf_box.pos);

            // Closest point on the box AABB (local space) to the sphere centre.
            const m3d::vec3 closest(
                m3d::clamp(local_centre.x, -box.half_extents.x, box.half_extents.x),
                m3d::clamp(local_centre.y, -box.half_extents.y, box.half_extents.y),
                m3d::clamp(local_centre.z, -box.half_extents.z, box.half_extents.z));

            const m3d::vec3  local_diff = local_centre - closest; // B→A (box surface → sphere)
            const m3d::scalar dist      = m3d::length(local_diff);

            // ── Case 1: sphere centre is OUTSIDE the box ──────────────────────
            if (dist > m3d::EPSILON)
            {
                if (dist >= sphere.radius)
                    return false; // separated

                // local_diff / dist  points B→A.
                // Negate for the required A→B convention (sphere → box).
                out.num_points = 1;
                out.normal            = -tf_box.rotate_vector(local_diff / dist);
                out.points[0].penetration_depth = sphere.radius - dist;
                out.points[0].position          = tf_box.transform_point(closest); // box surface contact
                return true;
            }

            // ── Case 2: sphere centre is INSIDE the box ───────────────────────
            // Find the face whose penetration is smallest — that is the exit direction.
            // Penetration along each face axis = half_extent − |local_coord|
            const m3d::scalar px = box.half_extents.x - m3d::abs(local_centre.x);
            const m3d::scalar py = box.half_extents.y - m3d::abs(local_centre.y);
            const m3d::scalar pz = box.half_extents.z - m3d::abs(local_centre.z);

            m3d::vec3 local_normal; // outward face normal in box local space (B→A)
            if (px <= py && px <= pz)
                local_normal = m3d::vec3(local_centre.x > 0 ? 1.0 : -1.0, 0.0, 0.0);
            else if (py <= px && py <= pz)
                local_normal = m3d::vec3(0.0, local_centre.y > 0 ? 1.0 : -1.0, 0.0);
            else
                local_normal = m3d::vec3(0.0, 0.0, local_centre.z > 0 ? 1.0 : -1.0);

            // Negate outward face normal (B→A) → A→B convention for the solver.
            out.num_points = 1;
            out.normal            = -tf_box.rotate_vector(local_normal);
            out.points[0].penetration_depth = sphere.radius + m3d::min(px, m3d::min(py, pz));

            // Contact point: closest point on the exit face to the sphere centre.
            // The exit-face axis is fixed by local_normal (one component is ±1, rest 0).
            // Along that axis use the face position; along the other axes clamp to box.
            const m3d::vec3 face_point(
                local_normal.x != 0.0
                    ? local_normal.x * box.half_extents.x
                    : m3d::clamp(local_centre.x, -box.half_extents.x, box.half_extents.x),
                local_normal.y != 0.0
                    ? local_normal.y * box.half_extents.y
                    : m3d::clamp(local_centre.y, -box.half_extents.y, box.half_extents.y),
                local_normal.z != 0.0
                    ? local_normal.z * box.half_extents.z
                    : m3d::clamp(local_centre.z, -box.half_extents.z, box.half_extents.z));

            out.points[0].position = tf_box.transform_point(face_point);
            return true;
        }
    };

    /**
     * @brief Box vs Sphere — reuses `<Sphere, Box>` and flips the normal.
     * @ingroup rbc
     * @ingroup internals
     */
    template <>
    struct CollisionAlgorithm<Box, Sphere>
        : CollisionAlgorithmSym<Box, Sphere>
    {
    };

} // namespace rbc