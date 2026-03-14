#pragma once
#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/Contact.hpp"
#include <math3d/math3d.hpp>

namespace rbc
{

    // ── Sphere vs Box ────────────────────────────────────────────────────────────
    // Strategy: find the closest point on the OBB surface to the sphere centre.
    // If that distance < radius, we have a collision.
    //
    // All work is done in the Box's local space to keep the clamping simple,
    // then results are transformed back to world space.

    template <>
    struct CollisionAlgorithm<Sphere, Box>
    {
        static bool test(const Sphere &sphere, const m3d::tf &tf_sphere,
                         const Box &box, const m3d::tf &tf_box,
                         Contact &out)
        {
            // Sphere centre in box local space

            const m3d::vec3 local_centre = tf_box.inverse_rotate_vector(tf_sphere.pos - tf_box.pos);

            // Closest point on the box AABB (in local space) to the sphere centre
            const m3d::vec3 closest = m3d::vec3(
                m3d::clamp(local_centre.x, -box.half_extents.x, box.half_extents.x),
                m3d::clamp(local_centre.y, -box.half_extents.y, box.half_extents.y),
                m3d::clamp(local_centre.z, -box.half_extents.z, box.half_extents.z));

            const m3d::vec3 local_diff = local_centre - closest;
            const m3d::scalar dist = m3d::length(local_diff);

            // ── Case 1: centre is OUTSIDE the box ────────────────────────────────
            if (dist > m3d::EPSILON)
            {
                if (dist >= sphere.radius)
                    return false; // separated

                // Normal in local space, then rotate to world space
                const m3d::vec3 local_normal = local_diff / dist;
                out.normal = tf_box.rotate_vector(local_normal);
                out.penetration_depth = sphere.radius - dist;
                out.pos = tf_box.transform_point(closest);
                return true;
            }

            // ── Case 2: centre is INSIDE the box ─────────────────────────────────
            // Find the face with the smallest penetration — that's the exit direction.
            // Penetration along each face axis = half_extent - |local_coord|
            const m3d::scalar px = box.half_extents.x - m3d::abs(local_centre.x);
            const m3d::scalar py = box.half_extents.y - m3d::abs(local_centre.y);
            const m3d::scalar pz = box.half_extents.z - m3d::abs(local_centre.z);

            m3d::vec3 local_normal;

            if (px <= py && px <= pz)
                local_normal = m3d::vec3(local_centre.x > 0 ? 1.0 : -1.0, 0.0, 0.0);
            else if (py <= px && py <= pz)
                local_normal = m3d::vec3(0.0, local_centre.y > 0 ? 1.0 : -1.0, 0.0);
            else
                local_normal = m3d::vec3(0.0, 0.0, local_centre.z > 0 ? 1.0 : -1.0);

            const m3d::scalar min_pen = m3d::min(px, m3d::min(py, pz));

            out.normal = -tf_box.rotate_vector(local_normal); // normal points from box (A) to sphere (B) (the direction the sphere needs to move to exit)
            out.penetration_depth = sphere.radius + min_pen;
            out.pos = tf_sphere.pos - out.normal * sphere.radius; // deepest point of sphere
            return true;
        }
    };

    // ── Box vs Sphere — symmetric shim ───────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Box, Sphere>
        : CollisionAlgorithmSym<Box, Sphere>
    {
    };

} // namespace rbc