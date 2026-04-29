#pragma once
#include <math3d/math3d.hpp>

/**
 * @file FaceHelpers.hpp
 * @brief Disc-approximation fallback for shapes without flat faces.
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Build a 4-corner polygon approximating the contact face at `support_pt`.
     *
     * Used as the `face_corners` fallback for convex shapes that don't have
     * a flat face on the contact normal (Sphere, Capsule, Ellipsoid, Cone).
     * The polygon is a square of side `radius` lying in the plane normal to
     * `normal` and centred on `support_pt`.
     *
     * Lives in this small header to break a cycle: every shape's
     * `face_corners` needs this helper, but `ContactManifoldGenerator.hpp`
     * (which is the natural home) depends on every shape.
     *
     * @param support_pt World-space point that the polygon is centred on.
     * @param normal     Unit world-space contact normal.
     * @param radius     Patch half-size (typically `representative_radius`).
     * @param[out] corners 4 polygon corners in world space.
     * @return Always 4.
     *
     * @ingroup rbc
     */
    inline int get_generic_face_corners(const m3d::vec3 &support_pt,
                                        const m3d::vec3 &normal,
                                        m3d::scalar radius,
                                        m3d::vec3 corners[4])
    {
        m3d::vec3 t1, t2;
        if (m3d::abs(normal.x) < 0.9)
            t1 = m3d::normalize(m3d::cross(normal, m3d::vec3(1, 0, 0)));
        else
            t1 = m3d::normalize(m3d::cross(normal, m3d::vec3(0, 1, 0)));
        t2 = m3d::cross(normal, t1);

        const m3d::scalar r = radius * 0.5;
        corners[0] = support_pt + (t1 + t2) * r;
        corners[1] = support_pt + (-t1 + t2) * r;
        corners[2] = support_pt + (-t1 - t2) * r;
        corners[3] = support_pt + (t1 - t2) * r;
        return 4;
    }
} // namespace rbc
