#pragma once
#include <math3d/math3d.hpp>

namespace rbc
{
    // Disc-approximation fallback used by face_corners() of convex shapes that
    // do not have a flat face (Sphere, Capsule, Ellipsoid, Cone). Lives here to
    // avoid a cycle: ContactManifoldGenerator.hpp depends on every shape, and
    // each shape's face_corners() needs this helper.
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
