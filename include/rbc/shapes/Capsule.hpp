#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    // ── Capsule ───────────────────────────────────────────────────────────────
    // A cylinder of radius `radius` capped with two hemispheres.
    // In local space the principal axis is Y.
    //   Top hemisphere centre  = (0, +half_height, 0)
    //   Bottom hemisphere centre = (0, -half_height, 0)
    // Total height = 2*(half_height + radius).
    struct Capsule
    {
        m3d::scalar half_height; // distance from centre to each cap centre
        m3d::scalar radius;

        Capsule() : half_height(0.5), radius(0.25) {}
        Capsule(m3d::scalar half_height, m3d::scalar radius)
            : half_height(half_height), radius(radius) {}

        inline bool operator==(const Capsule &o) const
        { return half_height == o.half_height && radius == o.radius; }
        inline bool operator!=(const Capsule &o) const { return !(*this == o); }
    };

    // ── World-space endpoints (convenience, used by collision algorithms) ─────
    inline void capsule_endpoints(const Capsule &c, const m3d::tf &tf,
                                  m3d::vec3 &p1_out, m3d::vec3 &p2_out)
    {
        const m3d::vec3 axis = tf.rotate_vector(m3d::vec3(0, c.half_height, 0));
        p1_out = tf.pos + axis;
        p2_out = tf.pos - axis;
    }

    // ── Support function (local space, axis = local Y) ────────────────────────
    // Capsule = Minkowski sum of segment [(0,-h,0),(0,+h,0)] and sphere of radius r.
    // Support = endpoint in dir.y sign direction, then nudge by r·normalize(dir).
    inline m3d::vec3 support(const Capsule &c, const m3d::vec3 &dir)
    {
        const m3d::vec3 endpoint(0.0,
                                  (dir.y >= 0.0 ? c.half_height : -c.half_height),
                                  0.0);
        const m3d::scalar len = m3d::length(dir);
        if (len < m3d::EPSILON) return endpoint;
        return endpoint + (dir / len) * c.radius;
    }

    inline m3d::scalar compute_volume(const Capsule &c)
    {
        // Cylinder + two hemispheres
        const m3d::scalar r = c.radius, h = c.half_height;
        return m3d::PI * r * r * (2.0 * h + (4.0 / 3.0) * r);
    }

    // Inertia tensor about the capsule's local axes (unit density).
    inline m3d::smat3 compute_inertia_tensor(const Capsule &c)
    {
        // References: Game Physics, Eberly; moment integrals for capsule.
        const m3d::scalar r  = c.radius, h = c.half_height;
        const m3d::scalar r2 = r * r;
        const m3d::scalar h2 = h * h;
        const m3d::scalar mass = compute_volume(c); // unit density

        // Cylinder mass and hemisphere mass fractions
        const m3d::scalar m_cyl  = m3d::PI * r2 * 2.0 * h;
        const m3d::scalar m_hemi = mass - m_cyl; // = (4/3)*pi*r³ total for both caps

        // About the principal axis (Y): Iyy
        const m3d::scalar Iyy = (m_cyl * r2 / 2.0) +
                                  (m_hemi * 2.0 * r2 / 5.0);

        // About a transverse axis (X or Z): Ixx
        // Cylinder contribution
        const m3d::scalar Ixx_cyl = m_cyl * (r2 / 4.0 + h2 / 3.0);
        // Hemisphere contribution (using parallel axis theorem, d = h + 3r/8 from cap centre)
        const m3d::scalar d_hemi  = h + 3.0 * r / 8.0;
        const m3d::scalar Ixx_hemi = m_hemi * (2.0 * r2 / 5.0 + d_hemi * d_hemi);
        const m3d::scalar Ixx = Ixx_cyl + Ixx_hemi;

        return m3d::smat3(Ixx, Iyy, Ixx, 0.0, 0.0, 0.0);
    }

    // ── Tight AABB ────────────────────────────────────────────────────────────
    // World axis of the capsule, then union of two sphere-radius boxes at endpoints.
    inline AABB compute_aabb(const Capsule &c, const m3d::tf &tf)
    {
        const m3d::vec3 axis = tf.rotate_vector(m3d::vec3(0, c.half_height, 0));
        const m3d::vec3 p1   = tf.pos + axis;
        const m3d::vec3 p2   = tf.pos - axis;
        const m3d::vec3 r(c.radius, c.radius, c.radius);
        return {
            m3d::vec3(m3d::min(p1.x, p2.x), m3d::min(p1.y, p2.y), m3d::min(p1.z, p2.z)) - r,
            m3d::vec3(m3d::max(p1.x, p2.x), m3d::max(p1.y, p2.y), m3d::max(p1.z, p2.z)) + r
        };
    }
} // namespace rbc