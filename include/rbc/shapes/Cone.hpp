#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    // ── Cone ──────────────────────────────────────────────────────────────────
    // Right circular cone. In local space:
    //   Apex   = (0, +half_height, 0)
    //   Base centre = (0, -half_height, 0), radius = base_radius
    // Total height = 2*half_height.
    struct Cone
    {
        m3d::scalar half_height;
        m3d::scalar base_radius;

        Cone() : half_height(0.5), base_radius(0.5) {}
        Cone(m3d::scalar half_height, m3d::scalar base_radius)
            : half_height(half_height), base_radius(base_radius) {}

        inline bool operator==(const Cone &o) const
        {
            return half_height == o.half_height && base_radius == o.base_radius;
        }
        inline bool operator!=(const Cone &o) const { return !(*this == o); }
    };

    // ── Support function (local space) ────────────────────────────────────────
    // Candidates: apex (0, h, 0) or base-circle edge point.
    // We compare d·apex vs d·(base_point), pick the larger.
    //   apex_dot       = d.y · h
    //   base_edge_dot  = d_radial · base_radius − d.y · h
    // where d_radial = sqrt(d.x² + d.z²).
    inline m3d::vec3 support(const Cone &c, const m3d::vec3 &dir)
    {
        const m3d::scalar d_radial = m3d::sqrt(dir.x * dir.x + dir.z * dir.z);

        if (d_radial < m3d::EPSILON)
        {
            // Direction is purely axial
            return (dir.y >= 0.0)
                       ? m3d::vec3(0.0, c.half_height, 0.0)             // apex
                       : m3d::vec3(c.base_radius, -c.half_height, 0.0); // base edge (arbitrary angle)
        }

        const m3d::scalar apex_dot = dir.y * c.half_height;
        const m3d::scalar base_edge_dot = d_radial * c.base_radius - dir.y * c.half_height;

        if (apex_dot >= base_edge_dot)
            return m3d::vec3(0.0, c.half_height, 0.0); // apex wins

        // Base-circle edge point in the radial direction of dir
        const m3d::scalar scale = c.base_radius / d_radial;
        return m3d::vec3(dir.x * scale, -c.half_height, dir.z * scale);
    }

    inline m3d::scalar compute_volume(const Cone &c)
    {
        return (1.0 / 3.0) * m3d::PI * c.base_radius * c.base_radius * (2.0 * c.half_height);
    }

    // Inertia tensor about the cone's local axes (unit density).
    // For a solid cone of height H=2h, base radius R, mass m:
    //   Iyy (axial)     = (3/10) · m · R²
    //   Ixx (transverse) = (3/20) · m · (R² + H²/4) shifted to centroid
    // The centroid is at 3H/8 from the base = h/4 from the centre (positive Y side).
    inline m3d::smat3 compute_inertia_tensor(const Cone &c)
    {
        const m3d::scalar mass = compute_volume(c);
        const m3d::scalar R2 = c.base_radius * c.base_radius;
        const m3d::scalar H = 2.0 * c.half_height;
        const m3d::scalar Iyy = (3.0 / 10.0) * mass * R2;
        // Transverse about apex, then shift to centroid (h/4 from mid-point)
        const m3d::scalar Ixx_apex = (3.0 / 5.0) * mass * (R2 / 4.0 + H * H);
        const m3d::scalar d_centroid = 3.0 * H / 8.0 - c.half_height; // offset from local origin
        const m3d::scalar Ixx = Ixx_apex - mass * d_centroid * d_centroid;
        return m3d::smat3(Ixx, Iyy, Ixx, 0.0, 0.0, 0.0);
    }

    // ── Tight AABB via 6-direction support ────────────────────────────────────
    // More accurate than a bounding sphere, still O(1).
    inline AABB compute_aabb(const Cone &c, const m3d::tf &tf)
    {
        const m3d::vec3 world_axes[6] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

        m3d::vec3 mn = tf.pos, mx = tf.pos;
        for (const auto &axis : world_axes)
        {
            const m3d::vec3 local_dir = tf.inverse_rotate_vector(axis);
            const m3d::vec3 world_sup = tf.pos + tf.rotate_vector(support(c, local_dir));
            mn = m3d::vec3(m3d::min(mn.x, world_sup.x), m3d::min(mn.y, world_sup.y), m3d::min(mn.z, world_sup.z));
            mx = m3d::vec3(m3d::max(mx.x, world_sup.x), m3d::max(mx.y, world_sup.y), m3d::max(mx.z, world_sup.z));
        }
        return {mn, mx};
    }
} // namespace rbc