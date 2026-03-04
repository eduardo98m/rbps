#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    // ── Ellipsoid ─────────────────────────────────────────────────────────────
    // Axis-aligned ellipsoid in local space; rotation handled by m3d::tf.
    struct Ellipsoid
    {
        m3d::vec3 half_extents; // semi-axes (a, b, c) along local X, Y, Z

        Ellipsoid() : half_extents(0.5, 0.5, 0.5) {}
        explicit Ellipsoid(const m3d::vec3 &half_extents) : half_extents(half_extents) {}

        inline bool operator==(const Ellipsoid &o) const { return half_extents == o.half_extents; }
        inline bool operator!=(const Ellipsoid &o) const { return !(*this == o); }
    };

    // ── Support function ──────────────────────────────────────────────────────
    // For x²/a² + y²/b² + z²/c² ≤ 1, maximise d·x via Lagrange multipliers:
    //   p_k = a_k² · d_k  /  sqrt( a_x²·d_x² + a_y²·d_y² + a_z²·d_z² )
    inline m3d::vec3 support(const Ellipsoid &e, const m3d::vec3 &dir)
    {
        const m3d::vec3 &a = e.half_extents;
        const m3d::vec3 scaled(a.x * a.x * dir.x,
                               a.y * a.y * dir.y,
                               a.z * a.z * dir.z);
        const m3d::scalar denom = m3d::sqrt(a.x * a.x * dir.x * dir.x +
                                            a.y * a.y * dir.y * dir.y +
                                            a.z * a.z * dir.z * dir.z);
        if (denom < m3d::EPSILON)
            return m3d::vec3(a.x, 0.0, 0.0);
        return scaled / denom;
    }

    inline m3d::scalar compute_volume(const Ellipsoid &e)
    {
        return (4.0 / 3.0) * m3d::PI * e.half_extents.x * e.half_extents.y * e.half_extents.z;
    }

    // Ixx = (2/5)·m·(b² + c²), cyclic.
    inline m3d::smat3 compute_inertia_tensor(const Ellipsoid &e)
    {
        const m3d::scalar mass = compute_volume(e);
        const m3d::scalar ax2 = e.half_extents.x * e.half_extents.x;
        const m3d::scalar ay2 = e.half_extents.y * e.half_extents.y;
        const m3d::scalar az2 = e.half_extents.z * e.half_extents.z;
        return m3d::smat3(
            (2.0 / 5.0) * mass * (ay2 + az2),
            (2.0 / 5.0) * mass * (ax2 + az2),
            (2.0 / 5.0) * mass * (ax2 + ay2),
            0.0, 0.0, 0.0);
    }

    // ── Tight AABB for a rotated ellipsoid ────────────────────────────────────
    // World-space half-extent along axis i:
    //   extent_i = sqrt( R[0][i]²·ax² + R[1][i]²·ay² + R[2][i]²·az² )
    // (Same derivation as Box but with sqrt of sum-of-squares instead of sum of |.|.)
    inline AABB compute_aabb(const Ellipsoid &e, const m3d::tf &tf)
    {
        const m3d::mat3 R = m3d::mat3_cast(tf.rot);
        const m3d::vec3 &a = e.half_extents;
        const m3d::vec3 extent(
            m3d::sqrt(R[0][0] * R[0][0] * a.x * a.x + R[1][0] * R[1][0] * a.y * a.y + R[2][0] * R[2][0] * a.z * a.z),
            m3d::sqrt(R[0][1] * R[0][1] * a.x * a.x + R[1][1] * R[1][1] * a.y * a.y + R[2][1] * R[2][1] * a.z * a.z),
            m3d::sqrt(R[0][2] * R[0][2] * a.x * a.x + R[1][2] * R[1][2] * a.y * a.y + R[2][2] * R[2][2] * a.z * a.z));
        return {tf.pos - extent, tf.pos + extent};
    }
} // namespace rbc