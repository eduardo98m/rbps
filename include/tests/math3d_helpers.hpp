#pragma once
#include <math3d/math3d.hpp>

// ── math3d test helpers ─────────────────────────────────────────────────────
// Small utilities used across the math3d test suite.
//   - identity_quat()       : the 1+0i+0j+0k quaternion
//   - axis_quat(axis, theta): quaternion for a rotation about an axis
//   - smat3_to_mat3(s)      : expand a symmetric 3x3 to a full mat3 (used in
//                             test_mat3.cpp's smat3-vs-mat3 comparisons)
namespace test
{
    inline m3d::quat identity_quat() { return m3d::quat(1.0, 0.0, 0.0, 0.0); }

    inline m3d::quat axis_quat(const m3d::vec3 &axis, m3d::scalar angle_rad)
    {
        return m3d::quat::from_axis_angle(axis, angle_rad);
    }

    inline m3d::mat3 smat3_to_mat3(const m3d::smat3 &s)
    {
        return m3d::mat3(
            s.xx, s.xy, s.xz,
            s.xy, s.yy, s.yz,
            s.xz, s.yz, s.zz);
    }
} // namespace test
