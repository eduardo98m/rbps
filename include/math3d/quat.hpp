#pragma once
#include "math3d/vec3.hpp"

/**
 * @file quat.hpp
 * @brief Unit quaternion type for rigid-body orientation.
 * @ingroup math3d
 */

namespace m3d
{
    struct quat;
    inline quat normalize(const quat &q);

    /**
     * @brief Quaternion `q = w + xi + yj + zk`, used to represent 3D rotations.
     *
     * Hamilton convention: `w` is the real (scalar) part, `(x, y, z)` is the
     * imaginary (vector) part. Identity rotation is `(w=1, x=y=z=0)`.
     *
     * Most physics code keeps quaternions normalised (unit length); use
     * `normalize` after a sequence of multiplications to remove drift.
     *
     * @code
     * // 90° rotation about +Y, applied to a vector along +X.
     * quat q = quat::from_axis_angle({0,1,0}, PI * 0.5);
     * vec3 r = rotate(q, {1, 0, 0});   // r ≈ (0, 0, -1)
     * @endcode
     *
     * @ingroup math3d
     */
    struct quat
    {
        scalar w, x, y, z;

        /** @brief Identity quaternion `(w=1, x=y=z=0)`. */
        quat() : w(1), x(0), y(0), z(0) {}
        /** @brief Component-wise constructor. */
        quat(scalar _w, scalar _x, scalar _y, scalar _z) : w(_w), x(_x), y(_y), z(_z) {}

        /**
         * @brief Build a quaternion from an axis–angle rotation.
         *
         * The axis is normalised internally; passing a near-zero axis returns
         * a quaternion with the imaginary part collapsed to zero.
         *
         * @param axis  Rotation axis (any non-zero length).
         * @param angle Rotation magnitude in radians (right-hand rule).
         */
        static quat from_axis_angle(const vec3 &axis, scalar angle)
        {
            scalar half_angle = angle * 0.5;
            scalar s = m3d::sin(half_angle);
            vec3 n = normalize(axis);
            return {m3d::cos(half_angle), n.x * s, n.y * s, n.z * s};
        }

        /**
         * @brief Build a quaternion from intrinsic roll–pitch–yaw (Tait-Bryan, XYZ order).
         *
         * @param roll  Rotation about local X (radians).
         * @param pitch Rotation about local Y (radians).
         * @param yaw   Rotation about local Z (radians).
         */
        static quat from_rpy(scalar roll, scalar pitch, scalar yaw)
        {
            scalar cr = m3d::cos(roll * 0.5);
            scalar sr = m3d::sin(roll * 0.5);
            scalar cp = m3d::cos(pitch * 0.5);
            scalar sp = m3d::sin(pitch * 0.5);
            scalar cy = m3d::cos(yaw * 0.5);
            scalar sy = m3d::sin(yaw * 0.5);

            return {
                cr * cp * cy + sr * sp * sy, // w
                sr * cp * cy - cr * sp * sy, // x
                cr * sp * cy + sr * cp * sy, // y
                cr * cp * sy - sr * sp * cy  // z
            };
        }

        /**
         * @brief Hamilton product `(*this) * r`.
         *
         * Composes two rotations: applying `(*this) * r` to a vector is
         * equivalent to applying `r` first, then `*this`.
         *
         * @note The result is NOT renormalised; call `normalize` periodically.
         */
        quat operator*(const quat &r) const
        {
            quat res;
            res.w = w * r.w - x * r.x - y * r.y - z * r.z;
            res.x = w * r.x + x * r.w + y * r.z - z * r.y;
            res.y = w * r.y + y * r.w + z * r.x - x * r.z;
            res.z = w * r.z + z * r.w + x * r.y - y * r.x;
            return res;
        }

        /** @brief Component-wise scaling (used for time-step integration). */
        quat operator*(scalar s) const { return {w * s, x * s, y * s, z * s}; }

        /** @brief Component-wise addition (used for time-step integration). */
        quat operator+(const quat &r) const { return {w + r.w, x + r.x, y + r.y, z + r.z}; }

        /** @brief In-place component-wise addition. */
        quat &operator+=(const quat &rhs)
        {
            w += rhs.w;
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        /** @brief Stream insertion as `quat(w:..., x:..., y:..., z:...)`. */
        friend std::ostream &operator<<(std::ostream &os, const quat &q)
        {
            return os << "quat(w:" << q.w << ", x:" << q.x << ", y:" << q.y << ", z:" << q.z << ")";
        }

        /** @brief Bitwise-exact equality. Use `is_approx` for physics state. */
        bool operator==(const quat &other) const
        {
            return w == other.w && x == other.x && y == other.y && z == other.z;
        }

        /**
         * @brief Tolerance-based equality.
         * @param other     Quaternion to compare against.
         * @param precision Absolute per-component tolerance (default `EPSILON`).
         */
        bool is_approx(const quat &other, scalar precision = EPSILON) const
        {
            return m3d::abs(w - other.w) < precision &&
                   m3d::abs(x - other.x) < precision &&
                   m3d::abs(y - other.y) < precision &&
                   m3d::abs(z - other.z) < precision;
        }
    };

    /**
     * @brief Renormalise `q` so its magnitude is 1.
     *
     * If the magnitude is already within one ULP of 1, applies the cheap
     * Padé approximation `q * (2 / (1 + |q|²))` instead of a `sqrt`.
     * If `q` is degenerate (near zero) the identity quaternion is returned.
     *
     * @ingroup math3d
     */
    inline quat normalize(const quat &q)
    {
        scalar mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

        if (m3d::abs(1.0 - mag_sq) < 2.107342e-08)
        {
            return q * (2.0 / (1.0 + mag_sq));
        }

        if (mag_sq < EPSILON)
            return {1, 0, 0, 0};

        return q * (1.0 / m3d::sqrt(mag_sq));
    }

    /**
     * @brief Quaternion conjugate `(w, -x, -y, -z)`.
     *
     * For unit quaternions, this is also the inverse: `q * conjugate(q) ≈ identity`.
     *
     * @ingroup math3d
     */
    inline quat conjugate(const quat &q)
    {
        return {q.w, -q.x, -q.y, -q.z};
    }

    /**
     * @brief Rotate vector `v` by quaternion `q`.
     *
     * Computes `q * v * q⁻¹` via the optimised cross-product form:
     * `v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)`.
     *
     * @ingroup math3d
     */
    inline vec3 rotate(const vec3 &v, const quat &q)
    {
        vec3 u(q.x, q.y, q.z);
        scalar s = q.w;

        vec3 uv = cross(u, v);
        vec3 uuv = cross(u, uv);

        return v + ((uv * s) + uuv) * 2.0;
    }

    /** @brief Argument-flipped overload; same result as `rotate(v, q)`. @ingroup math3d */
    inline vec3 rotate(const quat &q, const vec3 &v)
    {
        return rotate(v, q);
    }

    /**
     * @brief Convert a unit quaternion to roll–pitch–yaw (Tait-Bryan).
     *
     * Pitch is clamped to ±90° at the gimbal-lock singularity to avoid
     * NaN from `asin` on slightly out-of-range values.
     *
     * @return `vec3{roll, pitch, yaw}` in radians.
     * @ingroup math3d
     */
    inline vec3 to_rpy(const quat &q)
    {
        quat n = normalize(q);

        scalar sinr_cosp = 2.0 * (n.w * n.x + n.y * n.z);
        scalar cosr_cosp = 1.0 - 2.0 * (n.x * n.x + n.y * n.y);
        scalar roll = m3d::atan2(sinr_cosp, cosr_cosp);

        scalar sinp = 2.0 * (n.w * n.y - n.z * n.x);

        scalar pitch;
        if (m3d::abs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2.0, sinp);
        else
            pitch = std::asin(sinp);

        scalar siny_cosp = 2.0 * (n.w * n.z + n.x * n.y);
        scalar cosy_cosp = 1.0 - 2.0 * (n.y * n.y + n.z * n.z);
        scalar yaw = m3d::atan2(siny_cosp, cosy_cosp);

        return {roll, pitch, yaw};
    }

} // namespace m3d
