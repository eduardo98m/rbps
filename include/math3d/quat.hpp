#pragma once
#include "math3d/vec3.hpp"

namespace m3d
{   
    // Forward declarations
    struct quat;
    inline quat normalize(const quat &q);

    struct quat
    {
        // Layout: w is real part, v=(x,y,z) is imaginary part
        scalar w, x, y, z;

        quat() : w(1), x(0), y(0), z(0) {} // Identity
        quat(scalar _w, scalar _x, scalar _y, scalar _z) : w(_w), x(_x), y(_y), z(_z) {}

        // Construct from axis-angle
        static quat from_axis_angle(const vec3 &axis, scalar angle)
        {
            scalar half_angle = angle * 0.5;
            scalar s = std::sin(half_angle);
            vec3 n = normalize(axis);
            return {std::cos(half_angle), n.x * s, n.y * s, n.z * s};
        }

        static quat from_rpy(scalar roll, scalar pitch, scalar yaw)
        {
            scalar cr = std::cos(roll * 0.5);
            scalar sr = std::sin(roll * 0.5);
            scalar cp = std::cos(pitch * 0.5);
            scalar sp = std::sin(pitch * 0.5);
            scalar cy = std::cos(yaw * 0.5);
            scalar sy = std::sin(yaw * 0.5);

            return {
                cr * cp * cy + sr * sp * sy, // w
                sr * cp * cy - cr * sp * sy, // x
                cr * sp * cy + sr * cp * sy, // y
                cr * cp * sy - sr * sp * cy  // z
            };
        }

        // Quaternion Multiplication (Hamilton Product)
        quat operator*(const quat &r) const {
            quat res;
            res.w = w * r.w - x * r.x - y * r.y - z * r.z;
            res.x = w * r.x + x * r.w + y * r.z - z * r.y;
            res.y = w * r.y + y * r.w + z * r.x - x * r.z;
            res.z = w * r.z + z * r.w + x * r.y - y * r.x;
            return normalize(res);
        }

        // Scaling
        quat operator*(scalar s) const { return {w * s, x * s, y * s, z * s}; }

        // Addition (useful for integration)
        quat operator+(const quat &r) const { return {w + r.w, x + r.x, y + r.y, z + r.z}; }

        quat &operator+=(const quat &rhs)
        {
            w += rhs.w;
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        friend std::ostream &operator<<(std::ostream &os, const quat &q)
        {
            return os << "quat(w:" << q.w << ", x:" << q.x << ", y:" << q.y << ", z:" << q.z << ")";
        }

        // Inside struct quat
        bool operator==(const quat &other) const
        {
            return w == other.w && x == other.x && y == other.y && z == other.z;
        }

        bool is_approx(const quat &other, scalar precision = EPSILON) const
        {
            return std::abs(w - other.w) < precision &&
                   std::abs(x - other.x) < precision &&
                   std::abs(y - other.y) < precision &&
                   std::abs(z - other.z) < precision;
        }
    };

    inline quat normalize(const quat &q)
    {
        scalar mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        
        // Avoid the sqrt call 
        if (std::abs(1.0 - mag_sq) < 2.107342e-08) { // Typical float precision threshold
            return q * (2.0 / (1.0 + mag_sq));
        }
        
        if (mag_sq < EPSILON) return {1, 0, 0, 0}; // Avoid zero division
        
        return q * (1.0 / std::sqrt(mag_sq));
    }

    inline quat conjugate(const quat &q)
    {
        return {q.w, -q.x, -q.y, -q.z};
    }

    // Rotate vector v by quaternion q ( q * v * q' )
    // Optimized formula: v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
    inline vec3 rotate(const vec3 &v, const quat &q)
    {
        vec3 u(q.x, q.y, q.z);
        scalar s = q.w;

        vec3 uv = cross(u, v);
        vec3 uuv = cross(u, uv);

        return v + ((uv * s) + uuv) * 2.0;
    }
    
    inline vec3 rotate(const quat &q, const vec3 &v)
    {
        return rotate(v, q);
    }

} // namespace m3d