#pragma once
#include "math3d/core.hpp"

namespace m3d
{

    struct vec3
    {
        scalar x, y, z;

        // Constructors
        vec3() : x(0), y(0), z(0) {}
        vec3(scalar _x, scalar _y, scalar _z) : x(_x), y(_y), z(_z) {}

        // Operator Overloading for clean syntax
        vec3 operator+(const vec3 &rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
        vec3 operator-(const vec3 &rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
        vec3 operator*(scalar s) const { return {x * s, y * s, z * s}; }
        vec3 operator/(scalar s) const
        {
            scalar inv = 1.0 / s;
            return {x * inv, y * inv, z * inv};
        }

        // Unary negation
        vec3 operator-() const { return {-x, -y, -z}; }

        // In-place operators (important for performance)
        vec3 &operator+=(const vec3 &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        vec3 &operator-=(const vec3 &rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        vec3 &operator*=(scalar s)
        {
            x *= s;
            y *= s;
            z *= s;
            return *this;
        }

        bool operator==(const vec3 &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const vec3 &other) const
        {
            return !(*this == other);
        }

        // For physics-safe comparison
        bool is_approx(const vec3 &other, scalar precision = EPSILON) const
        {
            return std::abs(x - other.x) < precision &&
                   std::abs(y - other.y) < precision &&
                   std::abs(z - other.z) < precision;
        }

        // Meber acces via indices
        scalar &operator[](int i)
        {
            return (i == 0) ? x : (i == 1) ? y
                                           : z;
        }

        const scalar &operator[](int i) const
        {
            return (i == 0) ? x : (i == 1) ? y
                                           : z;
        }

        friend std::ostream &operator<<(std::ostream &os, const vec3 &v)
        {
            return os << "vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
        }
    };

    // Left-side scalar multiplication (s * v)
    inline vec3 operator*(scalar s, const vec3 &v) { return {v.x * s, v.y * s, v.z * s}; }

    // --- Vector Functions ---

    inline scalar dot(const vec3 &a, const vec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline vec3 cross(const vec3 &a, const vec3 &b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    }

    inline scalar length_sq(const vec3 &v)
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    inline scalar length(const vec3 &v)
    {
        return std::sqrt(length_sq(v));
    }

    // Alias for magnitude (Its intuitive to call it like that in some contexts)
    inline scalar magnitude(const vec3 &v)
    {
        return length(v);
    }

    inline vec3 normalize(const vec3 &v)
    {
        scalar len_sq = length_sq(v);
        if (len_sq < EPSILON)
            return {0, 0, 0};
        scalar inv_len = 1.0 / std::sqrt(len_sq);
        return v * inv_len;
    }

} // namespace math