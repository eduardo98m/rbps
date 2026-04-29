#pragma once
#include "math3d/core.hpp"

/**
 * @file vec3.hpp
 * @brief 3-component floating-point vector and inline operators.
 * @ingroup math3d
 */

namespace m3d
{

    /**
     * @brief 3-component vector of `scalar` (cartesian x, y, z).
     *
     * The workhorse vector type. Used for positions, velocities, forces,
     * torques, axes, normals — anywhere a 3D direction or magnitude is
     * needed.
     *
     * Supports the usual arithmetic operators (`+ - * /`), in-place forms
     * (`+= -= *=`), unary negation, equality, indexed access, and a
     * tolerance-based `is_approx` for physics comparisons. Free functions
     * provide `dot`, `cross`, `length`, `length_sq`, `normalize`.
     *
     * @code
     * vec3 a{1, 2, 3};
     * vec3 b{0, 1, 0};
     * scalar c = dot(a, b);            // 2
     * vec3 n = normalize(cross(a, b)); // unit perpendicular
     * @endcode
     *
     * @ingroup math3d
     */
    struct vec3
    {
        scalar x, y, z;

        /** @brief Zero vector. */
        vec3() : x(0), y(0), z(0) {}
        /** @brief Component-wise constructor. */
        vec3(scalar _x, scalar _y, scalar _z) : x(_x), y(_y), z(_z) {}
        /** @brief Splat constructor: all three components set to `_d`. */
        vec3(scalar _d) : x(_d), y(_d), z(_d) {}

        /** @brief Component-wise addition. */
        vec3 operator+(const vec3 &rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
        /** @brief Component-wise subtraction. */
        vec3 operator-(const vec3 &rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
        /** @brief Scalar multiplication. */
        vec3 operator*(scalar s) const { return {x * s, y * s, z * s}; }
        /** @brief Scalar division (multiplies by reciprocal once). */
        vec3 operator/(scalar s) const
        {
            scalar inv = 1.0 / s;
            return {x * inv, y * inv, z * inv};
        }

        /** @brief Unary negation. */
        vec3 operator-() const { return {-x, -y, -z}; }

        /** @brief In-place addition. */
        vec3 &operator+=(const vec3 &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        /** @brief In-place subtraction. */
        vec3 &operator-=(const vec3 &rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        /** @brief In-place scalar multiplication. */
        vec3 &operator*=(scalar s)
        {
            x *= s;
            y *= s;
            z *= s;
            return *this;
        }

        /** @brief Bitwise-exact equality. Use `is_approx` for physics comparisons. */
        bool operator==(const vec3 &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }

        /** @brief Bitwise-exact inequality. */
        bool operator!=(const vec3 &other) const
        {
            return !(*this == other);
        }

        /**
         * @brief Tolerance-based equality, suitable for physics state.
         *
         * Returns true iff every component differs by less than `precision`.
         *
         * @param other     Vector to compare against.
         * @param precision Absolute per-component tolerance (default `EPSILON`).
         */
        bool is_approx(const vec3 &other, scalar precision = EPSILON) const
        {
            return m3d::abs(x - other.x) < precision &&
                   m3d::abs(y - other.y) < precision &&
                   m3d::abs(z - other.z) < precision;
        }

        /** @brief Indexed mutable access (0=x, 1=y, 2=z). */
        scalar &operator[](int i)
        {
            return (i == 0) ? x : (i == 1) ? y
                                           : z;
        }

        /** @brief Indexed const access (0=x, 1=y, 2=z). */
        const scalar &operator[](int i) const
        {
            return (i == 0) ? x : (i == 1) ? y
                                           : z;
        }

        /** @brief Stream insertion as `vec3(x, y, z)`. */
        friend std::ostream &operator<<(std::ostream &os, const vec3 &v)
        {
            return os << "vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
        }
    };

    /** @brief Left-side scalar multiplication (`s * v`). @ingroup math3d */
    inline vec3 operator*(scalar s, const vec3 &v) { return {v.x * s, v.y * s, v.z * s}; }

    /**
     * @brief Dot (inner) product of two vectors.
     * @return `a.x*b.x + a.y*b.y + a.z*b.z`.
     * @ingroup math3d
     */
    inline scalar dot(const vec3 &a, const vec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * @brief Cross product (`a × b`), right-handed.
     * @return Vector perpendicular to both `a` and `b`.
     * @ingroup math3d
     */
    inline vec3 cross(const vec3 &a, const vec3 &b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    }

    /**
     * @brief Squared length of `v`. Cheaper than `length` (no `sqrt`).
     *
     * Prefer this for length comparisons (`length_sq(a) < r*r`).
     *
     * @ingroup math3d
     */
    inline scalar length_sq(const vec3 &v)
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    /** @brief Euclidean length of `v`. @ingroup math3d */
    inline scalar length(const vec3 &v)
    {
        return m3d::sqrt(length_sq(v));
    }

    /** @brief Alias of `length`, more natural in some physics contexts. @ingroup math3d */
    inline scalar magnitude(const vec3 &v)
    {
        return length(v);
    }

    /**
     * @brief Return `v / length(v)`, or the zero vector if `v` is near zero.
     *
     * The zero-length fallback prevents division by zero; callers that need
     * to detect a degenerate input should test `length_sq(v) < EPSILON`
     * themselves before calling.
     *
     * @ingroup math3d
     */
    inline vec3 normalize(const vec3 &v)
    {
        scalar len_sq = length_sq(v);
        if (len_sq < EPSILON)
            return {0, 0, 0};
        scalar inv_len = 1.0 / m3d::sqrt(len_sq);
        return v * inv_len;
    }

} // namespace m3d
