#pragma once
#include "math3d/core.hpp"

/**
 * @file vec2i.hpp
 * @brief 2-component integer vector — used for grid coordinates and pixel indices.
 * @ingroup math3d
 */

namespace m3d
{

    /**
     * @brief 2-component vector of `int` (cartesian x, y).
     *
     * Integer companion to `vec3`. Used where exactness matters and
     * floating-point round-off is unwelcome — for example, broad-phase
     * grid cell coordinates, heightmap indices, and screen pixels.
     *
     * Note that `cross(a, b)` returns a *scalar* (the signed z-component
     * of the equivalent 3D cross), as is conventional in 2D.
     *
     * @ingroup math3d
     */
    struct vec2i
    {
        int x, y;

        /** @brief Zero vector. */
        vec2i() : x(0), y(0) {}
        /** @brief Component-wise constructor. */
        vec2i(int _x, int _y) : x(_x), y(_y) {}
        /** @brief Splat constructor: both components set to `_d`. */
        vec2i(int _d) : x(_d), y(_d) {}

        /** @brief Component-wise addition. */
        vec2i operator+(const vec2i &rhs) const { return {x + rhs.x, y + rhs.y}; }
        /** @brief Component-wise subtraction. */
        vec2i operator-(const vec2i &rhs) const { return {x - rhs.x, y - rhs.y}; }
        /** @brief Scalar multiplication. */
        vec2i operator*(int s) const { return {x * s, y * s}; }
        /** @brief Integer scalar division (truncating). */
        vec2i operator/(int s) const
        {
            return {x / s, y / s};
        }

        /** @brief Unary negation. */
        vec2i operator-() const { return {-x, -y}; }

        /** @brief In-place addition. */
        vec2i &operator+=(const vec2i &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        /** @brief In-place subtraction. */
        vec2i &operator-=(const vec2i &rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        /** @brief In-place scalar multiplication. */
        vec2i &operator*=(int s)
        {
            x *= s;
            y *= s;
            return *this;
        }

        /** @brief Equality. */
        bool operator==(const vec2i &other) const
        {
            return x == other.x && y == other.y;
        }

        /** @brief Inequality. */
        bool operator!=(const vec2i &other) const
        {
            return !(*this == other);
        }

        /** @brief Indexed mutable access (0=x, 1=y). */
        int &operator[](int i)
        {
            return (i == 0) ? x : y;
        }

        /** @brief Indexed const access (0=x, 1=y). */
        const int &operator[](int i) const
        {
            return (i == 0) ? x : y;
        }

        /** @brief Stream insertion as `vec2i(x, y)`. */
        friend std::ostream &operator<<(std::ostream &os, const vec2i &v)
        {
            return os << "vec2i(" << v.x << ", " << v.y << ")";
        }
    };

    /** @brief Left-side scalar multiplication (`s * v`). @ingroup math3d */
    inline vec2i operator*(int s, const vec2i &v) { return {v.x * s, v.y * s}; }

    /**
     * @brief Dot (inner) product of two 2D integer vectors.
     * @return `a.x*b.x + a.y*b.y`.
     * @ingroup math3d
     */
    inline int dot(const vec2i &a, const vec2i &b)
    {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * @brief 2D cross product (signed scalar).
     *
     * Equivalent to the z-component of `cross(vec3(a.x,a.y,0), vec3(b.x,b.y,0))`.
     * Positive when `b` is counter-clockwise from `a`, negative when clockwise.
     *
     * @ingroup math3d
     */
    inline int cross(const vec2i &a, const vec2i &b)
    {
        return a.x * b.y - a.y * b.x;
    }

} // namespace m3d
