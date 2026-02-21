#pragma once
#include "math3d/core.hpp"

namespace m3d
{

    struct vec2i
    {
        int x, y;

        // Constructors
        vec2i() : x(0), y(0) {}
        vec2i(int _x, int _y) : x(_x), y(_y) {}
        vec2i(int _d) : x(_d), y(_d) {}

        // Operator Overloading for clean syntax
        vec2i operator+(const vec2i &rhs) const { return {x + rhs.x, y + rhs.y}; }
        vec2i operator-(const vec2i &rhs) const { return {x - rhs.x, y - rhs.y}; }
        vec2i operator*(int s) const { return {x * s, y * s}; }
        vec2i operator/(int s) const
        {
            return {x / s, y / s};
        }

        // Unary negation
        vec2i operator-() const { return {-x, -y}; }
        // In-place operators (important for performance)
        vec2i &operator+=(const vec2i &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        vec2i &operator-=(const vec2i &rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        vec2i &operator*=(int s)
        {
            x *= s;
            y *= s;
            return *this;
        }

        bool operator==(const vec2i &other) const
        {
            return x == other.x && y == other.y;
        }

        bool operator!=(const vec2i &other) const
        {
            return !(*this == other);
        }

        // Meber acces via indices
        int &operator[](int i)
        {
            return (i == 0) ? x : y;
        }

        const int &operator[](int i) const
        {
            return (i == 0) ? x : y;
        }

        friend std::ostream &operator<<(std::ostream &os, const vec2i &v)
        {
            return os << "vec2i(" << v.x << ", " << v.y << ")";
        }
    };

    // Left-side scalar multiplication (s * v)
    inline vec2i operator*(int s, const vec2i &v) { return {v.x * s, v.y * s}; }

    // --- Vector Functions ---

    inline int dot(const vec2i &a, const vec2i &b)
    {
        return a.x * b.x + a.y * b.y;
    }

    inline int cross(const vec2i &a, const vec2i &b)
    {
        return a.x * b.y - a.y * b.x;
    }
    

} // namespace m3d