#pragma once
#include <cmath>
#include <limits>
#include <cassert>
#include <iostream>

namespace m3d
{
    using scalar = double;
    constexpr scalar PI = 3.14159265358979323846;
    constexpr scalar EPSILON = 1e-9;           // std::numeric_limits<scalar>::epsilon();
    constexpr scalar ERROR_ULP = 2.107342e-08; //  unit of least precision (or Unit in the last place) for doubles

    template <typename T>
    inline T clamp(T v, T lo, T hi)
    {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    inline scalar to_radians(scalar degrees)
    {
        return degrees * (PI / 180.0);
    }

    inline scalar abs(scalar x)
    {
        return std::abs(x);
    }

    inline scalar sqrt(scalar x)
    {
        return std::sqrt(x);
    }

    inline scalar sin(scalar x)
    {
        return std::sin(x);
    }
    
    inline scalar cos(scalar x)
    {
        return std::cos(x);
    }
    
    inline scalar atan2(scalar y, scalar x)
    {
        return std::atan2(y, x);
    }
}