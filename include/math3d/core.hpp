#pragma once
#include <cmath>
#include <limits>
#include <cassert>
#include <iostream>

namespace m3d {
    using scalar = double;
    constexpr scalar PI = 3.14159265358979323846;
    constexpr scalar EPSILON = 1e-9; //std::numeric_limits<scalar>::epsilon();
    constexpr scalar ERROR_ULP = 2.107342e-08; //  unit of least precision (or Unit in the last place) for doubles

    template <typename T>
    inline T clamp(T v, T lo, T hi) {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    inline scalar to_radians(scalar degrees) {
        return degrees * (PI / 180.0);
    }
}