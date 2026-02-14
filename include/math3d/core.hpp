#pragma once
#include <cmath>
#include <limits>
#include <cassert>
#include <iostream>

using scalar = double;
namespace m3d {
    constexpr scalar PI = 3.14159265358979323846;
    constexpr scalar EPSILON = 1e-9; //std::numeric_limits<scalar>::epsilon();

    template <typename T>
    inline T clamp(T v, T lo, T hi) {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    inline scalar to_radians(scalar degrees) {
        return degrees * (PI / 180.0);
    }
}