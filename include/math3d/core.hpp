#pragma once
#include <cmath>
#include <limits>
#include <cassert>
#include <iostream>

#ifdef PI
#  undef PI
#endif

/**
 * @defgroup math3d math3d — 3D math primitives
 * @brief Foundational vector, matrix, quaternion, and transform types.
 *
 * Header-only. All types live in the `m3d` namespace and use a single
 * floating-point alias (`scalar = double`) so the engine's precision can
 * be swapped in one place.
 */

/**
 * @file core.hpp
 * @brief Scalar alias, math constants, and thin wrappers around `<cmath>`.
 * @ingroup math3d
 */

namespace m3d
{
    /**
     * @brief Default floating-point type for the whole engine.
     *
     * Aliased here so a future port to `float` (or to a fixed-point type
     * for embedded targets) is a single-line change.
     *
     * @ingroup math3d
     */
    using scalar = double;

    /** @brief Pi. @ingroup math3d */
    constexpr scalar PI = 3.14159265358979323846;

    /**
     * @brief Generic small-number tolerance used by `is_approx` helpers.
     *
     * Set wider than `std::numeric_limits<scalar>::epsilon()` so it absorbs
     * the round-off accumulated during physics integration.
     *
     * @ingroup math3d
     */
    constexpr scalar EPSILON = 1e-9;

    /**
     * @brief Unit-in-the-Last-Place threshold for `double`.
     *
     * Used by `quat::normalize` to skip the `sqrt` when the magnitude is
     * already within one ULP of 1.
     *
     * @ingroup math3d
     */
    constexpr scalar ERROR_ULP = 2.107342e-08;

    /**
     * @brief Clamp `v` into `[lo, hi]`.
     *
     * @tparam T Any orderable type.
     * @param v   Value to clamp.
     * @param lo  Lower bound (inclusive).
     * @param hi  Upper bound (inclusive).
     * @return    `lo` if `v < lo`, `hi` if `v > hi`, otherwise `v`.
     *
     * @ingroup math3d
     */
    template <typename T>
    inline T clamp(T v, T lo, T hi)
    {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    /**
     * @brief Convert an angle from degrees to radians.
     * @param degrees Angle in degrees.
     * @return Angle in radians.
     * @ingroup math3d
     */
    inline scalar to_radians(scalar degrees)
    {
        return degrees * (PI / 180.0);
    }

    /** @brief `std::min` for `scalar`. @ingroup math3d */
    inline scalar min(scalar x, scalar y) { return std::min(x, y); }

    /** @brief `std::max` for `scalar`. @ingroup math3d */
    inline scalar max(scalar x, scalar y) { return std::max(x, y); }

    /** @brief `std::abs` for `scalar`. @ingroup math3d */
    inline scalar abs(scalar x) { return std::abs(x); }

    /** @brief `std::sqrt` for `scalar`. @ingroup math3d */
    inline scalar sqrt(scalar x) { return std::sqrt(x); }

    /** @brief `std::sin` for `scalar`. @ingroup math3d */
    inline scalar sin(scalar x) { return std::sin(x); }

    /** @brief `std::cos` for `scalar`. @ingroup math3d */
    inline scalar cos(scalar x) { return std::cos(x); }

    /** @brief `std::atan2` for `scalar`. @ingroup math3d */
    inline scalar atan2(scalar y, scalar x) { return std::atan2(y, x); }
}
