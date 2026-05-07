#pragma once

#include "MinkowskiDiff.hpp"
#include "rbc/gjk/simplex/Simplex.hpp"

/**
 * @file GJK.hpp
 * @brief Gilbert–Johnson–Keerthi algorithm — overlap test for convex shapes.
 * @ingroup rbc
 *
 * GJK answers "do A and B overlap?" by searching for the origin inside
 * their Minkowski difference. The simplex (1–4 points) is grown one
 * support direction at a time; the algorithm terminates when either the
 * simplex encloses the origin (`Inside` — overlap) or no progress is
 * made toward the origin (`Valid` — separated).
 *
 * The simplex storage and per-rank handlers live under
 * `include/rbc/gjk/simplex/` and `src/rbc/gjk/simplex/`.
 */

namespace rbc {

    struct GJK {
        enum Status { Valid, Inside, Failed } status;

        unsigned int max_iterations = 128;
        m3d::scalar  tolerance      = 1e-6;

        const MinkowskiDiff *current_shape;
        m3d::vec3   ray;       ///< Last search direction; for the debugger.
        m3d::scalar distance;  ///< |ray|; meaningful only when status == Valid.

        Simplex  simplex;
        Simplex *active_simplex;

        GJK();
        Status evaluate(const MinkowskiDiff &shape, const m3d::vec3 &initial_guess);
    };

} // namespace rbc
