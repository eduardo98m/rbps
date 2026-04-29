#pragma once

#include "MinkowskiDiff.hpp"
#include <vector>

/**
 * @file GJK.hpp
 * @brief Gilbert–Johnson–Keerthi algorithm — overlap test for convex shapes.
 * @ingroup rbc
 * @ingroup internals
 *
 * GJK answers "do `A` and `B` overlap?" by searching for the origin
 * inside their Minkowski difference. The simplex (1–4 points) is grown
 * one support direction at a time; the algorithm terminates when either
 * the simplex encloses the origin (`Inside` — overlap) or no progress is
 * made toward the origin (`Valid` — separated).
 *
 * Reference: Ericson, *Real-Time Collision Detection*, §9.5.
 */

namespace rbc {

    /**
     * @brief One vertex of the GJK simplex.
     *
     * Stores both the original support points on `A` and `B` (`w0`, `w1`)
     * and their Minkowski difference (`w = w0 - w1`). EPA needs the
     * originals to recover a contact point in world space.
     *
     * @ingroup internals
     */
    struct SimplexVertex {
        m3d::vec3 w0; ///< World-space support point on shape A.
        m3d::vec3 w1; ///< World-space support point on shape B.
        m3d::vec3 w;  ///< Minkowski-difference point (`w0 - w1`).
    };

    /**
     * @brief Up-to-4-vertex simplex used by GJK.
     *
     * `rank` is the current vertex count (1 = point, 2 = line, 3 = triangle,
     * 4 = tetrahedron). Vertices are pointers into `GJK::store_v`.
     *
     * @ingroup internals
     */
    struct Simplex {
        SimplexVertex* vertex[4];
        int rank = 0; ///< Number of valid entries in `vertex`.
    };

    /**
     * @brief GJK solver state and entry point.
     *
     * Reusable: `evaluate` resets internal state on each call, so a single
     * `GJK` instance can be used for many pair tests.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    struct GJK {
        /**
         * @brief Outcome of `evaluate`.
         *
         * - `Valid`  — search converged with the simplex outside the origin
         *              (shapes are separated; `ray` and `distance` are valid).
         * - `Inside` — the origin lies in the Minkowski difference (overlap;
         *              the simplex is the seed for EPA).
         * - `Failed` — numerical breakdown or iteration limit hit.
         */
        enum Status { Valid, Inside, Failed } status;

        unsigned int max_iterations = 128; ///< Iteration cap to bound worst-case cost.
        m3d::scalar tolerance = 1e-6;      ///< Convergence threshold on the squared distance.

        const MinkowskiDiff* current_shape; ///< Shape pair under evaluation.
        m3d::vec3 ray;        ///< Closest approach direction; valid when `status == Valid`.
        m3d::scalar distance; ///< Magnitude of `ray`.

        Simplex simplices[2];        ///< Double-buffered simplex (current + next).
        SimplexVertex store_v[4];    ///< Pre-allocated vertex pool.
        SimplexVertex* free_v[4];    ///< Stack of free slots in `store_v`.
        int nfree;                   ///< Number of free vertex slots.
        int current;                 ///< Index of the active buffer in `simplices`.
        Simplex* active_simplex;     ///< Pointer to `simplices[current]`.

        /** @brief Construct a fresh solver. */
        GJK();

        /**
         * @brief Run GJK on the given Minkowski difference.
         *
         * @param shape         The Minkowski-difference proxy.
         * @param initial_guess Starting search direction (typically `tf_b.pos - tf_a.pos`).
         * @return One of `Valid`, `Inside`, `Failed`.
         */
        Status evaluate(const MinkowskiDiff& shape, const m3d::vec3& initial_guess);

        /** @brief Reset the simplex pool and counters before a new evaluation. @ingroup internals */
        void initialize();
        /** @brief Push a new vertex onto the simplex from a Minkowski-diff point. @ingroup internals */
        void append_vertex(Simplex& s, const m3d::vec3& v);
        /** @brief Pop the most recently appended vertex from the simplex. @ingroup internals */
        void remove_vertex(Simplex& s);

        /** @brief Special case: tetrahedron simplex contains the origin (overlap). @ingroup internals */
        bool enclose_origin();
        /** @brief Voronoi projection of the origin onto a 1-simplex (line). @ingroup internals */
        bool project_line_origin(const Simplex& curr, Simplex& next);
        /** @brief Voronoi projection of the origin onto a 2-simplex (triangle). @ingroup internals */
        bool project_triangle_origin(const Simplex& curr, Simplex& next);
        /** @brief Voronoi projection of the origin onto a 3-simplex (tetrahedron). @ingroup internals */
        bool project_tetrahedron_origin(const Simplex& curr, Simplex& next);
    };

} // namespace rbc
