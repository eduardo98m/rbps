#pragma once
#include "rbc/shapes/ShapeTypes.hpp"
#include <math3d/math3d.hpp>

/**
 * @file MinkowskiDiff.hpp
 * @brief Implicit Minkowski difference `A ⊖ B` used by GJK and EPA.
 * @ingroup rbc
 * @ingroup internals
 */

namespace rbc
{
    /**
     * @brief Minkowski difference of two shapes, accessed only through its support function.
     *
     * `A ⊖ B = { p - q | p ∈ A, q ∈ B }`. The set itself is never built —
     * GJK and EPA only need its support function (`support(dir)` returns
     * the vertex of `A ⊖ B` farthest along `dir`), which factors as
     * `support_A(dir) - support_B(-dir)` in world space.
     *
     * Two shapes overlap iff the origin lies inside their Minkowski
     * difference; that is the question GJK answers.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    struct MinkowskiDiff
    {
        const Shape *shape_a; ///< Non-owning pointer to shape A.
        const Shape *shape_b; ///< Non-owning pointer to shape B.
        m3d::tf tf_a;         ///< World transform of A.
        m3d::tf tf_b;         ///< World transform of B.

        /** @brief Build a Minkowski-difference proxy for the given shape pair. */
        MinkowskiDiff(const Shape *a, const Shape *b,
                      const m3d::tf &ta, const m3d::tf &tb)
            : shape_a(a), shape_b(b), tf_a(ta), tf_b(tb) {}

        /**
         * @brief World-space support of `A ⊖ B` along `dir`.
         *
         * Computes `support_A(dir) - support_B(-dir)` in world coordinates,
         * routing each call through the per-shape variant dispatcher
         * (`shape_support`).
         */
        m3d::vec3 support(const m3d::vec3 &dir) const
        {
            return support_a(dir) - support_b(-dir);
        }

        /** @brief World-space support of A along `dir`. */
        m3d::vec3 support_a(const m3d::vec3 &dir) const
        {
            const m3d::vec3 local_dir = tf_a.inverse_rotate_vector(dir);
            return tf_a.transform_point(shape_support(*shape_a, local_dir));
        }

        /** @brief World-space support of B along `dir`. */
        m3d::vec3 support_b(const m3d::vec3 &dir) const
        {
            const m3d::vec3 local_dir = tf_b.inverse_rotate_vector(dir);
            return tf_b.transform_point(shape_support(*shape_b, local_dir));
        }
    };
}
