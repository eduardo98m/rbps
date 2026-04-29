#pragma once

#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "math3d/tf.hpp"
#include "rbc/Contact.hpp"

/**
 * @file narrow_phase.hpp
 * @brief Public entry point for narrow-phase collision queries.
 * @ingroup rbc
 */

namespace rbc {

    /**
     * @brief Test a single shape pair and fill a `ContactManifold` on overlap.
     *
     * Routes through the dispatch table in [Dispatcher.hpp](Dispatcher.hpp):
     * analytic algorithms for the supported pairs, GJK + EPA + manifold
     * generation otherwise. Non-convex pairs without an analytic
     * specialisation return `false`.
     *
     * @param shape_a   First collider.
     * @param tf_a      Transform for `shape_a`.
     * @param shape_b   Second collider.
     * @param tf_b      Transform for `shape_b`.
     * @param[out] manifold  Filled with up to 4 contact points; `normal`
     *                       points from A toward B.
     * @return `true` if the shapes overlap and `manifold` is populated.
     *
     * @ingroup rbc
     */
    bool test_narrow_phase(const Shape& shape_a, const m3d::tf& tf_a,
                           const Shape& shape_b, const m3d::tf& tf_b,
                           rbc::ContactManifold& manifold);

} // namespace rbc
