#pragma once
#include <math3d/math3d.hpp>

/**
 * @file Contact.hpp
 * @brief `ContactPoint` and `ContactManifold` — the output of every collision query.
 * @ingroup rbc
 */

namespace rbc
{

    /**
     * @brief A single contact point: world-space position and penetration depth.
     *
     * Penetration is positive when the bodies overlap; the constraint solver
     * uses it as the bias / Baumgarte term for the contact constraint.
     *
     * @ingroup rbc
     */
    struct ContactPoint
    {
        m3d::vec3 position;            ///< World-space contact location.
        m3d::scalar penetration_depth; ///< Positive when bodies overlap.
    };

    /**
     * @brief Up to 4 contact points sharing a single normal (the "manifold").
     *
     * `normal` always points from body A toward body B — the direction A
     * needs to move to separate. The 4-point capacity covers the worst
     * case (face/face contact between boxes); simpler pairs (e.g. sphere
     * vs sphere) populate just `points[0]`.
     *
     * @ingroup rbc
     */
    struct ContactManifold
    {
        m3d::vec3 normal;       ///< Unit contact normal, from A toward B.
        ContactPoint points[4]; ///< Contact points; only `[0..num_points)` are valid.
        int num_points = 0;     ///< Number of valid entries in `points`.

        /**
         * @brief Single-point convenience accessor (kept for the GJK/EPA path
         *        that produces only one contact).
         */
        const ContactPoint &first() const { return points[0]; }

        /** @brief Reset the manifold to empty (does not clear the normal). */
        void clear() { num_points = 0; }
    };

} // namespace rbc
