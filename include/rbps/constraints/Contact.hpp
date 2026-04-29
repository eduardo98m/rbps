#pragma once
#include <math3d/math3d.hpp>
#include "rbps/Body.hpp"
#include "rbps/CollisionPipeline.hpp"

/**
 * @file Contact.hpp
 * @brief Position-level and velocity-level contact constraint solvers.
 * @ingroup rbps
 *
 * All functions operate on `ContactList` (the unified SoA defined in
 * [CollisionPipeline.hpp](../CollisionPipeline.hpp)). Naming follows
 * the XPBD literature:
 * - `λ` (`lambda`)        — accumulated Lagrange multiplier (constraint
 *                            impulse magnitude).
 * - `Δλ` (`delta_lambda`) — per-iteration increment.
 * - `w`                   — generalised inverse mass along the constraint
 *                            direction.
 */

namespace rbps
{
    /**
     * @brief Position-level solve for one contact (called every substep).
     *
     * Steps:
     * 1. Re-check penetration depth along the normal using current positions.
     *    If the bodies have separated, sets `collision[i] = false` and exits.
     * 2. Records the pre-solve normal relative velocity in `relative_velocity[i]`
     *    (used later by the velocity-level solver for restitution).
     * 3. Delegates to `solve_normal_constraint` and `solve_tangent_constraint`.
     *
     * @ingroup rbps
     */
    void apply_constraint_position_level(ContactList  &cl,
                                         uint32_t      i,
                                         BodyCollection &bc,
                                         m3d::scalar    inv_h);

    /**
     * @brief Non-penetration (normal) constraint — solver inner loop.
     *
     * Computes `Δλ = (−d − α·λ_old) / (w_a + w_b + α)` and applies the
     * resulting positional impulse to both bodies.
     *
     * @ingroup rbps
     * @ingroup internals
     */
    void solve_normal_constraint(ContactList    &cl,
                                 uint32_t        i,
                                 BodyCollection &bc,
                                 m3d::scalar     inv_h,
                                 m3d::scalar     penetration,
                                 m3d::vec3       r_a_wc,
                                 m3d::vec3       r_b_wc);

    /**
     * @brief Tangential (friction) constraint — solver inner loop.
     *
     * Computes the tangential slip Δp_t since the last substep and
     * applies a Coulomb-clamped friction impulse.
     *
     * @ingroup rbps
     * @ingroup internals
     */
    void solve_tangent_constraint(ContactList    &cl,
                                  uint32_t        i,
                                  BodyCollection &bc,
                                  m3d::scalar     inv_h);

    /**
     * @brief Velocity-level solve for one contact (called once per substep).
     *
     * Applies restitution along the normal and dynamic Coulomb friction in
     * the tangential plane. Uses `relative_velocity[i]` captured at
     * position-solve time as the pre-collision velocity for the
     * restitution coefficient.
     *
     * @ingroup rbps
     */
    void apply_constraint_velocity_level(ContactList    &cl,
                                         uint32_t        i,
                                         BodyCollection &bc,
                                         m3d::scalar     h);

    /**
     * @brief Iterate `apply_constraint_position_level` over every contact.
     * @ingroup rbps
     */
    void solve_contacts_position_level(ContactList    &cl,
                                       BodyCollection &bc,
                                       m3d::scalar     inv_h);

    /**
     * @brief Iterate `apply_constraint_velocity_level` over every contact.
     * @ingroup rbps
     */
    void solve_contacts_velocity_level(ContactList    &cl,
                                       BodyCollection &bc,
                                       m3d::scalar     h);

} // namespace rbps
