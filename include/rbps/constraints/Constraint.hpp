#pragma once

#include <storage/Soa.hpp>
#include <math3d/math3d.hpp>
#include <rbps/Body.hpp>

/**
 * @file Constraint.hpp
 * @brief Low-level constraint SoA + XPBD impulse computation.
 * @ingroup rbps
 */

namespace rbps
{
    /**
     * @brief Constraint flavour.
     *
     * - `POSITIONAL` — couples translation: enforces a vector relationship
     *   between two attachment points.
     * - `ROTATIONAL` — couples orientation: enforces an axis or angle
     *   relationship between two body frames.
     *
     * @ingroup rbps
     */
    enum ConstraintType
    {
        POSITIONAL,
        ROTATIONAL
    };

#define CONSTRAINT_FIELDS(X) \
    X(size_t, body_1)        \
    X(size_t, body_2)        \
    X(vec3, r_1)             \
    X(vec3, r_2)             \
    X(vec3, direction)       \
    X(scalar, magnitude)     \
    X(scalar, lambda)        \
    X(vec3, force)           \
    X(vec3, torque)          \
    X(scalar, compliance)    \
    X(ConstraintType, type)  \
    X(vec3, impulse)

    /**
     * @ingroup rbps
     * @brief SoA collection of per-constraint data used by the XPBD solver.
     *
     * Holds every data array needed to evaluate and apply constraint
     * impulses between pairs of bodies. Joints are built on top of this
     * collection (one row per low-level constraint they generate).
     */
    DEFINE_DYN_SOA(ConstraintCollection, uint32_t, /*GenerationBits=*/8, CONSTRAINT_FIELDS)

    /**
     * @brief Computes and stores magnitude and direction for a raw constraint vector.
     *
     * @param cc    The ConstraintCollection to update.
     * @param i     Index of the constraint.
     * @param value Raw 3D vector whose magnitude/direction to extract.
     *
     * @ingroup rbps
     */
    void set_value(ConstraintCollection &cc, uint32_t i, vec3 value);

    /**
     * @brief Computes the change in the Lagrange multiplier for one constraint.
     *
     * Implements: Δλ = (–b – α·λ_old) / (w₁ + w₂ + α)
     *
     * @param cc                The ConstraintCollection.
     * @param i                 Index of the constraint.
     * @param w_1               Generalized inverse mass of body 1.
     * @param w_2               Generalized inverse mass of body 2.
     * @param inverse_time_step Inverse of the time step (1/Δt).
     * @return                  Change in Lagrange multiplier Δλ.
     *
     * @ingroup rbps
     */
    scalar compute_delta_lambda(const ConstraintCollection &cc, uint32_t i, scalar w_1, scalar w_2, scalar inverse_time_step);

    /**
     * @brief Solves and applies a rotational constraint impulse between two bodies.
     *
     * - Computes generalized inverse masses for each body about the constraint axis.
     * - Computes Δλ and updates the constraint’s λ.
     * - Converts Δλ into an impulse and corresponding torque.
     * - Applies equal-and-opposite rotational impulses to each body.
     *
     * @param bc                The BodyCollection containing all bodies.
     * @param cc                The ConstraintCollection.
     * @param i                 Index of the constraint.
     * @param inverse_time_step Inverse of the time step (1/Δt).
     *
     * @ingroup rbps
     */
    void compute_rotational_constraint_impulse(BodyCollection &bc, ConstraintCollection &cc, uint32_t i, scalar inverse_time_step);

    /**
     * @brief Solves and applies a rotational constraint impulse between two bodies.
     *
     * - Computes generalized inverse masses for each body about the constraint axis.
     * - Computes Δλ and updates the constraint’s λ.
     * - Converts Δλ into an impulse and corresponding torque.
     * - Applies equal-and-opposite rotational impulses to each body.
     *
     * @param bc                The BodyCollection containing all bodies.
     * @param cc                The ConstraintCollection.
     * @param i                 Index of the constraint.
     * @param inverse_time_step Inverse of the time step (1/Δt).
     *
     * @ingroup rbps
     */
    void compute_positional_constraint_impulse(BodyCollection &bc, ConstraintCollection &cc, uint32_t i, scalar inverse_time_step);

    /**
     * @brief Resets the Lagrange multiplier for a given constraint to zero.
     *
     * @param cc The ConstraintCollection.
     * @param i  Index of the constraint to reset.
     *
     * @ingroup rbps
     */
    void reset_lagrange_multiplier(ConstraintCollection &cc, uint32_t i);

    /**
     * @brief Sets the local contact points for a constraint.
     *
     * @param cc  The ConstraintCollection.
     * @param i   Index of the constraint.
     * @param r_1 Local contact point on body 1.
     * @param r_2 Local contact point on body 2.
     *
     * @ingroup rbps
     */
    void set_constraint_positions(ConstraintCollection &cc, uint32_t i, const vec3 &r_1, const vec3 &r_2);
    /**
     * @brief Returns the current Lagrange multiplier λ for a constraint.
     *
     * @param cc The ConstraintCollection.
     * @param i  Index of the constraint.
     * @return   Current λ value.
     *
     * @ingroup rbps
     */
    scalar get_lagrange_multiplier(const ConstraintCollection &cc, uint32_t i);

    /**
     * @brief Computes the updated Lagrange multiplier for a single constraint.
     *
     * Does not store the result—returns λ_old + Δλ based on current state.
     *
     * @param bc                The BodyCollection.
     * @param cc                The ConstraintCollection.
     * @param i                 Index of the constraint.
     * @param inverse_time_step Inverse of the time step (1/Δt).
     * @return                  Updated Lagrange multiplier.
     *
     * @ingroup rbps
     */
    scalar compute_lagrange_multiplier(BodyCollection &bc, ConstraintCollection &cc, uint32_t i, scalar inverse_time_step);

    /**
     * @brief Solves the constraints by computing impulses of the bodies.
     *
     * @param bc                The BodyCollection containing all bodies.
     * @param cc                The ConstraintCollection containing all constraints.
     * @param inverse_time_step Inverse of the time step (1/Δt).
     *
     * @ingroup rbps
     */
    void solve_constraints(BodyCollection &bc, ConstraintCollection &cc, scalar inverse_time_step);

} // namespace rbps