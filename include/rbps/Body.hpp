#pragma once
// rbps/Body.hpp

#include <storage/Soa.hpp>
#include <math3d/math3d.hpp>

using namespace m3d;

namespace rbps
{

    /**
     * @brief Enum for the motion type of a body.
     * STATIC  — infinite mass, never moved by the solver.
     * DYNAMIC — integrated each frame.
     */
    enum BodyType
    {
        STATIC,
        DYNAMIC
    };

// ─────────────────────────────────────────────────────────────────────────────
//  BODY_FIELDS — the ONLY place body fields are declared.
//  BodyCollection, its vectors, swap, pop, and reserve are all derived
//  automatically from this macro.  To add a field: one line here.
// ─────────────────────────────────────────────────────────────────────────────
#define BODY_FIELDS(X)               \
    X(vec3, force)                   \
    X(vec3, torque)                  \
    X(scalar, mass)                  \
    X(scalar, inverse_mass)          \
    X(BodyType, type)                \
    X(vec3, position)                \
    X(quat, orientation)             \
    X(vec3, linear_velocity)         \
    X(vec3, angular_velocity)        \
    X(vec3, prev_position)           \
    X(quat, prev_orientation)        \
    X(vec3, prev_linear_velocity)    \
    X(vec3, prev_angular_velocity)   \
    X(smat3, inertia_tensor)         \
    X(smat3, inverse_inertia_tensor) \
    X(smat3, inertia_tensor_world)   \
    X(smat3, inverse_inertia_tensor_world)

    // ─────────────────────────────────────────────────────────────────────────────
    //  BodyCollection
    //
    //  Dynamic SoA: grows at runtime, no compile-time capacity needed.
    //  ID type:       uint32_t
    //  GenerationBits: 8  →  24-bit slot index (≤16 M bodies),
    //                         256 reuse cycles before generation wraps.
    //
    //  Access pattern:
    //    uint32_t id = create_body(bc, params);   // stable ID
    //    uint32_t i  = bc.index_of(id);           // current packed index
    //    bc.position[i] = {0, 5, 0};             // direct field write
    //
    //    for (uint32_t i = 0; i < bc.count(); ++i)   // solver loop
    //        bc.position[i] += bc.linear_velocity[i] * dt;
    // ─────────────────────────────────────────────────────────────────────────────
    DEFINE_DYN_SOA(BodyCollection, uint32_t, /*GenerationBits=*/8, BODY_FIELDS)

    // ─────────────────────────────────────────────────────────────────────────────
    //  Free functions — implementations in Body.cpp
    //  All take a packed data index i, NOT a stable ID.
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @brief Recompute the world-space inertia tensor and its inverse for body i.
     *
     * Must be called whenever orientation[i] changes before any function that
     * reads inertia_tensor_world or inverse_inertia_tensor_world.
     *
     * @param bc  The BodyCollection.
     * @param i   Packed data index of the body to update.
     */
    void update_inertia_tensor_world(BodyCollection &bc, uint32_t i);

    /**
     * @brief Integrate positions and orientations for all dynamic bodies.
     *
     * For each dynamic body:
     *   1. Store previous pose.
     *   2. Recompute world inertia tensor.
     *   3. Integrate linear velocity from applied forces.
     *   4. Integrate position.
     *   5. Integrate angular velocity from applied torques (gyroscopic correction).
     *   6. Update orientation via small-angle quaternion approximation.
     *
     * @param bc  The BodyCollection.
     * @param dt  Time step (Δt).
     */
    void update_position_and_orientation(BodyCollection &bc, scalar dt);

    /**
     * @brief Recompute linear and angular velocities from pose deltas.
     *
     * Called after constraint solving to derive post-solve velocities.
     *
     * @param bc      The BodyCollection.
     * @param inv_dt  Inverse time step (1/Δt).
     */
    void update_velocities(BodyCollection &bc, scalar inv_dt);

    /**
     * @brief Apply a positional impulse at offset r to body i.
     *
     * Adjusts position and orientation. Used during position-level constraint
     * solving (XPBD).
     *
     * @param bc      The BodyCollection.
     * @param i       Packed data index.
     * @param impulse Impulse vector in world space.
     * @param r       Lever arm from centre of mass to application point (world space).
     */
    void apply_positional_constraint_impulse(BodyCollection &bc, uint32_t i, vec3 impulse, vec3 r);

    /**
     * @brief Apply a pure rotational impulse to body i.
     *
     * @param bc      The BodyCollection.
     * @param i       Packed data index.
     * @param impulse Rotational impulse (torque impulse) in world space.
     */
    void apply_rotational_constraint_impulse(BodyCollection &bc, uint32_t i, vec3 impulse);

    /**
     * @brief Apply a velocity-level impulse at offset r to body i.
     *
     * Adjusts linear_velocity and angular_velocity. Used during velocity-level
     * constraint solving (restitution, friction).
     * No-op if the body is STATIC.
     *
     * @param bc      The BodyCollection.
     * @param i       Packed data index.
     * @param impulse Impulse vector in world space.
     * @param r       Lever arm from centre of mass to application point (world space).
     */
    void apply_positional_velocity_constraint_impulse(BodyCollection &bc, uint32_t i, vec3 impulse, vec3 r);

    /**
     * @brief Compute the generalised inverse mass for a positional constraint.
     *
     * Returns  w = 1/m + (r×n)ᵀ I⁻¹ (r×n).
     * Returns 0 for STATIC bodies.
     *
     * @param bc  The BodyCollection.
     * @param i   Packed data index.
     * @param r   Vector from centre of mass to contact point.
     * @param n   Constraint direction (unit normal).
     * @return    Generalised inverse mass scalar.
     */
    scalar get_positional_generalized_inverse_mass(BodyCollection &bc, uint32_t i, vec3 r, vec3 n);

    /**
     * @brief Compute the generalised inverse mass for a rotational constraint.
     *
     * Returns  w = nᵀ I⁻¹ n.
     * Returns 0 for STATIC bodies.
     *
     * @param bc  The BodyCollection.
     * @param i   Packed data index.
     * @param n   Rotation axis (unit direction).
     * @return    Generalised inverse mass scalar.
     */
    scalar get_rotational_generalized_inverse_mass(BodyCollection &bc, uint32_t i, vec3 n);

} // namespace rbps