#pragma once
#include <storage/Soa.hpp>
#include <math3d/math3d.hpp>
#include <rbps/Body.hpp>
#include <rbps/constraints/Constraint.hpp>

/**
 * @file Joint.hpp
 * @brief Joints (prismatic, revolute, fixed) layered on top of `ConstraintCollection`.
 * @ingroup rbps
 *
 * Each joint owns a small block of low-level rows in `ConstraintCollection`
 * (alignment / attachment / limit / drive). Each frame the joint
 * `compute_*_errors` functions write the current error values into those
 * rows; the XPBD solver then drives them to zero.
 */

using namespace m3d;

namespace rbps
{
    /**
     * @brief Joint kinematic kind.
     *
     * - `PRISMATIC` — sliding along an axis, no relative rotation.
     * - `REVOLUTE`  — rotation about an axis (hinge), no relative translation.
     * - `FIXED`     — no relative motion at all.
     *
     * @ingroup rbps
     */
    enum JointType
    {
        PRISMATIC,
        REVOLUTE,
        FIXED
    };

    /**
     * @brief Joint actuation mode.
     *
     * - `FREE`     — no driver; the joint is purely passive.
     * - `POSITION` — driver follows `target_position`.
     * - `SPEED`    — driver follows `target_speed` (target_position is
     *                advanced by `speed*dt` each frame).
     *
     * @ingroup rbps
     */
    enum JointActuationType
    {
        FREE,
        POSITION,
        SPEED,
    };

    /**
     * @brief Maximum number of low-level constraint rows owned by a single joint.
     *
     * Sized for the worst case (revolute has 4: alignment + attachment +
     * limit + drive). Bump if a future joint type needs more.
     *
     * @ingroup rbps
     */
    static constexpr uint32_t MAX_CONSTRAINTS_PER_JOINT = 4;

    /**
     * @brief Stable IDs of the low-level constraint rows that back a joint.
     *
     * Stored as part of each joint so the swap-and-pop in `JointCollection`
     * carries the constraint row IDs along with the joint.
     *
     * @ingroup rbps
     */
    struct ConstraintBlock
    {
        uint32_t ids[MAX_CONSTRAINTS_PER_JOINT] = {};
    };

// ─── Field list — ONE place to add / remove joint properties ─────────────────
//
// X(ValueType, field_name)
//
#define JOINT_FIELDS(X)                   \
    X(JointType, type)                    \
    X(JointActuationType, actuation_type) \
    X(u_int32_t, body_1)                     \
    X(u_int32_t, body_2)                     \
    X(vec3, r_1)                          \
    X(vec3, r_2)                          \
    X(vec3, main_axis)                    \
    X(bool, limited)                      \
    X(scalar, lower_limit)                \
    X(scalar, upper_limit)                \
    X(scalar, target_position)            \
    X(scalar, target_speed)               \
    X(scalar, current_position)           \
    X(scalar, damping)                    \
    X(vec3, limit_axis)                   \
    X(u_short, constraint_count)          \
    X(ConstraintBlock, constraints)

    /**
     * @ingroup rbps
     * @brief SoA collection of joints; each row owns up to `MAX_CONSTRAINTS_PER_JOINT`
     *        constraint rows in a `ConstraintCollection`.
     */
    DEFINE_DYN_SOA(JointCollection, uint32_t, /*GenerationBits=*/8, JOINT_FIELDS)

    /**
     * @brief Compute the three low-level XPBD constraint errors for a prismatic joint.
     *
     * For joint i:
     * 1. Computes the rotational alignment error between body orientations.
     * 2. Computes the positional attachment-point error along the sliding axis,
     *    including limit overshoot correction if enabled.
     * 3. Computes the drive error based on actuation type (FREE, SPEED, POSITION).
     *    - FREE: zero error
     *    - SPEED: advance target_position by speed*dt, then fall through
     *    - POSITION: error to target_position
     *
     * The errors are stored into the global ConstraintCollection at indices
     * [start+0..start+2].  Also updates jc.current_position[i] to the raw slide
     * distance.
     *
     * @param jc         The JointCollection containing prismatic joint data.
     * @param i          Index of the prismatic joint to process.
     * @param bc         The BodyCollection holding all body transforms.
     * @param cc         The global ConstraintCollection for low-level constraints.
     * @param time_step  The simulation time step Δt.
     *
     * @ingroup rbps
     */
    void compute_prismatic_joint_errors(JointCollection &jc,
                                        u_int32_t i,
                                        BodyCollection &bc,
                                        ConstraintCollection &cc,
                                        scalar time_step);

    /**
     * @brief Clamps the joint angle within limits and computes the rotational correction vector.
     *
     * If the current angle φ is outside [lower_limit, upper_limit], this function:
     * 1. Clamps φ into the valid range.
     * 2. Builds a quaternion representing rotation of φ about axis n.
     * 3. Rotates the initial limit-axis vector n_1 by that quaternion.
     * 4. Computes δq = n₁_rotated × n₂, which measures the misalignment.
     *
     * If φ was already within the limits, returns the zero vector.
     *
     * @param[in,out] phi         Current joint angle (in/out). On exit, clamped to [lower_limit, upper_limit].
     * @param[in]     n           Joint’s main rotation axis (unit vector in world space).
     * @param[in]     n_1         First body’s limit-axis vector (in world space).
     * @param[in]     n_2         Second body’s limit-axis vector (in world space).
     * @param[in]     lower_limit Minimum allowed angle (radians).
     * @param[in]     upper_limit Maximum allowed angle (radians).
     * @return                    Correction vector δq to drive the axes into alignment.
     *
     * @ingroup rbps
     * @ingroup internals
     */
    vec3 compute_angle_limit_correction(scalar &phi,
                                        vec3 n,
                                        vec3 n_1,
                                        vec3 n_2,
                                        scalar lower_limit,
                                        scalar upper_limit);

    /**
     * @brief Compute the four low-level XPBD constraint errors for a revolute joint.
     *
     * For joint i:
     * 1. Rotational alignment error of the main axis.
     * 2. Positional attachment-point error.
     * 3. Rotational angle-limit error (clamped if limited).
     * 4. Rotational drive error based on actuation type (FREE, SPEED, POSITION).
     *
     * The errors are stored into the global ConstraintCollection at indices
     * [start+0..start+3].  Also updates jc.current_position[i] to the current
     * joint angle φ.
     *
     * @param jc         The JointCollection containing revolute joint data.
     * @param i          Index of the revolute joint to process.
     * @param bc         The BodyCollection holding all body transforms.
     * @param cc         The global ConstraintCollection for low-level constraints.
     * @param time_step  The simulation time step Δt.
     *
     * @ingroup rbps
     */
    void compute_revolute_joint_errors(JointCollection &jc,
                                       u_int32_t i,
                                       BodyCollection &bc,
                                       ConstraintCollection &cc,
                                       scalar time_step);

    /**
     * @brief Compute the two low-level XPBD constraint errors for a fixed joint.
     *
     * For joint i:
     * 1. Rotational alignment error between body orientations.
     * 2. Positional attachment-point error.
     *
     * The errors are stored into the global ConstraintCollection at indices
     * [start+0..start+1].
     *
     * @param jc         The JointCollection containing fixed joint data.
     * @param i          Index of the fixed joint to process.
     * @param bc         The BodyCollection holding all body transforms.
     * @param cc         The global ConstraintCollection for low-level constraints.
     * @param time_step  The simulation time step Δt (unused for fixed joints).
     *
     * @ingroup rbps
     */
    void compute_fixed_joint_errors(
        JointCollection &jc,
        u_int32_t i,
        BodyCollection &bc,
        ConstraintCollection &cc,
        scalar /*time_step—unused*/
    );

    /**
     * @brief Applies damping impulses to a primsatic joint by resisting relative linear motion.
     *
     * Computes a corrective impulse based on the relative linear velocity between
     * the two bodies connected by joint i, scaled by the joint’s damping coefficient.
     * This impulse is applied as a positional‐velocity‐level constraint at each
     * body’s attachment point to dissipate energy and simulate viscous damping.
     *
     * Steps:
     * 1. Compute Δv = (v₂ − v₁) * min(damping * Δt, 1).
     * 2. If ‖Δv‖ < EPSILON, exit early (no significant relative motion).
     * 3. Calculate world‐space attachment points r₁, r₂.
     * 4. Normalize n = Δv / ‖Δv‖.
     * 5. Compute each body’s generalized inverse mass along n.
     * 6. Compute impulse = Δv / (w₁ + w₂).
     * 7. Apply equal-and-opposite velocity impulses at r₁ and r₂.
     *
     * @param jc         The JointCollection containing damping parameters.
     * @param i          Index of the joint to apply damping for.
     * @param bc         The BodyCollection holding body velocities and inverses.
     * @param time_step  Simulation time step Δt.
     *
     * @ingroup rbps
     */
    void apply_prismatic_joint_damping(JointCollection &jc,
                                       u_int32_t i,
                                       BodyCollection &bc,
                                       scalar time_step);

    /**
     * @brief Applies angular damping to a revolute joint to resist relative rotation.
     *
     * Computes and applies an angular damping impulse based on the difference
     * in angular velocity between two connected bodies. This simulates viscous
     * resistance around the joint’s axis and helps stabilize the simulation.
     *
     * Steps:
     * 1. Compute Δω = (ω₂ − ω₁) * min(damping * Δt, 1).
     * 2. Apply +Δω to body 1 and −Δω to body 2.
     *
     * @param jc         The JointCollection with damping values.
     * @param i          Index of the revolute joint.
     * @param bc         The BodyCollection holding angular velocities.
     * @param time_step  Simulation timestep Δt.
     *
     * @ingroup rbps
     */
    void apply_revolute_joint_damping(JointCollection &jc,
                                      u_int32_t i,
                                      BodyCollection &bc,
                                      scalar time_step);

    /**
     * @brief Computes joint errors.
     *
     * This function calculates the errors for each joint contraint
     * The errors represent the degree to which the joint constraints are violated.
     * These errors are then used to apply corrective impulses to satisfy the constraints.
     *
     * @param jc JointCollection reference
     * @param bc BodyCollection
     * @param cc ConstraintCollection
     * @param time_step timestep
     *
     * @ingroup rbps
     */
    void compute_joint_errors(JointCollection &jc,
                              BodyCollection &bc,
                              ConstraintCollection &cc,
                              scalar time_step);

    /**
     * @brief Applies joint damping to all the joints
     *
     * @param jc JointCollection reference
     * @param bc BodyCollection reference
     * @param time_step timestep
     *
     * @ingroup rbps
     */
    void apply_joint_damping(JointCollection &jc,
                             BodyCollection &bc,
                             scalar time_step);

} // namespace rbps