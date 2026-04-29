#pragma once
#include <cstdint>
#include <variant>
#include <math3d/math3d.hpp>

/**
 * @file Command.hpp
 * @brief Command types the visualizer can send back to the physics engine.
 * @ingroup visr
 *
 * Each `Cmd*` is a plain struct (no virtual dispatch). `Command =
 * std::variant<...>` is dispatched by `std::visit` in
 * [CommandDispatch.hpp](CommandDispatch.hpp).
 *
 * @note Pause / step-once are handled by `DebugChannel` directly (they
 *       don't mutate `World` state), so they intentionally do NOT mutate
 *       state in `dispatch_command`.
 */

namespace visr
{
    /** @brief Change the simulation timestep and/or substep count live. @ingroup visr */
    struct CmdSetTimestep
    {
        double   timestep;
        int      substeps;
    };

    /** @brief Select a body by stable ID. @ingroup visr */
    struct CmdSelectBody     { uint32_t id; };
    /** @brief Select a collider by stable ID. @ingroup visr */
    struct CmdSelectCollider { uint32_t id; };
    /** @brief Clear all selection state. @ingroup visr */
    struct CmdClearSelection {};

    /** @brief Apply an instantaneous linear impulse at a world-space point. @ingroup visr */
    struct CmdApplyImpulse
    {
        uint32_t    body_id;
        m3d::vec3   impulse;     ///< Impulse vector (world space).
        m3d::vec3   world_point; ///< World-space application point.
    };

    /** @brief Teleport a body (position + orientation) and zero its velocities. @ingroup visr */
    struct CmdTeleportBody
    {
        uint32_t    body_id;
        m3d::vec3   position;
        m3d::quat   orientation;
    };

    /** @brief Zero a body's linear and angular velocities. @ingroup visr */
    struct CmdZeroVelocity { uint32_t body_id; };

    /** @brief Set the position target for an actuated joint. @ingroup visr */
    struct CmdSetJointTarget
    {
        uint32_t    joint_id;
        m3d::scalar target; ///< Position target — radians for revolute, metres for prismatic.
    };

    /** @brief Set the speed target for an actuated joint. @ingroup visr */
    struct CmdSetJointSpeed
    {
        uint32_t    joint_id;
        m3d::scalar speed;
    };

    /** @brief Update a joint's damping coefficient live. @ingroup visr */
    struct CmdSetJointDamping
    {
        uint32_t    joint_id;
        m3d::scalar damping;
    };

    /**
     * @brief Quantities the visualizer can plot live.
     *
     * The render thread sends `CmdTrackQuantity` to subscribe / unsubscribe;
     * the physics thread samples these values into a `TrackedSeries` ring
     * buffer in `DebugChannel`.
     *
     * @ingroup visr
     */
    enum class TrackTarget : uint8_t
    {
        BodyLinearSpeed,
        BodyAngularSpeed,
        BodyPositionX, BodyPositionY, BodyPositionZ,
        ContactNormalLambda,
        ContactTangentLambda,
        ContactPenetrationDepth,
        JointPosition,
        JointError,
        ConstraintLambda,
    };

    /** @brief Subscribe / unsubscribe a tracked quantity for the live plots. @ingroup visr */
    struct CmdTrackQuantity
    {
        TrackTarget target;
        uint32_t    id;      ///< Body / contact / joint / constraint stable ID.
        bool        enabled; ///< `false` to stop tracking the series.
    };

    /** @brief Set the global gravity vector. @ingroup visr */
    struct CmdSetGravity { m3d::vec3 gravity; };

    /** @brief Pause the simulation — physics thread stops calling `world.step()`. @ingroup visr */
    struct CmdPause  {};
    /** @brief Resume a previously paused simulation. @ingroup visr */
    struct CmdResume {};

    /**
     * @brief Variant over every command type.
     *
     * Dispatched in [CommandDispatch.hpp](CommandDispatch.hpp) via
     * `std::visit`. Add a new command by extending this variant and
     * adding an `else if constexpr` arm there.
     *
     * @ingroup visr
     */
    using Command = std::variant<
        CmdPause,
        CmdResume,
        CmdSetTimestep,
        CmdSelectBody,
        CmdSelectCollider,
        CmdClearSelection,
        CmdApplyImpulse,
        CmdTeleportBody,
        CmdZeroVelocity,
        CmdSetJointTarget,
        CmdSetJointSpeed,
        CmdSetJointDamping,
        CmdTrackQuantity,
        CmdSetGravity
    >;

} // namespace visr