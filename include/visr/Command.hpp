#pragma once
#include <cstdint>
#include <variant>
#include <math3d/math3d.hpp>

// ============================================================================
//  visr/Command.hpp
//
//  All commands the visualizer can send back to the physics engine.
//  Each Cmd* is a plain struct with zero virtual functions.
//  Command = std::variant<all of them> — dispatched with std::visit.
//
//  Paused / step-once are handled by DebugChannel directly (they don't
//  mutate World state), so they're NOT in the Command variant.
// ============================================================================

namespace visr
{
    // ── Simulation control ───────────────────────────────────────────────────

    /// Change the simulation timestep and/or substep count live.
    struct CmdSetTimestep
    {
        double   timestep;
        int      substeps;
    };

    // ── Selection ────────────────────────────────────────────────────────────

    /// Tell the engine which body the user has selected (optional: engine can
    /// highlight it, freeze it, expose extra debug data, etc.).
    struct CmdSelectBody     { uint32_t id; };
    struct CmdSelectCollider { uint32_t id; };
    struct CmdClearSelection {};

    // ── Scene mutation ───────────────────────────────────────────────────────

    /// Apply an instantaneous linear impulse to a body at a world-space point.
    struct CmdApplyImpulse
    {
        uint32_t    body_id;
        m3d::vec3   impulse;        // world space
        m3d::vec3   world_point;    // application point
    };

    /// Teleport a body (position + orientation) — zeroes velocities.
    struct CmdTeleportBody
    {
        uint32_t    body_id;
        m3d::vec3   position;
        m3d::quat   orientation;
    };

    /// Zero a body's linear and angular velocities.
    struct CmdZeroVelocity { uint32_t body_id; };

    // ── Joint control ────────────────────────────────────────────────────────

    struct CmdSetJointTarget
    {
        uint32_t    joint_id;
        m3d::scalar target;         // position (rad or m) depending on joint type
    };

    struct CmdSetJointSpeed
    {
        uint32_t    joint_id;
        m3d::scalar speed;
    };

    struct CmdSetJointDamping
    {
        uint32_t    joint_id;
        m3d::scalar damping;
    };

    // ── Quantity tracking (for live graphs) ─────────────────────────────────

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

    struct CmdTrackQuantity
    {
        TrackTarget target;
        uint32_t    id;         // body/contact/joint/constraint id
        bool        enabled;    // false = stop tracking this series
    };

    // ── Gravity ─────────────────────────────────────────────────────────────

    struct CmdSetGravity { m3d::vec3 gravity; };

    // ── Variant ─────────────────────────────────────────────────────────────

    using Command = std::variant<
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