#pragma once
#include "visr/Command.hpp"
#include "rbps/API/World.hpp"

// ============================================================================
//  visr/CommandDispatch.hpp
//
//  Single entry point: dispatch_command(World&, Command) applies whatever
//  mutation the command requests.  Uses std::visit so adding a new Cmd*
//  type is one extra lambda arm — no switch, no virtual dispatch.
//
//  NOTE: CmdSetGravity is left as a comment hook.  Gravity lives in the
//  integrator (Body.cpp), not in World directly.  Wire it up to wherever
//  you store your gravity vector (e.g. World::gravity or a global in Body.cpp).
// ============================================================================

namespace visr
{
    inline void dispatch_command(rbps::World &world, const Command &cmd)
    {
        std::visit([&world](auto &&c)
                   {
                       using T = std::decay_t<decltype(c)>;

                       // ── Simulation config ────────────────────────────────────────
                       if constexpr (std::is_same_v<T, CmdSetTimestep>)
                       {
                           world.timestep = c.timestep;
                           world.substeps = c.substeps;
                       }

                       // ── Selection (engine side: no-op by default, user can hook) ─
                       else if constexpr (std::is_same_v<T, CmdSelectBody>)
                       {
                       }
                       else if constexpr (std::is_same_v<T, CmdSelectCollider>)
                       {
                       }
                       else if constexpr (std::is_same_v<T, CmdClearSelection>)
                       {
                       }

                       // ── Impulse ──────────────────────────────────────────────────
                       else if constexpr (std::is_same_v<T, CmdApplyImpulse>)
                       {
                           const uint32_t slot = world.bodies.index_of(c.body_id);
                           const m3d::vec3 r = c.world_point - world.bodies.position[slot];
                           rbps::apply_positional_velocity_constraint_impulse(
                               world.bodies, slot, c.impulse, r);
                       }

                       // ── Teleport ─────────────────────────────────────────────────
                       else if constexpr (std::is_same_v<T, CmdTeleportBody>)
                       {
                           const uint32_t slot = world.bodies.index_of(c.body_id);
                           world.bodies.position[slot] = c.position;
                           world.bodies.orientation[slot] = c.orientation;
                           world.bodies.linear_velocity[slot] = {};
                           world.bodies.angular_velocity[slot] = {};
                           world.bodies.prev_position[slot] = c.position;
                           world.bodies.prev_orientation[slot] = c.orientation;
                       }

                       // ── Zero velocity ────────────────────────────────────────────
                       else if constexpr (std::is_same_v<T, CmdZeroVelocity>)
                       {
                           const uint32_t slot = world.bodies.index_of(c.body_id);
                           world.bodies.linear_velocity[slot] = {};
                           world.bodies.angular_velocity[slot] = {};
                       }

                       // ── Joint control ────────────────────────────────────────────
                       else if constexpr (std::is_same_v<T, CmdSetJointTarget>)
                       {
                           const uint32_t slot = world.joints.index_of(c.joint_id);
                           world.joints.target_position[slot] = c.target;
                       }
                       else if constexpr (std::is_same_v<T, CmdSetJointSpeed>)
                       {
                           const uint32_t slot = world.joints.index_of(c.joint_id);
                           world.joints.target_speed[slot] = c.speed;
                       }
                       else if constexpr (std::is_same_v<T, CmdSetJointDamping>)
                       {
                           const uint32_t slot = world.joints.index_of(c.joint_id);
                           world.joints.damping[slot] = c.damping;
                       }

                       // ── Quantity tracking: handled at DebugChannel level ─────────
                       else if constexpr (std::is_same_v<T, CmdTrackQuantity>)
                       {
                       }

                       // ── Gravity ──────────────────────────────────────────────────
                       // TODO: wire to wherever World stores the gravity vector.
                       else if constexpr (std::is_same_v<T, CmdSetGravity>)
                       {
                       }
                   },
                   cmd);
    }

} // namespace visr