#pragma once
#include "visr/Command.hpp"
#include "rbps/API/World.hpp"

/**
 * @file CommandDispatch.hpp
 * @brief Single entry point that applies any `Command` to a `World`.
 * @ingroup visr
 *
 * Uses `std::visit` so adding a new `Cmd*` is one extra `if constexpr`
 * arm — no switch, no virtual dispatch.
 *
 * @note `CmdSetGravity` is intentionally left as a stub: gravity lives in
 *       the integrator (`Body.cpp`), not in `World`. Wire it up to
 *       wherever the gravity vector is stored when the field is added.
 */

namespace visr
{
    /**
     * @brief Apply `cmd` to `world` (visit-based dispatch).
     *
     * Channel-level commands (`CmdPause`, `CmdResume`, `CmdTrackQuantity`)
     * are intentional no-ops here — they're handled in
     * `DebugChannel::poll` so the world isn't touched when the user just
     * pauses or starts plotting a value.
     *
     * @ingroup visr
     */
    inline void dispatch_command(rbps::World &world, const Command &cmd)
    {
        std::visit([&world](auto &&c)
                   {
                       using T = std::decay_t<decltype(c)>;

                       // ── Pause/resume (handled at DebugChannel level; no-op here) ───────
                       if constexpr (std::is_same_v<T, CmdPause>)
                       {
                       }
                       else if constexpr (std::is_same_v<T, CmdResume>)
                       {
                       }

                       // ── Simulation config ────────────────────────────────────────
                       else if constexpr (std::is_same_v<T, CmdSetTimestep>)
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

                       else if constexpr (std::is_same_v<T, CmdSpawnBody>)
                    {
                        rbps::BodyParams bp{};
                        bp.type = rbps::BodyType::DYNAMIC;
                        bp.position = c.position;
                        bp.orientation = c.orientation;
                        bp.mass = c.mass;
                        // Scale unit inertia by mass/volume
                        bp.inertia_tensor = c.unit_inertia * (c.mass / c.volume);
                        
                        uint32_t id = world.create_body(bp);

                        rbps::ColliderParams cp{};
                        cp.body_id = id;
                        cp.shape = c.shape;
                        cp.restitution = 0.2;
                        cp.static_friction = 0.5;
                        cp.dynamic_friction = 0.4;
                        world.create_collider(cp);
                    } },
                   cmd);
    }

} // namespace visr