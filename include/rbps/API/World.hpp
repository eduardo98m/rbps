#pragma once
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/JointAPI.hpp"
#include "rbps/API/ConstraintAPI.hpp"
#include "rbps/CollisionPipeline.hpp"
#include "rbps/constraints/Contact.hpp"
#include "rbc/BroadPhase.hpp"

/**
 * @file World.hpp
 * @brief `World` — the public RBPS facade.
 * @ingroup rbps
 *
 * `World` owns every collection (bodies, colliders, joints, constraints,
 * contacts) and the broad-phase state, and exposes the per-frame `step()`
 * loop. This is the only API most users need to interact with directly.
 *
 * @par Step loop structure (per frame)
 * @code
 * ┌─ frame ─────────────────────────────────────────────────────────────┐
 * │  1. update_broad_phase_aabbs()   — push swept AABBs into SAP        │
 * │  2. broad_phase_update()         — SAP sort+sweep → bp.pairs        │
 * │  3. contacts.clear()             — drop last frame's contacts       │
 * │  4. run_narrow_phase()           — GJK/EPA dispatch → ContactList   │
 * │                                                                     │
 * │  ┌─ substep × N ──────────────────────────────────────────────────┐ │
 * │  │  5.  update_position_and_orientation()                         │ │
 * │  │  6a. solve_contacts_position_level()   ─┐                      │ │
 * │  │  6b. compute_joint_errors()             ├─ solve_positions()   │ │
 * │  │  6c. solve_constraints()               ─┘                      │ │
 * │  │  7.  update_velocities()                                       │ │
 * │  │  8a. solve_contacts_velocity_level()  ─┐                       │ │
 * │  │  8b. apply_joint_damping()             ├─ solve_velocities()   │ │
 * │  └────────────────────────────────────────┘                       │ │
 * └─────────────────────────────────────────────────────────────────────┘
 * @endcode
 *
 * @par Why broad+narrow outside the substep loop?
 * Running narrow phase once per frame and reusing the same contact set
 * across substeps (resetting lambdas each *frame*, NOT each *substep*) is
 * standard XPBD practice. It avoids `N × GJK/EPA` calls per frame while
 * the position-level solver incrementally refines the same contacts.
 *
 * If you need higher accuracy for fast-moving objects, move
 * `run_narrow_phase` inside the substep loop and call `contacts.clear()`
 * at the top of each substep — at proportional CPU cost.
 *
 * @par Hello world
 * @code
 * #include <rbps/API/World.hpp>
 *
 * rbps::World world{1.0 / 60.0, 20};   // 60 Hz, 20 substeps
 *
 * // A static plane as a floor.
 * uint32_t floor_id = world.create_body({ .type = rbps::STATIC });
 * world.create_collider({ .shape = rbc::Plane{}, .body_id = floor_id });
 *
 * // A dynamic sphere dropped from y=5.
 * uint32_t ball_id = world.create_body({ .mass = 1.0, .position = {0, 5, 0} });
 * world.create_collider({ .shape = rbc::Sphere{0.5}, .body_id = ball_id });
 *
 * for (int frame = 0; frame < 600; ++frame)
 *     world.step();
 *
 * uint32_t i = world.bodies.index_of(ball_id);
 * std::cout << world.bodies.position[i] << "\n";
 * @endcode
 */

namespace rbps
{
    /**
     * @brief Owns the simulation state and runs the per-frame step loop.
     *
     * Plain aggregate of the underlying collections: callers can poke at
     * the data arrays directly when they need to (e.g. `world.bodies.position[i]`).
     * The `create_*` helpers wrap the corresponding free-function
     * `rbps::create_*` so the broad phase / constraint book-keeping is
     * kept consistent.
     *
     * @ingroup rbps
     */
    struct World
    {
        scalar timestep = 1.0 / 60.0; ///< Frame duration in seconds.
        int substeps = 20;            ///< Substeps per frame; higher = more stable, more expensive.

        BodyCollection       bodies;            ///< All rigid bodies.
        ConstraintCollection constraints;       ///< Low-level constraint rows (joints, contacts share this pool).
        JointCollection      joints;            ///< High-level joints (each owns several constraint rows).
        ContactList          contacts;          ///< Output of the per-frame contact pipeline.
        ColliderCollection   colliders;         ///< All colliders (shape + material + offset).
        rbc::BroadPhaseState broad_phase_state; ///< SAP broad-phase scratch and outputs.

        /** @brief Default-construct a world with 60 Hz / 20 substeps. */
        World() = default;

        /** @brief Construct with explicit timestep and substep count. */
        World(scalar timestep_, int substeps_)
            : timestep(timestep_), substeps(substeps_)
        {
        }

        // -----------------------------------------------------------------
        //  Creation helpers
        // -----------------------------------------------------------------

        /** @brief Create a body. Returns its stable ID. */
        uint32_t create_body(BodyParams params)
        {
            return rbps::create_body(bodies, params);
        }

        /**
         * @brief Create a collider attached to an existing body.
         *
         * The world transform is computed from the owning body so the
         * broad-phase AABB is correct on the first frame.
         */
        uint32_t create_collider(ColliderParams params)
        {
            const uint32_t body_slot = bodies.index_of(params.body_id);
            const m3d::vec3 body_pos = bodies.position[body_slot];
            const m3d::quat body_rot = bodies.orientation[body_slot];
            const m3d::tf body_tf{body_pos, body_rot};

            return rbps::create_collider(colliders, broad_phase_state, params, body_tf);
        }

        /** @brief Create a low-level constraint row. */
        uint32_t create_constraint(ConstraintParams params)
        {
            return rbps::create_constraint(constraints, params);
        }

        /** @brief Create a prismatic (sliding) joint. */
        uint32_t create_prismatic_joint(PrismaticJointParams params)
        {
            return rbps::create_prismatic_joint(joints, constraints, params);
        }

        /** @brief Create a revolute (hinge) joint. */
        uint32_t create_revolute_joint(RevoluteJointParams params)
        {
            return rbps::create_revolute_joint(joints, constraints, params);
        }

        /** @brief Create a fixed (rigidly-locked) joint. */
        uint32_t create_fixed_joint(FixedJointParams params)
        {
            return rbps::create_fixed_joint(joints, constraints, params);
        }

        // -----------------------------------------------------------------
        //  Substep solvers (called N times per frame)
        // -----------------------------------------------------------------

        /**
         * @brief Position-level solver for one substep.
         *
         * Reuses the contact set built once at frame start; lambdas
         * accumulate across substeps. Do NOT clear `contacts` here.
         */
        void solve_positions(scalar inv_h, scalar h)
        {
            solve_contacts_position_level(contacts, bodies, inv_h);
            compute_joint_errors(joints, bodies, constraints, h);
            solve_constraints(bodies, constraints, inv_h);
        }

        /** @brief Velocity-level solver for one substep (restitution + friction + joint damping). */
        void solve_velocities(scalar h)
        {
            solve_contacts_velocity_level(contacts, bodies, h);
            apply_joint_damping(joints, bodies, h);
        }

        // -----------------------------------------------------------------
        //  Main step
        // -----------------------------------------------------------------
        /**
         * @brief Advance the simulation by one frame (`timestep` seconds, `substeps` substeps).
         *
         * Runs the broad/narrow phase once per frame (per the XPBD step
         * loop diagram in this file's overview) then iterates the
         * position-level and velocity-level solvers `substeps` times.
         */
        void step()
        {
            const scalar h = timestep / substeps;
            const scalar inv_h = 1.0 / h;

            update_broad_phase_aabbs(colliders, broad_phase_state,
                                     bodies, timestep,
                                     /*use_velocity_expansion=*/true);
            rbc::broad_phase_update(broad_phase_state);

            contacts.reserve(broad_phase_state.pairs.size() * 4);
            for (int i = 0; i < substeps; ++i)
            {
                contacts.clear();
                rbps::run_narrow_phase(broad_phase_state, colliders, bodies, contacts);
                update_position_and_orientation(bodies, h);
                solve_positions(inv_h, h);
                update_velocities(bodies, inv_h);
                solve_velocities(h);
            }
        }
    };

} // namespace rbps
