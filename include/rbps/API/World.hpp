#pragma once
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/JointAPI.hpp"
#include "rbps/API/ConstraintAPI.hpp"
#include "rbps/CollisionPipeline.hpp"   // ContactList, run_narrow_phase, etc.
#include "rbps/constraints/Contact.hpp" // solve_contacts_*
#include "rbc/BroadPhase.hpp"

// ============================================================================
//  World.hpp
//
//  Step loop structure (per frame):
//
//    ┌─ frame ─────────────────────────────────────────────────────────────┐
//    │  1. update_broad_phase_aabbs()   — push swept AABBs into SAP        │
//    │  2. broad_phase_update()         — SAP sort+sweep → bp.pairs        │
//    │  3. contacts.clear()             — drop last frame's contacts       │
//    │  4. run_narrow_phase()           — GJK/EPA dispatch → ContactList   │
//    │                                                                     │
//    │  ┌─ substep × N ──────────────────────────────────────────────────┐ │
//    │  │  5.  update_position_and_orientation()                         │ │
//    │  │  6a. solve_contacts_position_level()   ─┐                      │ │
//    │  │  6b. compute_joint_errors()             ├─ solve_positions()   │ │
//    │  │  6c. solve_constraints()               ─┘                      │ │
//    │  │  7.  update_velocities()                                       │ │
//    │  │  8a. solve_contacts_velocity_level()  ─┐                       │ │
//    │  │  8b. apply_joint_damping()             ├─ solve_velocities()   │ │
//    │  └────────────────────────────────────────┘                       │ │
//    └─────────────────────────────────────────────────────────────────────┘
//
//  WHY broad+narrow outside the substep loop?
//    Running narrow phase once per frame and reusing the same contact set
//    across substeps (resetting lambdas each frame, NOT each substep) is
//    standard XPBD practice.  It avoids N×GJK/EPA calls per frame while
//    the position-level solver incrementally refines the same contacts.
//
//    If you need higher accuracy for fast-moving objects, move
//    run_narrow_phase() inside the substep loop and call contacts.clear()
//    at the top of each substep — at a proportional CPU cost.
// ============================================================================

namespace rbps
{
    struct World
    {
        scalar timestep = 1.0 / 60.0;
        int substeps = 20;

        BodyCollection bodies;
        ConstraintCollection constraints;
        JointCollection joints;
        ContactList contacts; // unified pipeline output + solver state
        ColliderCollection colliders;
        rbc::BroadPhaseState broad_phase_state;

        // -----------------------------------------------------------------
        World() = default;

        World(scalar timestep_, int substeps_)
            : timestep(timestep_), substeps(substeps_)
        {
        }

        // -----------------------------------------------------------------
        //  Creation helpers
        // -----------------------------------------------------------------

        uint32_t create_body(BodyParams params)
        {
            return rbps::create_body(bodies, params);
        }

        uint32_t create_collider(ColliderParams params)
        {
            // Build the initial world transform from the owning body so the
            // broad phase AABB is correct from the first frame.
            const uint32_t body_slot = bodies.index_of(params.body_id);
            const m3d::vec3 body_pos = bodies.position[body_slot];
            const m3d::quat body_rot = bodies.orientation[body_slot];
            const m3d::tf body_tf{body_pos, body_rot};

            return rbps::create_collider(colliders, broad_phase_state, params, body_tf);
        }

        uint32_t create_constraint(ConstraintParams params)
        {
            return rbps::create_constraint(constraints, params);
        }

        uint32_t create_prismatic_joint(PrismaticJointParams params)
        {
            return rbps::create_prismatic_joint(joints, constraints, params);
        }

        uint32_t create_revolute_joint(RevoluteJointParams params)
        {
            return rbps::create_revolute_joint(joints, constraints, params);
        }

        uint32_t create_fixed_joint(FixedJointParams params)
        {
            return rbps::create_fixed_joint(joints, constraints, params);
        }

        // -----------------------------------------------------------------
        //  Substep solvers (called N times per frame)
        // -----------------------------------------------------------------

        // solve_positions: uses the contact set built once at frame start.
        // Lambdas accumulate across substeps — do NOT clear contacts here.
        void solve_positions(scalar inv_h, scalar h)
        {
            solve_contacts_position_level(contacts, bodies, inv_h);
            compute_joint_errors(joints, bodies, constraints, h);
            solve_constraints(bodies, constraints, inv_h);
        }

        void solve_velocities(scalar h)
        {
            solve_contacts_velocity_level(contacts, bodies, h);
            apply_joint_damping(joints, bodies, h);
        }

        // -----------------------------------------------------------------
        //  Main step
        // -----------------------------------------------------------------
        void step()
        {
            const scalar h = timestep / substeps;
            const scalar inv_h = 1.0 / h;

            // ── FRAME-LEVEL: broad phase + narrow phase (once per frame) ──

            // 1. Sweep dynamic body AABBs (velocity expansion optional).
            update_broad_phase_aabbs(colliders, broad_phase_state,
                                     bodies, timestep, /*use_velocity_expansion=*/true);

            // 2. SAP sort + sweep → broad_phase_state.pairs
            rbc::broad_phase_update(broad_phase_state);
            contacts.clear();
            contacts.reserve(broad_phase_state.pairs.size() * 4); // rough pre-alloc
            
            for (int i = 0; i < substeps; ++i)
            {
                // 4. Narrow phase dispatch → contacts
                rbps::run_narrow_phase(broad_phase_state, colliders, bodies, contacts);
                update_position_and_orientation(bodies, h);
                solve_positions(inv_h, h);
                update_velocities(bodies, inv_h);
                solve_velocities(h);
            }
        }
    };

} // namespace rbps