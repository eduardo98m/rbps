#pragma once
#include <vector>
#include "rbps/Collider.hpp"
#include "rbc/BroadPhase.hpp"
#include "rbc/Contact.hpp"
#include "rbps/Body.hpp"

// ============================================================================
//  rbps/CollisionPipeline.hpp
//
//  Ties rbc (geometry) to rbps (bodies) and runs the full pipeline:
//
//    1. Update world-space AABBs from body transforms      (broad phase feed)
//    2. SAP broad phase                                    (candidate pairs)
//    3. Static-static pair filter                          (free cull)
//    4. Narrow phase via rbc::dispatch()                   (exact contacts)
//    5. ContactList output                                 (feeds constraint solver)
//
//  WHY rbps, NOT rbc?
//    rbc knows nothing about bodies, velocities, or BodyType.
//    The moment we read bc.linear_velocity or bc.type we are in rbps territory.
//
//  GRAPH COLORING HOOK:
//    get_collision_groups() is declared below and returns independent contact
//    groups for parallel constraint solving.  See CollisionPipeline.cpp for
//    the greedy graph coloring implementation carried over from your old code.
// ============================================================================

namespace rbps
{
    // -----------------------------------------------------------------------
    //  ContactList — SoA output of the pipeline, feeds directly into the
    //  constraint solver. Mirrors your old ContactCollection layout.
    // -----------------------------------------------------------------------
    struct ContactList
    {
        uint32_t n_contacts = 0;

        // Which bodies are involved
        std::vector<uint32_t> body_a;
        std::vector<uint32_t> body_b;

        // Which colliders produced this contact (for post-solve pose update)
        std::vector<uint32_t> collider_a;
        std::vector<uint32_t> collider_b;

        // Contact geometry
        std::vector<m3d::vec3> normal;     // from b toward a (unit)
        std::vector<m3d::vec3> point_on_a; // world-space contact point on body A
        std::vector<m3d::vec3> point_on_b; // world-space contact point on body B
        std::vector<m3d::scalar> penetration_depth;

        // Material mix (averaged / min of the two colliders)
        std::vector<m3d::scalar> restitution;
        std::vector<m3d::scalar> static_friction;
        std::vector<m3d::scalar> dynamic_friction;

        // Solver state (initialised to zero, filled during solve)
        std::vector<m3d::scalar> normal_lambda;
        std::vector<m3d::scalar> tangent_lambda;
        std::vector<m3d::vec3> normal_force;
        std::vector<m3d::vec3> tangent_force;

        void reserve(size_t n)
        {
            body_a.reserve(n);
            body_b.reserve(n);
            collider_a.reserve(n);
            collider_b.reserve(n);
            normal.reserve(n);
            point_on_a.reserve(n);
            point_on_b.reserve(n);
            penetration_depth.reserve(n);
            restitution.reserve(n);
            static_friction.reserve(n);
            dynamic_friction.reserve(n);
            normal_lambda.reserve(n);
            tangent_lambda.reserve(n);
            normal_force.reserve(n);
            tangent_force.reserve(n);
        }

        void clear()
        {
            n_contacts = 0;
            body_a.clear();
            body_b.clear();
            collider_a.clear();
            collider_b.clear();
            normal.clear();
            point_on_a.clear();
            point_on_b.clear();
            penetration_depth.clear();
            restitution.clear();
            static_friction.clear();
            dynamic_friction.clear();
            normal_lambda.clear();
            tangent_lambda.clear();
            normal_force.clear();
            tangent_force.clear();
        }
    };

    // -----------------------------------------------------------------------
    //  CollisionPipelineConfig
    // -----------------------------------------------------------------------
    struct CollisionPipelineConfig
    {
        // If true, AABB expansion uses velocity * dt (anti-tunnelling).
        // If false, only the static fat_margin is used.
        bool use_velocity_expansion = true;
    };

    // -----------------------------------------------------------------------
    //  Step 1 — Update broad phase AABBs
    //
    //  Static bodies: their AABB is inserted once at startup and never moved.
    //  Dynamic bodies: update every tick, optionally with velocity expansion.
    // -----------------------------------------------------------------------
    void update_broad_phase_aabbs(ColliderCollection &cc,
                                  rbc::BroadPhaseState &bp,
                                  const BodyCollection &bc,
                                  m3d::scalar dt,
                                  bool use_velocity_expansion);

    // -----------------------------------------------------------------------
    //  Step 2+3+4 — Full pipeline (broad + narrow → ContactList)
    //
    //  Clears and refills `contacts_out` each call.
    //  Optionally returns the broad phase pair count for profiling.
    // -----------------------------------------------------------------------
    void run_collision_pipeline(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                const BodyCollection &bc,
                                m3d::scalar dt,
                                ContactList &contacts_out,
                                const CollisionPipelineConfig &cfg = {});
    
    
    static void run_narrow_phase(const rbc::BroadPhaseState    &bp,
                                 const rbps::ColliderCollection  &cc,
                                 const BodyCollection           &bc,
                                 ContactList                    &out);

    // -----------------------------------------------------------------------
    //  Graph coloring — independent contact groups for parallel solving.
    //
    //  Returns a list of groups, where each group contains contact indices
    //  that share NO body in common and can therefore be solved in parallel.
    //
    //  Algorithm: greedy graph coloring (O(n + e), same as your old code).
    //  Contacts are nodes; an edge exists when two contacts share a body.
    //  Color = solve-wave index. Each color forms one independent group.
    //
    //  Call this after run_collision_pipeline().
    // -----------------------------------------------------------------------
    std::vector<std::vector<u_int32_t>>
    get_collision_groups(const ContactList &contacts,
                         const BodyCollection &bc);

} // namespace rbps