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
        // -------------------------------------------------------------------------
    //  ContactList — unified SoA for both the pipeline output AND the solver.
    //
    //  This replaces the old ContactCollection. Fields are grouped by lifecycle:
    //    • Pipeline output  — filled by run_narrow_phase(), read-only during solve
    //    • Solver state     — zeroed at frame start, written every substep
    //
    //  Body indices vs IDs
    //  -------------------
    //  body_a / body_b store raw BodyCollection *slot indices*, NOT user-facing
    //  IDs.  The pipeline converts at emit time (emit_contact receives slot
    //  indices from collider_world_tf lookups).  The solver can therefore index
    //  bc.position[body_a[i]] directly with no extra lookup.
    // -------------------------------------------------------------------------
    struct ContactList
    {
        uint32_t n_contacts = 0;
 
        // ── Frame data ────────────────────────────────────────────────────
        std::vector<uint32_t>    body_a;
        std::vector<uint32_t>    body_b;
        std::vector<uint32_t>    collider_a;
        std::vector<uint32_t>    collider_b;
 
        // Lever arms in body-LOCAL space — set once by run_narrow_phase,
        // used every substep to reconstruct world-space contact points.
        std::vector<m3d::vec3>   r_a_local;
        std::vector<m3d::vec3>   r_b_local;
 
        // Contact normal at detection time (world space, unit length).
        // Reused each substep — valid as long as bodies haven't rotated far,
        // which is guaranteed by small substep sizes.
        std::vector<m3d::vec3>   normal;
 
        // Material mix
        std::vector<m3d::scalar> restitution;
        std::vector<m3d::scalar> static_friction;
        std::vector<m3d::scalar> dynamic_friction;
 
        // Lagrange multipliers — zeroed at frame start, accumulate per substep
        std::vector<m3d::scalar> normal_lambda;
        std::vector<m3d::scalar> tangent_lambda;
 
        // ── Substep data (recomputed inside apply_constraint_position_level) ─
        std::vector<m3d::vec3>   point_on_a;        // rebuilt from r_a_local each substep
        std::vector<m3d::vec3>   point_on_b;        // rebuilt from r_b_local each substep
        std::vector<m3d::scalar> penetration_depth;
        std::vector<bool>        collision;
        std::vector<m3d::scalar> relative_velocity;
        std::vector<m3d::vec3>   normal_force;
        std::vector<m3d::vec3>   tangent_force;
        std::vector<bool> use_dynamic_friction;
 
        void reserve(size_t n)
        {
            body_a            .reserve(n);
            body_b            .reserve(n);
            collider_a        .reserve(n);
            collider_b        .reserve(n);
            r_a_local         .reserve(n);
            r_b_local         .reserve(n);
            normal            .reserve(n);
            restitution       .reserve(n);
            static_friction   .reserve(n);
            dynamic_friction  .reserve(n);
            normal_lambda     .reserve(n);
            tangent_lambda    .reserve(n);
            point_on_a        .reserve(n);
            point_on_b        .reserve(n);
            penetration_depth .reserve(n);
            collision         .reserve(n);
            relative_velocity .reserve(n);
            normal_force      .reserve(n);
            tangent_force     .reserve(n);
            use_dynamic_friction.reserve(n);
        }
 
        // Call ONCE per frame — resets everything including lambdas.
        void clear()
        {
            n_contacts = 0;
            body_a            .clear();
            body_b            .clear();
            collider_a        .clear();
            collider_b        .clear();
            r_a_local         .clear();
            r_b_local         .clear();
            normal            .clear();
            restitution       .clear();
            static_friction   .clear();
            dynamic_friction  .clear();
            normal_lambda     .clear();
            tangent_lambda    .clear();
            point_on_a        .clear();
            point_on_b        .clear();
            penetration_depth .clear();
            collision         .clear();
            relative_velocity .clear();
            normal_force      .clear();
            tangent_force     .clear();
            use_dynamic_friction.clear();
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
    
    // -------------------------------------------------------------------------
    //  Step 2 — Detect contacts from broad phase pairs (call once per frame)
    //
    //  Runs GJK/EPA for each candidate pair, emits ContactList entries with
    //  local-space lever arms and zero lambdas.  Does NOT fill world-space
    //  substep data (that is done by refresh_contacts each substep).
    // -------------------------------------------------------------------------
    void run_narrow_phase(const rbc::BroadPhaseState    &bp,
                                 const rbps::ColliderCollection  &cc,
                                 const BodyCollection           &bc,
                                 ContactList                    &out);

    // -------------------------------------------------------------------------
    //  Step 3 — Refresh per-substep contact geometry (call every substep)
    //
    //  Reconstructs world-space contact points from stored local-space lever
    //  arms and the bodies' current transforms.  Updates normal, point_on_a/b,
    //  penetration_depth.  Does NOT touch lambdas.
    //
    //  Implements eq. (26) from Müller et al.:
    //    p1 = x1 + rotate(q1, r1_local)
    //    p2 = x2 + rotate(q2, r2_local)
    //    d  = dot(p1 - p2, n)
    // -------------------------------------------------------------------------
    void refresh_contacts(ContactList          &cl,
                          const BodyCollection &bc);

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