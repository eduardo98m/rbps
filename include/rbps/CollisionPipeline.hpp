#pragma once
#include <vector>
#include "rbps/Collider.hpp"
#include "rbc/BroadPhase.hpp"
#include "rbc/Contact.hpp"
#include "rbps/Body.hpp"

/**
 * @file CollisionPipeline.hpp
 * @brief Glues `rbc` (geometry) to `rbps` (bodies); per-frame contact pipeline.
 * @ingroup rbps
 *
 * @par Stages (per call to `run_collision_pipeline`)
 * 1. Update world-space AABBs from body transforms (broad-phase feed).
 * 2. SAP broad phase (candidate pairs).
 * 3. Static–static pair filter (free cull).
 * 4. Narrow phase via `rbc::dispatch` (exact contacts).
 * 5. Emit `ContactList` rows (drives the constraint solver).
 *
 * @par Why does this live in `rbps`?
 * `rbc` knows nothing about bodies, velocities, or `BodyType`. The moment
 * code reads `bc.linear_velocity` or `bc.type` it is in `rbps` territory.
 *
 * @par Graph-coloring hook
 * `get_collision_groups` returns independent contact groups for parallel
 * constraint solving — see the implementation in CollisionPipeline.cpp.
 */

namespace rbps
{
    /**
     * @brief Unified SoA carrying the pipeline output AND the solver scratch state.
     *
     * Replaces the old `ContactCollection`. Fields are grouped by lifecycle:
     * - **Pipeline output** — filled by `run_narrow_phase`, read-only during the solve.
     * - **Solver state**    — zeroed at frame start, written every substep.
     *
     * @par Body indices vs IDs
     * `body_a` / `body_b` store raw `BodyCollection` *slot indices*, not
     * user-facing IDs. The pipeline does the conversion at emit time, so
     * the solver can index `bc.position[body_a[i]]` directly without an
     * extra ID-to-index lookup.
     *
     * @ingroup rbps
     */
    struct ContactList
    {
        uint32_t n_contacts = 0; ///< Live entry count; arrays may have larger reserved capacity.

        // ── Frame data ────────────────────────────────────────────────────
        std::vector<uint32_t>    body_a;     ///< Slot index of body A in `BodyCollection`.
        std::vector<uint32_t>    body_b;     ///< Slot index of body B in `BodyCollection`.
        std::vector<uint32_t>    collider_a; ///< Slot index of collider A in `ColliderCollection`.
        std::vector<uint32_t>    collider_b; ///< Slot index of collider B in `ColliderCollection`.

        /// Lever arm in body-A LOCAL space; used each substep to rebuild the world-space contact point.
        std::vector<m3d::vec3>   r_a_local;
        /// Lever arm in body-B LOCAL space; used each substep to rebuild the world-space contact point.
        std::vector<m3d::vec3>   r_b_local;

        /// Contact normal at detection time (world space, unit length).
        /// Reused across substeps — valid as long as bodies haven't rotated far.
        std::vector<m3d::vec3>   normal;

        // Material mix
        std::vector<m3d::scalar> restitution;       ///< Per-contact restitution coefficient.
        std::vector<m3d::scalar> static_friction;   ///< Per-contact static-friction coefficient.
        std::vector<m3d::scalar> dynamic_friction;  ///< Per-contact dynamic-friction coefficient.

        // Lagrange multipliers — zeroed at frame start, accumulated per substep.
        std::vector<m3d::scalar> normal_lambda;     ///< Accumulated normal-impulse magnitude.
        std::vector<m3d::scalar> tangent_lambda;    ///< Accumulated friction-impulse magnitude.

        // ── Substep data (recomputed inside `apply_constraint_position_level`) ─
        std::vector<m3d::vec3>   point_on_a;        ///< World-space contact point on A; rebuilt from `r_a_local`.
        std::vector<m3d::vec3>   point_on_b;        ///< World-space contact point on B; rebuilt from `r_b_local`.
        std::vector<m3d::scalar> penetration_depth; ///< Current penetration along `normal`.
        std::vector<bool>        collision;         ///< False after bodies separate during the solve.
        std::vector<m3d::scalar> relative_velocity; ///< Pre-solve normal relative velocity (used by velocity-level restitution).
        std::vector<m3d::vec3>   normal_force;      ///< Reported normal force (debug / sensors).
        std::vector<m3d::vec3>   tangent_force;     ///< Reported friction force (debug / sensors).
        std::vector<bool>        use_dynamic_friction; ///< Set when slipping exceeds the static-friction cone.

        /** @brief Pre-allocate space for `n` contact rows in every field. */
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

        /**
         * @brief Reset every field; call ONCE per frame.
         *
         * Resets the accumulated `normal_lambda` / `tangent_lambda` too —
         * which is why this must NOT be called between substeps.
         */
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


    /**
     * @brief Tunables for the per-frame collision pipeline.
     * @ingroup rbps
     */
    struct CollisionPipelineConfig
    {
        /**
         * @brief When `true`, AABB expansion adds `velocity * dt` (anti-tunnelling).
         *
         * When `false`, only the static `fat_margin` of the broad phase is used.
         */
        bool use_velocity_expansion = true;
    };

    /**
     * @brief Step 1 — refresh the broad-phase AABBs.
     *
     * Static colliders are inserted once at startup and never moved. Dynamic
     * colliders are updated every tick, optionally with velocity-swept
     * AABB expansion (`use_velocity_expansion`).
     *
     * @ingroup rbps
     */
    void update_broad_phase_aabbs(ColliderCollection &cc,
                                  rbc::BroadPhaseState &bp,
                                  const BodyCollection &bc,
                                  m3d::scalar dt,
                                  bool use_velocity_expansion);

    /**
     * @brief Steps 2–4 — full pipeline (broad phase → static cull → narrow phase).
     *
     * Clears and refills `contacts_out` each call.
     *
     * @ingroup rbps
     */
    void run_collision_pipeline(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                const BodyCollection &bc,
                                m3d::scalar dt,
                                ContactList &contacts_out,
                                const CollisionPipelineConfig &cfg = {});

    /**
     * @brief Step 2 — narrow-phase pass over the broad-phase pair list.
     *
     * Runs GJK/EPA (or analytic algorithms via the dispatcher) for each
     * candidate pair, emits `ContactList` rows with body-LOCAL lever arms
     * and zero lambdas. Does NOT fill the world-space substep data —
     * that is done by `refresh_contacts` at the start of each substep.
     *
     * @ingroup rbps
     */
    void run_narrow_phase(const rbc::BroadPhaseState    &bp,
                          const rbps::ColliderCollection  &cc,
                          const BodyCollection           &bc,
                          ContactList                    &out);

    /**
     * @brief Step 3 — refresh per-substep contact geometry.
     *
     * Reconstructs world-space contact points from stored body-LOCAL lever
     * arms and the bodies' current transforms. Updates `point_on_a`,
     * `point_on_b`, and `penetration_depth`. Does NOT touch the lambdas.
     *
     * Implements eq. (26) of Müller et al. (XPBD):
     * - `p1 = x1 + rotate(q1, r1_local)`
     * - `p2 = x2 + rotate(q2, r2_local)`
     * - `d  = dot(p1 - p2, n)`
     *
     * @ingroup rbps
     */
    void refresh_contacts(ContactList          &cl,
                          const BodyCollection &bc);

    /**
     * @brief Greedy graph coloring → independent contact groups for parallel solving.
     *
     * Each returned inner vector contains contact indices that share NO
     * body in common, so they can be solved in parallel. Algorithm:
     * `O(n + e)` greedy coloring where contacts are nodes and edges
     * connect contacts that share a body. Each color = one solve wave.
     *
     * Call after `run_collision_pipeline` / `run_narrow_phase`.
     *
     * @ingroup rbps
     */
    std::vector<std::vector<u_int32_t>>
    get_collision_groups(const ContactList &contacts,
                         const BodyCollection &bc);

} // namespace rbps
