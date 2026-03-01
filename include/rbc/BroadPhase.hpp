#pragma once
#include <vector>
#include <cstdint>
#include <limits>
#include "rbc/AABB.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

namespace rbc
{
    // =========================================================================
    //  BROAD PHASE — Sort and Sweep (SAP) Algorithm Aka Sweep and Prune
    // =========================================================================
    //
    //  ALGORITHM OVERVIEW
    //  ------------------
    //  Sort and Sweep projects each AABB onto one axis (here, X) and maintains
    //  a sorted list of interval endpoints [min_i, max_i].  A sweep through the
    //  list with an "active set" finds all pairs whose X-intervals overlap in
    //  O(n + k) time after the sort, where k is the number of overlapping pairs.
    //
    //  Each candidate pair is then verified against the full 3-D AABB test to
    //  discard false positives from the 1-axis projection.
    //
    //  WHY SAP OVER OCTREES / BVH?
    //  ----------------------------
    //   - Octrees require expensive node re-insertion every frame for moving objects.
    //   - BVH (dynamic AABB tree) is excellent but adds per-node pointer overhead.
    //   - SAP is a flat sorted array — cache-coherent, simple, allocation-free
    //     at steady state, and nearly O(n) per frame when objects move little
    //     (insertion sort is O(n) on nearly-sorted data).
    //   - SAP is the algorithm used internally by PhysX and Havok.
    //
    //  TEMPORAL COHERENCE
    //  ------------------
    //  AABBs are "fattened" by a configurable margin.  A moving object only
    //  triggers a AABB update (and a re-sort) when it leaves its fattened AABB,
    //  not every frame.  This keeps the sorted list nearly-sorted between frames,
    //  which is exactly the best case for insertion sort.
    //
    //  DATA LAYOUT (data-driven, no virtual dispatch)
    //  -----------------------------------------------
    //    BroadPhaseState  — owns all allocations
    //    BPHandle         — opaque index returned by broad_phase_insert()
    //    BPObject         — per-object AABB + user ID
    //    SAPEndpoint      — one entry per AABB endpoint (2 per object)
    //    BroadPhasePair   — output pair of user IDs
    // =========================================================================

    // Opaque handle for a registered object. Never store raw indices; use these.
    using BPHandle = uint32_t;
    static constexpr BPHandle BP_INVALID_HANDLE = std::numeric_limits<uint32_t>::max();

    // Output: a pair of user IDs that may be colliding (needs narrow phase).
    struct BroadPhasePair
    {
        uint32_t id_a;
        uint32_t id_b;
    };

    // Internal: one sorted endpoint per AABB edge on the projection axis.
    // Packed into 8 bytes: 4-byte scalar + 4-byte bitfield.
    struct alignas(8) SAPEndpoint
    {
        float    value;        // projected value on the sweep axis
        uint32_t slot  : 31;  // index into BroadPhaseState::objects
        uint32_t is_max : 1;  // 0 = min endpoint, 1 = max endpoint
    };
    static_assert(sizeof(SAPEndpoint) == 8, "SAPEndpoint size check");

    // Internal: per-registered-object record.
    struct BPObject
    {
        AABB     fat_aabb;     // fattened AABB actually stored in the sorted list
        AABB     tight_aabb;   // last tight AABB provided by the caller
        uint32_t user_id;      // opaque ID provided by the caller
        bool     alive;        // false = slot is in free_list, available for reuse
    };

    // Configurable parameters for the broad phase.
    struct BroadPhaseConfig
    {
        // How much to fatten AABBs (world-space units).
        // Larger = fewer updates but more false positives in the narrow phase.
        m3d::scalar fat_margin = 0.1;

        // If an object's tight AABB grows beyond its current fat AABB,
        // we reinsert. `reinsert_threshold` is an extra slack so small
        // oscillations don't trigger constant reinsertion.
        m3d::scalar reinsert_threshold = 0.05;
    };

    // The full broad phase state. Treat as opaque; mutate only through the API.
    struct BroadPhaseState
    {
        BroadPhaseConfig          config;
        std::vector<BPObject>     objects;      // slot array (may have gaps)
        std::vector<uint32_t>     free_list;    // recycled slot indices
        std::vector<SAPEndpoint>  endpoints;    // sorted on `value` (X axis)
        std::vector<uint32_t>     active_set;   // scratch buffer for sweep
        std::vector<BroadPhasePair> pairs;      // OUTPUT — filled each broad_phase_update()
        bool                      dirty;        // true = full re-sort needed
    };

    // -------------------------------------------------------------------------
    //  API
    // -------------------------------------------------------------------------

    // Initialise a BroadPhaseState with optional config.
    void broad_phase_init(BroadPhaseState &bp,
                          const BroadPhaseConfig &cfg = BroadPhaseConfig{});

    // Register an object. `user_id` is whatever the caller wants (body index, etc.)
    // Returns a BPHandle used for future move/remove calls.
    BPHandle broad_phase_insert(BroadPhaseState &bp,
                                uint32_t         user_id,
                                const AABB      &tight_aabb);

    // Unregister an object. The handle is invalidated.
    void broad_phase_remove(BroadPhaseState &bp, BPHandle h);

    // Notify that an object has moved. Internally checks whether the tight AABB
    // is still contained in the fat AABB; only triggers a re-sort if it escaped.
    void broad_phase_move(BroadPhaseState &bp,
                          BPHandle         h,
                          const AABB      &new_tight_aabb);

    // Run the broad phase. Fills bp.pairs with all potentially-colliding pairs.
    // Call once per frame after all broad_phase_move() calls.
    void broad_phase_update(BroadPhaseState &bp);

    // Convenience: compute an AABB from a shape + transform and call broad_phase_move.
    inline void broad_phase_move_shape(BroadPhaseState &bp,
                                       BPHandle         h,
                                       const Shape     &shape,
                                       const m3d::tf   &tf)
    {
        broad_phase_move(bp, h, compute_aabb(shape, tf));
    }

    // -------------------------------------------------------------------------
    //  Velocity-expanded move (anti-tunnelling)
    //
    //  Expands the AABB *asymmetrically* per axis based on linear velocity and
    //  the physics timestep. Ensures the stored fat AABB covers the entire volume
    //  the object will sweep through before the next broad_phase_update() call.
    //
    //  Per-axis rule:
    //    vel > 0  →  max side expands by vel*dt,    min side by fat_margin
    //    vel < 0  →  min side expands by |vel|*dt,  max side by fat_margin
    //    vel == 0 →  both sides expand by fat_margin  (same as standard move)
    //
    //  Temporal coherence is preserved: broad_phase_move() is still called
    //  internally, so re-sorts are skipped if the swept AABB fits inside the
    //  previously stored fat_aabb.
    //
    //  Usage:
    //    broad_phase_move_swept(bp, handle, tight_aabb, body.linear_vel, dt);
    // -------------------------------------------------------------------------
    inline void broad_phase_move_swept(BroadPhaseState &bp,
                                       BPHandle         h,
                                       const AABB      &tight_aabb,
                                       const m3d::vec3 &linear_velocity,
                                       m3d::scalar      dt)
    {
        const m3d::scalar margin = bp.config.fat_margin;
        const m3d::vec3   dv     = linear_velocity * dt;

        // Expand min side where velocity is negative, max side where positive.
        const m3d::vec3 expand_min(
            margin + (dv.x < 0 ? -dv.x : m3d::scalar(0)),
            margin + (dv.y < 0 ? -dv.y : m3d::scalar(0)),
            margin + (dv.z < 0 ? -dv.z : m3d::scalar(0)));

        const m3d::vec3 expand_max(
            margin + (dv.x > 0 ?  dv.x : m3d::scalar(0)),
            margin + (dv.y > 0 ?  dv.y : m3d::scalar(0)),
            margin + (dv.z > 0 ?  dv.z : m3d::scalar(0)));

        broad_phase_move(bp, h, AABB{tight_aabb.min - expand_min,
                                     tight_aabb.max + expand_max});
    }

    // Convenience: shape + transform + velocity.
    inline void broad_phase_move_shape_swept(BroadPhaseState &bp,
                                              BPHandle         h,
                                              const Shape     &shape,
                                              const m3d::tf   &tf,
                                              const m3d::vec3 &linear_velocity,
                                              m3d::scalar      dt)
    {
        broad_phase_move_swept(bp, h, compute_aabb(shape, tf), linear_velocity, dt);
    }

    // -------------------------------------------------------------------------
    //  Internal helpers (exposed for testing; not part of the public API)
    // -------------------------------------------------------------------------

    // Insertion sort — O(n) when nearly sorted (the common case each frame).
    void bp_insertion_sort(std::vector<SAPEndpoint> &endpoints);

    // Sweep the sorted endpoint list and fill bp.pairs.
    void bp_sweep(BroadPhaseState &bp);

} // namespace rbc