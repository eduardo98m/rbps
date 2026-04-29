#pragma once
#include <vector>
#include <cstdint>
#include <limits>
#include "rbc/AABB.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

/**
 * @file BroadPhase.hpp
 * @brief Sweep-and-Prune (SAP) broad-phase data structures and API.
 * @ingroup rbc
 *
 * @par Algorithm — Sort and Sweep
 * Each AABB is projected onto one axis (here, X) and a sweep through the
 * sorted endpoint list with an active set finds all pairs whose
 * X-intervals overlap in `O(n + k)` time after the sort, where `k` is the
 * number of overlapping pairs. Each candidate pair is then verified with
 * the full 3-D AABB test to discard false positives from the 1-axis
 * projection.
 *
 * @par Why SAP over octrees / BVH?
 * - Octrees require expensive node re-insertion every frame for moving
 *   objects.
 * - BVH (dynamic AABB tree) is excellent but adds per-node pointer
 *   overhead.
 * - SAP is a flat sorted array — cache-coherent, simple, allocation-free
 *   at steady state, and nearly `O(n)` per frame when objects move little
 *   (insertion sort is `O(n)` on nearly-sorted data).
 * - SAP is the broad phase used internally by PhysX and Havok.
 *
 * @par Temporal coherence
 * AABBs are "fattened" by a configurable margin. A moving object only
 * triggers an AABB update (and a re-sort) when it leaves its fattened
 * AABB, not every frame. This keeps the sorted list nearly-sorted between
 * frames — best case for insertion sort.
 */

namespace rbc
{

    /**
     * @brief Opaque handle for a registered broad-phase object.
     *
     * Returned by `broad_phase_insert`; passed to `broad_phase_move` /
     * `broad_phase_remove`. Handles are not stable IDs — they refer to a
     * slot inside `BroadPhaseState::objects` and are recycled after
     * `broad_phase_remove`.
     *
     * @ingroup rbc
     */
    using BPHandle = uint32_t;

    /** @brief Sentinel "no object" value for `BPHandle`. @ingroup rbc */
    static constexpr BPHandle BP_INVALID_HANDLE = std::numeric_limits<uint32_t>::max();

    /**
     * @brief A pair of user IDs that the broad phase flagged as potentially overlapping.
     *
     * Output of `broad_phase_update`. Each pair must still pass the
     * narrow phase before becoming a real contact.
     *
     * @ingroup rbc
     */
    struct BroadPhasePair
    {
        uint32_t id_a; ///< User ID of the first object.
        uint32_t id_b; ///< User ID of the second object.
    };

    /**
     * @brief Internal: one sorted endpoint per AABB edge on the projection axis.
     *
     * Packed into 8 bytes so the sorted endpoint array stays cache-friendly.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    struct alignas(8) SAPEndpoint
    {
        float    value;        ///< Projected coordinate on the sweep axis.
        uint32_t slot  : 31;   ///< Index into `BroadPhaseState::objects`.
        uint32_t is_max : 1;   ///< 0 = min endpoint, 1 = max endpoint.
    };
    static_assert(sizeof(SAPEndpoint) == 8, "SAPEndpoint size check");

    /**
     * @brief Internal: per-registered-object record stored in `BroadPhaseState::objects`.
     * @ingroup rbc
     * @ingroup internals
     */
    struct BPObject
    {
        AABB     fat_aabb;    ///< Fattened AABB actually in the sorted list.
        AABB     tight_aabb;  ///< Last tight AABB provided by the caller.
        uint32_t user_id;     ///< Opaque ID provided by the caller.
        bool     alive;       ///< False if the slot is in the free list.
    };

    /**
     * @brief Broad-phase tuning parameters.
     *
     * `fat_margin` is the dominant knob: too small and every motion
     * triggers a re-sort; too large and the narrow phase wastes work on
     * false positives.
     *
     * @ingroup rbc
     */
    struct BroadPhaseConfig
    {
        /**
         * @brief How much to fatten AABBs (world-space units).
         *
         * Larger ⇒ fewer updates but more false positives in the narrow phase.
         */
        m3d::scalar fat_margin = 0.1;

        /**
         * @brief Extra slack before a re-insert is triggered.
         *
         * If an object's tight AABB grows beyond its current fat AABB by
         * more than this threshold, the broad phase reinserts it. Prevents
         * small oscillations from causing constant re-sorts.
         */
        m3d::scalar reinsert_threshold = 0.05;
    };

    /**
     * @brief All state owned by the broad phase. Treat as opaque; mutate only via the API.
     *
     * @ingroup rbc
     */
    struct BroadPhaseState
    {
        BroadPhaseConfig            config;     ///< Tuning parameters.
        std::vector<BPObject>       objects;    ///< Slot array (may have gaps, see `free_list`).
        std::vector<uint32_t>       free_list;  ///< Indices of recycled slots in `objects`.
        std::vector<SAPEndpoint>    endpoints;  ///< Sorted endpoint list (sorted on `value`, X axis).
        std::vector<uint32_t>       active_set; ///< Scratch buffer reused by the sweep.
        std::vector<BroadPhasePair> pairs;      ///< Output — refilled by every `broad_phase_update`.
        bool                        dirty;      ///< True ⇒ next update needs a full re-sort.
    };

    // -------------------------------------------------------------------------
    //  API
    // -------------------------------------------------------------------------

    /**
     * @brief Initialise a `BroadPhaseState` with the given config.
     * @ingroup rbc
     */
    void broad_phase_init(BroadPhaseState &bp,
                          const BroadPhaseConfig &cfg = BroadPhaseConfig{});

    /**
     * @brief Register a new object.
     *
     * @param bp          Broad-phase state.
     * @param user_id     Opaque ID the caller will use to recognise this
     *                    object in the `BroadPhasePair` output.
     * @param tight_aabb  The object's current (un-fattened) AABB.
     * @return Handle used for future `broad_phase_move` / `broad_phase_remove` calls.
     *
     * @ingroup rbc
     */
    BPHandle broad_phase_insert(BroadPhaseState &bp,
                                uint32_t         user_id,
                                const AABB      &tight_aabb);

    /**
     * @brief Unregister an object. The handle is invalidated; its slot is recycled.
     * @ingroup rbc
     */
    void broad_phase_remove(BroadPhaseState &bp, BPHandle h);

    /**
     * @brief Notify the broad phase that an object has moved.
     *
     * Internally checks whether the new tight AABB is still contained in
     * the previously-fattened AABB. Only triggers a re-insert (and a
     * re-sort on the next update) if the tight AABB escaped.
     *
     * @ingroup rbc
     */
    void broad_phase_move(BroadPhaseState &bp,
                          BPHandle         h,
                          const AABB      &new_tight_aabb);

    /**
     * @brief Run the broad phase. Refills `bp.pairs` with all potential contact pairs.
     *
     * Call once per frame after every `broad_phase_move` for that frame.
     *
     * @ingroup rbc
     */
    void broad_phase_update(BroadPhaseState &bp);

    /**
     * @brief Convenience: compute an AABB from `(shape, tf)` and call `broad_phase_move`.
     * @ingroup rbc
     */
    inline void broad_phase_move_shape(BroadPhaseState &bp,
                                       BPHandle         h,
                                       const Shape     &shape,
                                       const m3d::tf   &tf)
    {
        broad_phase_move(bp, h, compute_aabb(shape, tf));
    }

    /**
     * @brief Velocity-expanded move (anti-tunnelling).
     *
     * Expands the AABB *asymmetrically* per axis based on linear velocity
     * and the physics timestep so the stored fat AABB covers the entire
     * volume the object will sweep through before the next
     * `broad_phase_update` call.
     *
     * @par Per-axis rule
     * - `vel > 0`  → max side expands by `vel*dt`,  min side by `fat_margin`
     * - `vel < 0`  → min side expands by `|vel|*dt`, max side by `fat_margin`
     * - `vel == 0` → both sides expand by `fat_margin` (same as standard move)
     *
     * Temporal coherence is preserved: `broad_phase_move` is still called
     * internally, so re-sorts are skipped if the swept AABB fits inside
     * the previously stored `fat_aabb`.
     *
     * @ingroup rbc
     */
    inline void broad_phase_move_swept(BroadPhaseState &bp,
                                       BPHandle         h,
                                       const AABB      &tight_aabb,
                                       const m3d::vec3 &linear_velocity,
                                       m3d::scalar      dt)
    {
        const m3d::scalar margin = bp.config.fat_margin;
        const m3d::vec3   dv     = linear_velocity * dt;

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

    /** @brief Convenience: shape + transform + velocity sweep. @ingroup rbc */
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

    /**
     * @brief Insertion sort on the endpoint list — O(n) on nearly-sorted input.
     * @ingroup rbc
     * @ingroup internals
     */
    void bp_insertion_sort(std::vector<SAPEndpoint> &endpoints);

    /**
     * @brief Sweep the sorted endpoint list and fill `bp.pairs`.
     * @ingroup rbc
     * @ingroup internals
     */
    void bp_sweep(BroadPhaseState &bp);

} // namespace rbc
