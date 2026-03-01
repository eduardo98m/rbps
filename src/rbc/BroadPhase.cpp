#include "rbc/BroadPhase.hpp"
#include <algorithm>
#include <cassert>

namespace rbc
{
    // =========================================================================
    //  Helpers
    // =========================================================================

    // Check whether `tight` is still fully contained inside `fat`.
    // Returns true when the object does NOT need a fat-AABB rebuild.
    static inline bool aabb_contained(const AABB &tight, const AABB &fat)
    {
        return tight.min.x >= fat.min.x && tight.max.x <= fat.max.x &&
               tight.min.y >= fat.min.y && tight.max.y <= fat.max.y &&
               tight.min.z >= fat.min.z && tight.max.z <= fat.max.z;
    }

    static inline AABB make_fat(const AABB &tight, m3d::scalar margin)
    {
        return aabb_expand(tight, margin);
    }

    // =========================================================================
    //  Endpoint management
    // =========================================================================

    // Append two endpoints (min + max on X) for a newly inserted object.
    static void ep_insert(std::vector<SAPEndpoint> &endpoints,
                          uint32_t                  slot,
                          const AABB               &fat)
    {
        SAPEndpoint ep_min, ep_max;
        ep_min.value  = static_cast<float>(fat.min.x);
        ep_min.slot   = slot;
        ep_min.is_max = 0;

        ep_max.value  = static_cast<float>(fat.max.x);
        ep_max.slot   = slot;
        ep_max.is_max = 1;

        endpoints.push_back(ep_min);
        endpoints.push_back(ep_max);
        // Full re-sort will be triggered via dirty flag.
    }

    // Remove both endpoints belonging to `slot` (O(n) scan — acceptable because
    // removals are rare compared to moves; a free-list keeps slots stable).
    static void ep_remove(std::vector<SAPEndpoint> &endpoints, uint32_t slot)
    {
        auto it = std::remove_if(endpoints.begin(), endpoints.end(),
                                 [slot](const SAPEndpoint &e) { return e.slot == slot; });
        endpoints.erase(it, endpoints.end());
    }

    // Update the stored values for both endpoints of `slot` in place (O(n)).
    // After updating we re-sort, so we don't need to keep the array sorted here.
    static void ep_update_values(std::vector<SAPEndpoint> &endpoints,
                                 uint32_t                  slot,
                                 const AABB               &fat)
    {
        for (SAPEndpoint &e : endpoints)
        {
            if (e.slot != slot)
                continue;
            if (e.is_max == 0)
                e.value = static_cast<float>(fat.min.x);
            else
                e.value = static_cast<float>(fat.max.x);
        }
    }

    // =========================================================================
    //  Sort
    // =========================================================================

    // Insertion sort: O(n²) worst case but O(n) when the array is nearly sorted,
    // which is exactly what happens frame-to-frame for slowly-moving objects.
    void bp_insertion_sort(std::vector<SAPEndpoint> &ep)
    {
        const int n = static_cast<int>(ep.size());
        for (int i = 1; i < n; ++i)
        {
            SAPEndpoint key = ep[i];
            int j = i - 1;
            while (j >= 0 && ep[j].value > key.value)
            {
                ep[j + 1] = ep[j];
                --j;
            }
            ep[j + 1] = key;
        }
    }

    // =========================================================================
    //  Sweep
    // =========================================================================

    // Classic SAP sweep on the X axis.
    // Active set = slots whose X-interval has started but not yet ended.
    // When we open a new interval (is_max==0), we pair it with everything active
    // and verify with a full 3-D AABB test to reject false positives.
    void bp_sweep(BroadPhaseState &bp)
    {
        bp.pairs.clear();
        bp.active_set.clear();

        for (const SAPEndpoint &ep : bp.endpoints)
        {
            const BPObject &obj = bp.objects[ep.slot];
            if (!obj.alive)
                continue;

            if (ep.is_max == 0)
            {
                // Min endpoint — open interval: test against every currently active slot.
                for (uint32_t active_slot : bp.active_set)
                {
                    const BPObject &other = bp.objects[active_slot];
                    if (!other.alive)
                        continue;

                    // Reject false positives: full 3-D fat AABB overlap test.
                    if (!aabb_overlap(obj.fat_aabb, other.fat_aabb))
                        continue;

                    // Emit the pair (lower user_id first for determinism).
                    BroadPhasePair pair;
                    if (obj.user_id < other.user_id)
                    {
                        pair.id_a = obj.user_id;
                        pair.id_b = other.user_id;
                    }
                    else
                    {
                        pair.id_a = other.user_id;
                        pair.id_b = obj.user_id;
                    }
                    bp.pairs.push_back(pair);
                }
                bp.active_set.push_back(ep.slot);
            }
            else
            {
                // Max endpoint — close interval: remove from active set.
                auto it = std::find(bp.active_set.begin(), bp.active_set.end(), ep.slot);
                if (it != bp.active_set.end())
                    bp.active_set.erase(it);
            }
        }
    }

    // =========================================================================
    //  Public API
    // =========================================================================

    void broad_phase_init(BroadPhaseState &bp, const BroadPhaseConfig &cfg)
    {
        bp.config = cfg;
        bp.objects.clear();
        bp.free_list.clear();
        bp.endpoints.clear();
        bp.active_set.clear();
        bp.pairs.clear();
        bp.dirty = false;
    }

    BPHandle broad_phase_insert(BroadPhaseState &bp,
                                uint32_t         user_id,
                                const AABB      &tight_aabb)
    {
        uint32_t slot;

        if (!bp.free_list.empty())
        {
            slot = bp.free_list.back();
            bp.free_list.pop_back();
            bp.objects[slot] = {}; // clear old data
        }
        else
        {
            slot = static_cast<uint32_t>(bp.objects.size());
            bp.objects.emplace_back();
        }

        BPObject &obj  = bp.objects[slot];
        obj.tight_aabb = tight_aabb;
        obj.fat_aabb   = make_fat(tight_aabb, bp.config.fat_margin);
        obj.user_id    = user_id;
        obj.alive      = true;

        ep_insert(bp.endpoints, slot, obj.fat_aabb);
        bp.dirty = true; // needs re-sort before next sweep

        return static_cast<BPHandle>(slot);
    }

    void broad_phase_remove(BroadPhaseState &bp, BPHandle h)
    {
        assert(h < bp.objects.size() && "Invalid BPHandle");
        BPObject &obj = bp.objects[h];
        assert(obj.alive && "Double-remove of BPHandle");

        obj.alive = false;
        ep_remove(bp.endpoints, h);
        bp.free_list.push_back(h);
        // No dirty flag needed: endpoints were already removed.
    }

    void broad_phase_move(BroadPhaseState &bp,
                          BPHandle         h,
                          const AABB      &new_tight_aabb)
    {
        assert(h < bp.objects.size() && "Invalid BPHandle");
        BPObject &obj = bp.objects[h];
        assert(obj.alive && "Move on removed BPHandle");

        obj.tight_aabb = new_tight_aabb;

        // Temporal coherence: only rebuild fat AABB and trigger a re-sort when
        // the object has escaped its current fattened envelope.
        if (aabb_contained(new_tight_aabb, obj.fat_aabb))
            return; // Still inside — no work needed this frame.

        // The tight AABB escaped: rebuild the fat AABB with extra slack.
        obj.fat_aabb = make_fat(new_tight_aabb,
                                bp.config.fat_margin + bp.config.reinsert_threshold);

        ep_update_values(bp.endpoints, h, obj.fat_aabb);
        bp.dirty = true;
    }

    void broad_phase_update(BroadPhaseState &bp)
    {
        if (bp.dirty)
        {
            // Use insertion sort — exploits near-sortedness from previous frame.
            // Falls back to std::sort for the very first frame or after many insertions.
            bp_insertion_sort(bp.endpoints);
            bp.dirty = false;
        }

        bp_sweep(bp);
    }

} // namespace rbc