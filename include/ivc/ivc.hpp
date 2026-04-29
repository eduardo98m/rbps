#pragma once

/**
 * @defgroup ivc ivc — Index Vector Core
 * @brief Stable ID + handle bookkeeping for Struct-of-Arrays collections.
 *
 * Where `storage::DEFINE_DYN_SOA` owns both the bookkeeping AND the data
 * arrays, IVC owns only the bookkeeping (index table + metadata) and lets
 * your collection own its own arrays. The trade-off: IVC is more flexible
 * (you can mix in fixed-size arrays, custom layouts, etc.) at the cost of
 * having to call the swap-and-pop yourself when erasing.
 */

/**
 * @file ivc.hpp
 * @brief Index Vector Core (IVC) — stable ID management for Struct-of-Arrays collections.
 * @ingroup ivc
 *
 * This is the SoA equivalent of siv::Vector.  Instead of owning the data, it
 * only owns the two bookkeeping tables (indexes + metadata) that make IDs
 * stable across insertions and deletions.  Your collection owns its own data
 * arrays and calls the IVC helpers to keep them in sync.
 *
 * ── Core idea ────────────────────────────────────────────────────────────────
 *
 *  indexes[id]        → data_index   (stable ID  → current position in arrays)
 *  metadata[data_idx] → {rid, vid}   (position   → {its ID, validity counter})
 *
 *  On erase(id):
 *    1. Increment metadata[data_idx].validity_id  (invalidates all Handles)
 *    2. Swap-and-pop:  move last element into the freed slot.
 *       User does the same swap on their own data arrays, then pop_back.
 *    3. Decrement n_items.
 *
 *  Free slots are reused automatically (metadata + indexes arrays grow
 *  monotonically, never shrink).
 *
 * ── Usage ────────────────────────────────────────────────────────────────────
 *
 *  struct BodyCollection {
 *      IVC_CORE;                         // embeds ivc::Core _ivc
 *      std::vector<vec3>  position;
 *      std::vector<quat>  orientation;
 *      // ...
 *  };
 *
 *  // --- Adding ---
 *  ivc::ID id = ivc::add(bc._ivc);      // reserve a stable ID
 *  bc.position.push_back(p);            // keep all arrays in sync
 *  bc.orientation.push_back(q);
 *
 *  // --- Accessing ---
 *  size_t idx = ivc::index(bc._ivc, id);
 *  bc.position[idx].x = 1.0f;
 *
 *  // --- Handles (safe references) ---
 *  ivc::Handle h = ivc::make_handle(bc._ivc, id);
 *  if (h) { auto idx = h.index(); }     // checks validity automatically
 *
 *  // --- Erasing ---
 *  ivc::erase(bc._ivc, id, [&](size_t a, size_t b) {
 *      std::swap(bc.position[a],    bc.position[b]);
 *      std::swap(bc.orientation[a], bc.orientation[b]);
 *      // ... every array in the collection
 *  });
 *  bc.position.pop_back();              // after erase, pop all arrays
 *  bc.orientation.pop_back();
 *
 * ── Fixed-size arrays ────────────────────────────────────────────────────────
 *
 *  If Capacity > 0 the bookkeeping tables are stack-allocated and no dynamic
 *  allocation occurs.  Your own data arrays must also be fixed-size then.
 *  Use IVC_CORE_FIXED(N) in the struct and the same free functions work.
 *
 *      struct BodyCollection {
 *          IVC_CORE_FIXED(256);
 *          vec3 position[256];
 *          // ...
 *      };
 */

#include <cstdint>
#include <cstddef>
#include <cassert>
#include <limits>
#include <algorithm>   // std::swap
#include <vector>

namespace ivc
{

// ─── Types ────────────────────────────────────────────────────────────────────

using ID = uint64_t;
static constexpr ID InvalidID = std::numeric_limits<ID>::max();

struct Metadata {
    ID rid         = 0; ///< reverse-ID: data_index → the ID that owns this slot
    ID validity_id = 0; ///< bumped on every erase; used to detect stale Handles
};

// ─── Core struct (two flavours) ───────────────────────────────────────────────

/**
 * @ingroup ivc
 * @brief Dynamic bookkeeping tables (heap-allocated, unlimited capacity).
 *
 * Embed via the IVC_CORE macro.
 *
 * @example test_ivc.cpp
 */
struct Core
{
    size_t              n_items  = 0; ///< number of live items
    std::vector<ID>     indexes;      ///< indexes[id]       → data_index
    std::vector<Metadata> metadata;   ///< metadata[data_idx] → {rid, validity_id}
};

/**
 * @ingroup ivc
 * @brief Fixed-size bookkeeping tables (stack-allocated, compile-time capacity).
 *
 * Embed via the IVC_CORE_FIXED(N) macro.
 * @tparam Capacity  Maximum number of live items at any one time.
 */
template<size_t Capacity>
struct CoreFixed
{
    size_t   n_items             = 0;
    size_t   n_slots             = 0; ///< total allocated slots (≥ n_items)
    ID       indexes[Capacity]   = {};
    Metadata metadata[Capacity]  = {};
};

// ─── Handle ───────────────────────────────────────────────────────────────────

/**
 * @ingroup ivc
 * @brief A safe, stable reference to an item inside a Core-bearing collection.
 *
 * Stores (id, validity_id, Core*).  Checking the handle re-reads the current
 * validity counter from the Core, so it automatically becomes invalid after
 * the item is erased.
 *
 * The handle does NOT know about your data arrays; call ivc::index(h) to get
 * the current data_index and then index into your arrays yourself.
 */
struct Handle
{
    ID    id          = InvalidID;
    ID    validity_id = 0;
    Core* core        = nullptr;

    /// True iff the item still exists in the collection.
    [[nodiscard]] bool isValid() const
    {
        return core
            && id < core->indexes.size()
            && core->metadata[core->indexes[id]].validity_id == validity_id;
    }

    explicit operator bool() const { return isValid(); }

    /// Current data_index.  Only call when isValid() == true.
    [[nodiscard]] size_t index() const
    {
        assert(isValid());
        return static_cast<size_t>(core->indexes[id]);
    }

    [[nodiscard]] ID getID() const { return id; }
};

// ─── Internal helpers (not part of the public API) ───────────────────────────

namespace detail
{
    /** Obtain a free ID, reusing freed slots when available.
     *  Grows the bookkeeping tables by one slot when needed.
     *  Does NOT touch n_items. */
    inline ID get_free_id(Core& c)
    {
        if (c.metadata.size() > c.n_items)
        {
            // A previously freed slot is available; bump its validity_id so
            // any old handles to the same ID are invalidated immediately.
            ++c.metadata[c.n_items].validity_id;
            return c.metadata[c.n_items].rid;
        }
        // No free slot — allocate a brand-new one.
        const ID new_id = static_cast<ID>(c.n_items);
        c.metadata.push_back({new_id, 0});
        c.indexes.push_back(new_id);
        return new_id;
    }

    template<size_t Capacity>
    inline ID get_free_id(CoreFixed<Capacity>& c)
    {
        assert(c.n_items < Capacity && "CoreFixed capacity exceeded");
        if (c.n_slots > c.n_items)
        {
            ++c.metadata[c.n_items].validity_id;
            return c.metadata[c.n_items].rid;
        }
        const ID new_id = static_cast<ID>(c.n_items);
        c.indexes[new_id]  = new_id;
        c.metadata[new_id] = {new_id, 0};
        ++c.n_slots;
        return new_id;
    }

    /** Core erase logic — works for both Core (dynamic) and CoreFixed (fixed). */
    template<typename TCore, typename TSwapFn>
    inline void erase_impl(TCore& c, ID id, TSwapFn&& swap_fn)
    {
        assert(c.n_items > 0 && "erase called on empty collection");
        const size_t data_idx = static_cast<size_t>(c.indexes[id]);
        const size_t last_idx = c.n_items - 1;

        // Invalidate the erased slot *before* the swap so that the
        // incremented validity_id ends up in the free-slot position.
        ++c.metadata[data_idx].validity_id;

        if (data_idx != last_idx)
        {
            const ID last_rid = c.metadata[last_idx].rid;

            // 1. User swaps their data arrays (swap-and-pop pattern).
            swap_fn(data_idx, last_idx);

            // 2. Update bookkeeping so the moved item is still reachable.
            std::swap(c.metadata[data_idx], c.metadata[last_idx]);
            std::swap(c.indexes[id],        c.indexes[last_rid]);
        }

        --c.n_items;
        // NOTE: metadata and indexes arrays keep their allocated size;
        //       the freed slot at position n_items is reused on next add().
        //       Caller must pop_back on every SoA data array.
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

// ----- reserve / capacity ----------------------------------------------------

/**
 * @ingroup ivc
 * @brief Pre-allocate space for `n` IDs in the bookkeeping tables.
 *
 * No-op for `CoreFixed` (its tables are already sized for `Capacity`).
 *
 * @param c The Core embedded in your collection.
 * @param n Expected number of items.
 */
inline void reserve(Core& c, size_t n)
{
    c.indexes.reserve(n);
    c.metadata.reserve(n);
}

template<size_t Capacity>
inline void reserve(CoreFixed<Capacity>&, size_t) { /* no-op */ }

// ----- add -------------------------------------------------------------------

/**
 * @ingroup ivc
 * @brief Register a new item and return its stable ID.
 *
 * After calling this, push_back (or assign at n_items-1) to EVERY data array
 * in your collection.  The ID is valid immediately.
 */
inline ID add(Core& c)
{
    const ID id   = detail::get_free_id(c);
    c.indexes[id] = static_cast<ID>(c.n_items);
    ++c.n_items;
    return id;
}

template<size_t Capacity>
inline ID add(CoreFixed<Capacity>& c)
{
    const ID id   = detail::get_free_id(c);
    c.indexes[id] = static_cast<ID>(c.n_items);
    ++c.n_items;
    return id;
}

// ----- erase -----------------------------------------------------------------

/**
 * @ingroup ivc
 * @brief Erase the item with the given ID.
 *
 * @param c        The Core embedded in your collection.
 * @param id       Stable ID of the item to remove.
 * @param swap_fn  Callable  void(size_t a, size_t b)  that swaps the
 *                 elements at indices a and b in ALL your SoA data arrays.
 *
 * After this call, pop_back on every data array in your collection.
 *
 * @code
 * ivc::erase(bc._ivc, id, [&](size_t a, size_t b) {
 *     std::swap(bc.position[a],    bc.position[b]);
 *     std::swap(bc.orientation[a], bc.orientation[b]);
 *     std::swap(bc.mass[a],        bc.mass[b]);
 *     // ... every array
 * });
 * bc.position.pop_back();
 * bc.orientation.pop_back();
 * bc.mass.pop_back();
 * @endcode
 */
template<typename TSwapFn>
inline void erase(Core& c, ID id, TSwapFn&& swap_fn)
{
    detail::erase_impl(c, id, std::forward<TSwapFn>(swap_fn));
}

template<size_t Capacity, typename TSwapFn>
inline void erase(CoreFixed<Capacity>& c, ID id, TSwapFn&& swap_fn)
{
    detail::erase_impl(c, id, std::forward<TSwapFn>(swap_fn));
}

/**
 * @ingroup ivc
 * @brief Convenience overload: erase via a Handle.
 */
template<typename TSwapFn>
inline void erase(Core& c, const Handle& h, TSwapFn&& swap_fn)
{
    assert(h.core == &c && h.isValid());
    erase(c, h.id, std::forward<TSwapFn>(swap_fn));
}

// ----- index access ----------------------------------------------------------

/**
 * @ingroup ivc
 * @brief Return the current packed data index for a live ID.
 *
 * No validity check — pass a stale ID and you get a stale index. Use a
 * `Handle` (or `is_valid`) when the ID may have been freed.
 */
inline size_t index(const Core& c, ID id)
{
    assert(id < c.indexes.size());
    return static_cast<size_t>(c.indexes[id]);
}

template<size_t Capacity>
inline size_t index(const CoreFixed<Capacity>& c, ID id)
{
    assert(id < c.n_slots);
    return static_cast<size_t>(c.indexes[id]);
}

/**
 * @ingroup ivc
 * @brief Return the current packed data index from a Handle (validity-checked).
 */
inline size_t index(const Handle& h)
{
    return h.index();
}

// ----- handles ---------------------------------------------------------------

/**
 * @ingroup ivc
 * @brief Create a Handle to an existing item.
 */
inline Handle make_handle(Core& c, ID id)
{
    assert(id < c.indexes.size() && c.indexes[id] < c.n_items);
    return {id, c.metadata[c.indexes[id]].validity_id, &c};
}

/**
 * @ingroup ivc
 * @brief Check validity of an `(id, validity_id)` pair stored externally.
 *
 * Use when you've serialised a snapshot of a Handle's id+validity but
 * don't want to keep the `Core*` reference around.
 */
inline bool is_valid(const Core& c, ID id, ID validity_id)
{
    return id < c.indexes.size()
        && c.metadata[c.indexes[id]].validity_id == validity_id;
}

template<size_t Capacity>
inline bool is_valid(const CoreFixed<Capacity>& c, ID id, ID validity_id)
{
    return id < c.n_slots
        && c.metadata[c.indexes[id]].validity_id == validity_id;
}

// ----- utilities -------------------------------------------------------------

/**
 * @ingroup ivc
 * @brief The ID that would be returned by the next call to `add(c)`.
 *
 * Useful when a caller needs to know the upcoming ID before committing to
 * the insertion (e.g. to wire it into a parallel structure first).
 */
inline ID next_id(const Core& c)
{
    if (c.metadata.size() > c.n_items)
        return c.metadata[c.n_items].rid;
    return static_cast<ID>(c.n_items);
}

/**
 * @ingroup ivc
 * @brief Clear all items and invalidate all existing IDs / handles.
 *
 * Bumps every validity_id so outstanding Handles become stale.
 * Caller must also clear() or resize(0) every SoA data array.
 */
inline void clear(Core& c)
{
    c.n_items = 0;
    for (auto& m : c.metadata)
        ++m.validity_id;
    // Caller: bc.position.clear(); bc.orientation.clear(); etc.
}

template<size_t Capacity>
inline void clear(CoreFixed<Capacity>& c)
{
    c.n_items = 0;
    for (size_t i = 0; i < c.n_slots; ++i)
        ++c.metadata[i].validity_id;
}

} // namespace ivc

// ─── Convenience macros ───────────────────────────────────────────────────────

/**
 * @ingroup ivc
 * @brief Embed dynamic stable-ID bookkeeping into a SoA collection struct.
 *
 * The embedded member is named `_ivc` so all `ivc::` free functions can be called
 * as  `ivc::add(myCollection._ivc)`.
 *
 * @code
 * struct BodyCollection {
 *     IVC_CORE;
 *     std::vector<vec3> position;
 *     // ...
 * };
 * @endcode
 */
#define IVC_CORE  ::ivc::Core _ivc

/**
 * @ingroup ivc
 * @brief Embed fixed-size stable-ID bookkeeping (no heap allocation).
 *
 * @param N  Maximum number of live items (compile-time constant).
 *
 * @code
 * struct BodyCollection {
 *     IVC_CORE_FIXED(256);
 *     vec3 position[256];
 *     // ...
 * };
 * @endcode
 */
#define IVC_CORE_FIXED(N)  ::ivc::CoreFixed<(N)> _ivc