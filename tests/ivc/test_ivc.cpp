#include "ivc/ivc.hpp"
#include "tests/test_helper.hpp"

// ─── Minimal SoA collection used across all tests ─────────────────────────────
//
// Deliberately small — just enough fields to verify the swap-and-pop
// bookkeeping across multiple arrays.

struct SimpleCollection
{
    IVC_CORE;
    std::vector<int>   value;
    std::vector<float> weight;

    // Convenience: add one item and return its stable ID.
    ivc::ID add(int v, float w)
    {
        ivc::ID id = ivc::add(_ivc);
        value.push_back(v);
        weight.push_back(w);
        return id;
    }

    // Convenience: remove one item by stable ID.
    void remove(ivc::ID id)
    {
        ivc::erase(_ivc, id, [&](size_t a, size_t b) {
            std::swap(value[a],  value[b]);
            std::swap(weight[a], weight[b]);
        });
        value.pop_back();
        weight.pop_back();
    }
};

// ─── Tests ────────────────────────────────────────────────────────────────────

// Basic: single add, then index access
TEST(add_and_index)
{
    SimpleCollection c;
    ivc::ID id = c.add(42, 1.5f);

    ASSERT_EQ(c._ivc.n_items, 1u);
    size_t idx = ivc::index(c._ivc, id);
    ASSERT_EQ(c.value[idx],  42);
    ASSERT_NEAR(c.weight[idx], 1.5f);
}

// Multiple adds produce sequential indices and unique IDs
TEST(multiple_adds)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(10, 0.1f);
    ivc::ID id1 = c.add(20, 0.2f);
    ivc::ID id2 = c.add(30, 0.3f);

    ASSERT_EQ(c._ivc.n_items, 3u);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id0)], 10);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id1)], 20);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id2)], 30);
}

// Erase the only element
TEST(erase_single)
{
    SimpleCollection c;
    ivc::ID id = c.add(99, 9.9f);
    c.remove(id);

    ASSERT_EQ(c._ivc.n_items, 0u);
    ASSERT_EQ(c.value.size(),  0u);
}

// Erase the last element in a multi-element collection (no swap needed)
TEST(erase_last_element)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(1, 1.0f);
    ivc::ID id1 = c.add(2, 2.0f);
    ivc::ID id2 = c.add(3, 3.0f);

    c.remove(id2);   // erasing the last → no swap should occur

    ASSERT_EQ(c._ivc.n_items, 2u);
    // id0 and id1 must still point to correct data
    ASSERT_EQ(c.value[ivc::index(c._ivc, id0)], 1);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id1)], 2);
}

// Erase a middle element: the last item slides into the freed slot
TEST(erase_middle_element_swap)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(10, 1.0f);
    ivc::ID id1 = c.add(20, 2.0f); // this will be erased
    ivc::ID id2 = c.add(30, 3.0f); // this should slide into slot 1

    c.remove(id1);

    ASSERT_EQ(c._ivc.n_items, 2u);
    // id0 must be unchanged
    ASSERT_EQ(c.value[ivc::index(c._ivc, id0)], 10);
    // id2 must still be reachable with its original value
    ASSERT_EQ(c.value[ivc::index(c._ivc, id2)], 30);
    // data arrays must be exactly size 2
    ASSERT_EQ(c.value.size(),  2u);
    ASSERT_EQ(c.weight.size(), 2u);
}

// Erase the first element
TEST(erase_first_element)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(100, 1.0f); // erased
    ivc::ID id1 = c.add(200, 2.0f);
    ivc::ID id2 = c.add(300, 3.0f);

    c.remove(id0);

    ASSERT_EQ(c._ivc.n_items, 2u);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id1)], 200);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id2)], 300);
}

// ID stability: IDs of surviving items must not change across deletions
TEST(id_stability_across_deletions)
{
    SimpleCollection c;
    ivc::ID ids[5];
    for (int i = 0; i < 5; ++i)
        ids[i] = c.add(i * 10, float(i));

    // Remove items out of order
    c.remove(ids[2]);
    c.remove(ids[0]);

    // Remaining: ids[1]=10, ids[3]=30, ids[4]=40
    ASSERT_EQ(c.value[ivc::index(c._ivc, ids[1])], 10);
    ASSERT_EQ(c.value[ivc::index(c._ivc, ids[3])], 30);
    ASSERT_EQ(c.value[ivc::index(c._ivc, ids[4])], 40);
}

// ID reuse: after erase, next add reuses the freed slot
TEST(id_reuse_after_erase)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(1, 0.0f);
    ivc::ID id1 = c.add(2, 0.0f);
    c.remove(id0);

    ivc::ID id2 = c.add(99, 0.0f); // should reuse freed slot

    // The reused ID is different from id1 but may equal id0
    ASSERT_EQ(c._ivc.n_items, 2u);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id2)], 99);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id1)], 2);
}

// ─── Handle tests ─────────────────────────────────────────────────────────────

// A fresh handle must be valid
TEST(handle_valid_after_creation)
{
    SimpleCollection c;
    ivc::ID id = c.add(7, 0.7f);
    ivc::Handle h = ivc::make_handle(c._ivc, id);

    ASSERT_TRUE(h.isValid());
    ASSERT_TRUE(static_cast<bool>(h));
    ASSERT_EQ(h.getID(), id);
}

// After erase, the handle must become invalid
TEST(handle_invalid_after_erase)
{
    SimpleCollection c;
    ivc::ID id = c.add(7, 0.7f);
    ivc::Handle h = ivc::make_handle(c._ivc, id);

    c.remove(id);

    ASSERT_FALSE(h.isValid());
    ASSERT_FALSE(static_cast<bool>(h));
}

// Handles to other items remain valid when an unrelated item is erased
TEST(handle_unaffected_by_other_erase)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(1, 0.0f);
    ivc::ID id1 = c.add(2, 0.0f);
    ivc::Handle h0 = ivc::make_handle(c._ivc, id0);
    ivc::Handle h1 = ivc::make_handle(c._ivc, id1);

    c.remove(id1);  // erase id1, not id0

    ASSERT_TRUE(h0.isValid());
    ASSERT_FALSE(h1.isValid());
}

// Handle index() follows the item even if it moves due to swap-and-pop
TEST(handle_index_follows_moved_item)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(10, 0.0f);
    ivc::ID id1 = c.add(20, 0.0f); // erased — causes id2 to move
    ivc::ID id2 = c.add(30, 0.0f);
    ivc::Handle h2 = ivc::make_handle(c._ivc, id2);

    c.remove(id1); // id2 slides from index 2 into index 1

    ASSERT_TRUE(h2.isValid());
    // Data must be the original value regardless of new position
    ASSERT_EQ(c.value[h2.index()], 30);
}

// ─── clear() ──────────────────────────────────────────────────────────────────

TEST(clear_invalidates_all_handles)
{
    SimpleCollection c;
    ivc::ID id0 = c.add(1, 0.0f);
    ivc::ID id1 = c.add(2, 0.0f);
    ivc::Handle h0 = ivc::make_handle(c._ivc, id0);
    ivc::Handle h1 = ivc::make_handle(c._ivc, id1);

    ivc::clear(c._ivc);
    c.value.clear();
    c.weight.clear();

    ASSERT_EQ(c._ivc.n_items, 0u);
    ASSERT_FALSE(h0.isValid());
    ASSERT_FALSE(h1.isValid());
}

TEST(add_after_clear_works)
{
    SimpleCollection c;
    c.add(1, 0.0f);
    ivc::clear(c._ivc);
    c.value.clear();
    c.weight.clear();

    ivc::ID fresh = c.add(42, 4.2f);
    ASSERT_EQ(c._ivc.n_items, 1u);
    ASSERT_EQ(c.value[ivc::index(c._ivc, fresh)], 42);
}

// ─── Fixed-size variant ───────────────────────────────────────────────────────

struct FixedCollection
{
    static constexpr size_t CAP = 8;
    IVC_CORE_FIXED(CAP);
    int   value[CAP]  = {};
    float weight[CAP] = {};

    ivc::ID add(int v, float w)
    {
        ivc::ID id  = ivc::add(_ivc);
        size_t  idx = _ivc.n_items - 1;
        value[idx]  = v;
        weight[idx] = w;
        return id;
    }

    void remove(ivc::ID id)
    {
        ivc::erase(_ivc, id, [&](size_t a, size_t b) {
            std::swap(value[a],  value[b]);
            std::swap(weight[a], weight[b]);
        });
        // No pop_back for plain arrays; n_items is already decremented.
    }
};

TEST(fixed_core_add_and_access)
{
    FixedCollection c;
    ivc::ID id = c.add(55, 5.5f);

    ASSERT_EQ(c._ivc.n_items, 1u);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id)], 55);
}

TEST(fixed_core_erase_and_stability)
{
    FixedCollection c;
    ivc::ID id0 = c.add(1, 0.1f);
    ivc::ID id1 = c.add(2, 0.2f);
    ivc::ID id2 = c.add(3, 0.3f);

    c.remove(id1);

    ASSERT_EQ(c._ivc.n_items, 2u);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id0)], 1);
    ASSERT_EQ(c.value[ivc::index(c._ivc, id2)], 3);
}

TEST(fixed_core_handle_validity)
{
    FixedCollection c;
    ivc::ID id = c.add(77, 7.7f);
    // Fixed core does not produce ivc::Handle directly (no Core* stored),
    // but we can test is_valid() with a manually captured validity_id.
    ivc::ID vid = c._ivc.metadata[c._ivc.indexes[id]].validity_id;

    ASSERT_TRUE(ivc::is_valid(c._ivc, id, vid));   // must be valid now

    // NOTE: is_valid is only templated for Core, not CoreFixed,
    // so this tests the overload for CoreFixed via index bounds check.
    c.remove(id);

    // After removal the validity_id is bumped; old vid no longer matches.
    ASSERT_FALSE(ivc::is_valid(c._ivc, id, vid));
}

// ─── n_items tracking ─────────────────────────────────────────────────────────

TEST(n_items_tracks_correctly)
{
    SimpleCollection c;
    ASSERT_EQ(c._ivc.n_items, 0u);

    ivc::ID a = c.add(1, 0.0f);
    ASSERT_EQ(c._ivc.n_items, 1u);

    ivc::ID b = c.add(2, 0.0f);
    ivc::ID d = c.add(4, 0.0f);
    ASSERT_EQ(c._ivc.n_items, 3u);

    c.remove(b);
    ASSERT_EQ(c._ivc.n_items, 2u);

    c.remove(a);
    c.remove(d);
    ASSERT_EQ(c._ivc.n_items, 0u);
}

// ─── Stress: interleaved add / erase ─────────────────────────────────────────

TEST(stress_interleaved_add_erase)
{
    SimpleCollection c;
    std::vector<ivc::ID> live_ids;

    // Add 10 items
    for (int i = 0; i < 10; ++i)
        live_ids.push_back(c.add(i, float(i)));

    // Remove every other one
    for (size_t i = 0; i < live_ids.size(); i += 2)
        c.remove(live_ids[i]);

    // The 5 surviving items must still have correct values
    for (size_t i = 1; i < live_ids.size(); i += 2)
    {
        size_t idx = ivc::index(c._ivc, live_ids[i]);
        ASSERT_EQ(c.value[idx], static_cast<int>(i));
    }

    ASSERT_EQ(c._ivc.n_items, 5u);
    ASSERT_EQ(c.value.size(),  5u);

    // Add 5 more — must reuse freed slots without corrupting survivors
    for (int i = 100; i < 105; ++i)
        c.add(i, float(i));

    ASSERT_EQ(c._ivc.n_items, 10u);
}

// ─── Suite ────────────────────────────────────────────────────────────────────

TEST_SUITE(
    RUN_TEST(add_and_index),
    RUN_TEST(multiple_adds),
    RUN_TEST(erase_single),
    RUN_TEST(erase_last_element),
    RUN_TEST(erase_middle_element_swap),
    RUN_TEST(erase_first_element),
    RUN_TEST(id_stability_across_deletions),
    RUN_TEST(id_reuse_after_erase),
    RUN_TEST(handle_valid_after_creation),
    RUN_TEST(handle_invalid_after_erase),
    RUN_TEST(handle_unaffected_by_other_erase),
    RUN_TEST(handle_index_follows_moved_item),
    RUN_TEST(clear_invalidates_all_handles),
    RUN_TEST(add_after_clear_works),
    RUN_TEST(fixed_core_add_and_access),
    RUN_TEST(fixed_core_erase_and_stability),
    RUN_TEST(fixed_core_handle_validity),
    RUN_TEST(n_items_tracks_correctly),
    RUN_TEST(stress_interleaved_add_erase)
)