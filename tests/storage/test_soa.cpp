#include "storage/Soa.hpp"
#include "tests/test_helper.hpp"
#include <algorithm>
#include <set>
#include <vector>

// Define a small local collection for fixed-capacity tests [cite: 66, 67, 68,
// 69, 70]
#define SIMPLE_FIELDS(X)                                                       \
  X(int, value)                                                                \
  X(float, weight)

DEFINE_SOA(SimpleCollection, uint32_t, 64, 8, SIMPLE_FIELDS)

// Define a dynamic collection for growth tests [cite: 11, 35]
DEFINE_DYN_SOA(DynCollection, uint32_t, 2, SIMPLE_FIELDS)

// --- 7.1 Basic add and access [cite: 86] ---

TEST(add_single_item) {
  // Create an empty collection. Call add() once. Verify count() equals 1.
  // [cite: 88]
  SimpleCollection c;
  uint32_t id = c.add();
  ASSERT_EQ(c.count(), 1);

  // Use index_of() on the returned ID and verify the data index is in range [0,
  // count()). [cite: 88]
  uint32_t idx = c.index_of(id);
  ASSERT_TRUE(idx < c.count());
}

TEST(add_writes_zero_fields) {
  // Verify that every field of the new item equals its zero-initialised value
  // before any write. [cite: 89, 90]
  SimpleCollection c;
  uint32_t id = c.add();
  uint32_t idx = c.index_of(id);

  ASSERT_EQ(c.value[idx], 0);
  ASSERT_NEAR(c.weight[idx], 0.0f, 1e-6);
}

TEST(write_and_read_fields) {
  // Write distinct values to each field. Read them back through the same data
  // index and assert they match. [cite: 91, 92]
  SimpleCollection c;
  uint32_t id = c.add();
  uint32_t idx = c.index_of(id);

  c.value[idx] = 42;
  c.weight[idx] = 3.14f;

  ASSERT_EQ(c.value[c.index_of(id)], 42);
  ASSERT_NEAR(c.weight[c.index_of(id)], 3.14f, 1e-6);
}

TEST(multiple_adds_unique_ids) {
  // Call add() three times. Assert all three returned IDs are distinct. Assert
  // count() equals 3. [cite: 93, 94]
  SimpleCollection c;
  uint32_t id1 = c.add();
  uint32_t id2 = c.add();
  uint32_t id3 = c.add();

  ASSERT_TRUE(id1 != id2);
  ASSERT_TRUE(id1 != id3);
  ASSERT_TRUE(id2 != id3);
  ASSERT_EQ(c.count(), 3);
}

// --- 7.2 contains() [cite: 95] ---

TEST(contains_live_id) {
  // After add(), contains(id) must return true. [cite: 96, 97]
  SimpleCollection c;
  uint32_t id = c.add();
  ASSERT_TRUE(c.contains(id));
}

TEST(contains_removed_id) {
  // After add() then remove(), contains(id) must return false. [cite: 98, 99]
  SimpleCollection c;
  uint32_t id = c.add();
  c.remove(id);
  ASSERT_FALSE(c.contains(id));
}

TEST(contains_never_issued_id) {
  // Pass a value that was never returned by add() (e.g. 9999). contains() must
  // return false without crashing. [cite: 100, 101]
  SimpleCollection c;
  ASSERT_FALSE(c.contains(9999));
}

// --- 7.3 remove() — data integrity [cite: 102] ---

TEST(remove_single_item) {
  // Add one item, remove it. Assert count() equals 0. Assert contains() returns
  // false. [cite: 103, 104]
  SimpleCollection c;
  uint32_t id = c.add();
  c.remove(id);
  ASSERT_EQ(c.count(), 0);
  ASSERT_FALSE(c.contains(id));
}

TEST(remove_last_item_no_swap) {
  // Add three items (A, B, C). Remove C (the last one). [cite: 105, 106]
  SimpleCollection c;
  uint32_t idA = c.add();
  c.value[c.index_of(idA)] = 10;
  uint32_t idB = c.add();
  c.value[c.index_of(idB)] = 20;
  uint32_t idC = c.add();
  c.value[c.index_of(idC)] = 30;

  uint32_t idxA_before = c.index_of(idA);
  uint32_t idxB_before = c.index_of(idB);

  c.remove(idC);

  // Assert A and B are still accessible at the same data index they had before
  // the removal, with original values. [cite: 107]
  ASSERT_EQ(c.index_of(idA), idxA_before);
  ASSERT_EQ(c.index_of(idB), idxB_before);
  ASSERT_EQ(c.value[idxA_before], 10);
  ASSERT_EQ(c.value[idxB_before], 20);
}

TEST(remove_middle_item_swap) {
  // Add three items (A, B, C) in order. Remove B. [cite: 108, 109]
  SimpleCollection c;
  uint32_t idA = c.add();
  c.value[c.index_of(idA)] = 10;
  uint32_t idB = c.add();
  c.value[c.index_of(idB)] = 20;
  uint32_t idC = c.add();
  c.value[c.index_of(idC)] = 30;

  c.remove(idB);

  // Assert count() equals 2. Assert A and C are still reachable by their IDs
  // with original values. [cite: 109]
  ASSERT_EQ(c.count(), 2);
  ASSERT_TRUE(c.contains(idA));
  ASSERT_TRUE(c.contains(idC));
  ASSERT_EQ(c.value[c.index_of(idA)], 10);
  ASSERT_EQ(c.value[c.index_of(idC)], 30);
}

TEST(remove_first_item) {
  // Add three items. Remove the one added first. Assert the other two are
  // reachable with correct values. [cite: 111, 112]
  SimpleCollection c;
  uint32_t idA = c.add();
  c.value[c.index_of(idA)] = 10;
  uint32_t idB = c.add();
  c.value[c.index_of(idB)] = 20;
  uint32_t idC = c.add();
  c.value[c.index_of(idC)] = 30;

  c.remove(idA);

  ASSERT_TRUE(c.contains(idB));
  ASSERT_TRUE(c.contains(idC));
  ASSERT_EQ(c.value[c.index_of(idB)], 20);
  ASSERT_EQ(c.value[c.index_of(idC)], 30);
}

TEST(remove_stale_id_is_noop) {
  // Remove the same ID twice. The second call must not crash and count() must
  // equal what it was. [cite: 113, 114]
  SimpleCollection c;
  uint32_t id = c.add();
  c.remove(id);
  ASSERT_EQ(c.count(), 0);

  c.remove(id);
  ASSERT_EQ(c.count(), 0);
}

// --- 7.4 ID stability [cite: 115] ---

TEST(id_stability_across_removals) {
  // Add five items, storing all five IDs. Remove items at positions 0 and 2.
  // [cite: 116, 117]
  SimpleCollection c;
  uint32_t ids[5];
  for (int i = 0; i < 5; ++i) {
    ids[i] = c.add();
    c.value[c.index_of(ids[i])] = i * 10;
  }

  c.remove(ids[0]);
  c.remove(ids[2]);

  // Assert surviving three IDs still map to original field values via
  // index_of(). [cite: 118]
  ASSERT_EQ(c.value[c.index_of(ids[1])], 10);
  ASSERT_EQ(c.value[c.index_of(ids[3])], 30);
  ASSERT_EQ(c.value[c.index_of(ids[4])], 40);
}

TEST(data_index_changes_after_swap) {
  // Add three items (A, B, C). Record data index of C before removal of B.
  // Remove B. [cite: 119, 120]
  SimpleCollection c;
  uint32_t idA = c.add();
  uint32_t idB = c.add();
  uint32_t idC = c.add();
  c.value[c.index_of(idC)] = 99;

  uint32_t idxC_before = c.index_of(idC);
  c.remove(idB);

  // Assert data index of C changed but contains() is true and value is correct.
  // [cite: 121]
  ASSERT_TRUE(c.contains(idC));
  ASSERT_TRUE(c.index_of(idC) != idxC_before);
  ASSERT_EQ(c.value[c.index_of(idC)], 99);
}

// --- 7.5 Generation counter and ID reuse [cite: 122] ---

TEST(stale_id_after_slot_reuse) {
  // Add one item, record ID, remove it. Add another item. Assert old ID is
  // false for contains(). [cite: 123, 124, 125]
  SimpleCollection c;
  uint32_t old_id = c.add();
  c.remove(old_id);

  c.add(); // Might reuse slot
  ASSERT_FALSE(c.contains(old_id));
}

TEST(new_id_after_reuse_is_valid) {
  // Assert the ID returned for the new item passes contains() and index_of() is
  // valid. [cite: 126, 127]
  SimpleCollection c;
  uint32_t old_id = c.add();
  c.remove(old_id);

  uint32_t new_id = c.add();
  ASSERT_TRUE(c.contains(new_id));
  ASSERT_TRUE(c.index_of(new_id) < c.count());
}

// --- 7.6 emplace() [cite: 128] ---

TEST(emplace_callback_receives_correct_id) {
  // Call emplace with a lambda capturing id. Assert captured equals returned
  // and contains() is true. [cite: 129, 130, 131]
  SimpleCollection c;
  uint32_t captured_id = 9999;
  uint32_t returned_id =
      c.emplace([&](uint32_t id, SimpleCollection &col) { captured_id = id; });

  ASSERT_EQ(captured_id, returned_id);
  ASSERT_TRUE(c.contains(returned_id));
}

TEST(emplace_writes_are_visible) {
  // In emplace callback, write value. Assert field contains value after return.
  // [cite: 132, 133, 134]
  SimpleCollection c;
  uint32_t returned_id = c.emplace([](uint32_t id, SimpleCollection &col) {
    col.value[col.index_of(id)] = 99;
  });

  ASSERT_EQ(c.value[c.index_of(returned_id)], 99);
}

// --- 7.7 add_n() [cite: 135] ---

TEST(add_n_correct_count) {
  // Call add_n(5, fn). Assert count() equals 5. Assert fn called exactly 5
  // times. [cite: 136, 137, 138]
  SimpleCollection c;
  int call_count = 0;
  c.add_n(5, [&](uint32_t id) { call_count++; });

  ASSERT_EQ(c.count(), 5);
  ASSERT_EQ(call_count, 5);
}

TEST(add_n_all_ids_valid) {
  // Collect all IDs passed to fn. Assert contains() returns true for everyone.
  // [cite: 139, 140]
  SimpleCollection c;
  std::vector<uint32_t> ids;
  c.add_n(5, [&](uint32_t id) { ids.push_back(id); });

  for (uint32_t id : ids) {
    ASSERT_TRUE(c.contains(id));
  }
}

// --- 7.8 for_each_index() and for_each_id() [cite: 141] ---

TEST(for_each_index_visits_all) {
  // Add 4 items. Call for_each_index, collect indices. Assert list is {0, 1, 2,
  // 3}. [cite: 142, 143, 144]
  SimpleCollection c;
  c.add_n(4, [](uint32_t) {});

  std::vector<uint32_t> indices;
  c.for_each_index([&](uint32_t idx) { indices.push_back(idx); });

  std::sort(indices.begin(), indices.end());
  ASSERT_EQ(indices.size(), 4);
  for (uint32_t i = 0; i < 4; ++i) {
    ASSERT_EQ(indices[i], i);
  }
}

TEST(for_each_id_visits_all) {
  // Add 4 items, store IDs. Call for_each_id, collect visited IDs. Assert sets
  // equal. [cite: 145, 146]
  SimpleCollection c;
  std::set<uint32_t> stored_ids;
  c.add_n(4, [&](uint32_t id) { stored_ids.insert(id); });

  std::set<uint32_t> visited_ids;
  c.for_each_id([&](uint32_t id) { visited_ids.insert(id); });

  ASSERT_TRUE(stored_ids == visited_ids);
}

TEST(for_each_after_removal) {
  // Add 4 items, remove one. Call for_each_index. Assert it visits exactly 3
  // indices. [cite: 147, 148]
  SimpleCollection c;
  uint32_t id = c.add();
  c.add_n(3, [](uint32_t) {});

  c.remove(id);

  int count = 0;
  c.for_each_index([&](uint32_t idx) { count++; });

  ASSERT_EQ(count, 3);
}

// --- 7.9 clear() [cite: 149] ---

TEST(clear_resets_count) {
  // Add several items, call clear(), assert count() equals 0. [cite: 150, 151]
  SimpleCollection c;
  c.add_n(5, [](uint32_t) {});
  c.clear();
  ASSERT_EQ(c.count(), 0);
}

TEST(clear_invalidates_ids) {
  // Add two items, store IDs, call clear(). Assert contains() false for both.
  // [cite: 152, 153]
  SimpleCollection c;
  uint32_t id1 = c.add();
  uint32_t id2 = c.add();

  c.clear();

  ASSERT_FALSE(c.contains(id1));
  ASSERT_FALSE(c.contains(id2));
}

TEST(add_after_clear) {
  // Clear a collection that had items, call add(). Assert count() is 1, new
  // item accessible. [cite: 154, 155, 156]
  SimpleCollection c;
  c.add_n(3, [](uint32_t) {});
  c.clear();

  uint32_t new_id = c.add();
  ASSERT_EQ(c.count(), 1);
  ASSERT_TRUE(c.contains(new_id));
}

// --- 7.10 Stress test [cite: 157] ---

TEST(interleaved_add_remove) {
  // Add 10 items. Remove every other one. Assert count()=5. Assert 5 surviving
  // IDs map correctly. [cite: 158, 159]
  SimpleCollection c;
  std::vector<uint32_t> ids;
  for (int i = 0; i < 10; ++i) {
    uint32_t id = c.add();
    c.value[c.index_of(id)] = i * 100;
    ids.push_back(id);
  }

  for (int i = 0; i < 10; i += 2) {
    c.remove(ids[i]);
  }

  ASSERT_EQ(c.count(), 5);
  for (int i = 1; i < 10; i += 2) {
    ASSERT_TRUE(c.contains(ids[i]));
    ASSERT_EQ(c.value[c.index_of(ids[i])], i * 100);
  }

  // Add 5 more items. Assert count() equals 10. [cite: 160]
  c.add_n(5, [](uint32_t) {});
  ASSERT_EQ(c.count(), 10);
}

// --- 8. Dynamic Variant Additional Tests [cite: 161, 162] ---

TEST(grows_beyond_initial_capacity) {
  // Create DynSoA (initial capacity 2). Add 10 items. Assert count()=10 and all
  // valid. [cite: 164, 165, 166]
  DynCollection c;
  std::vector<uint32_t> ids;
  for (int i = 0; i < 10; ++i) {
    ids.push_back(c.add());
  }

  ASSERT_EQ(c.count(), 10);
  for (uint32_t id : ids) {
    ASSERT_TRUE(c.contains(id));
  }
}

TEST(reserve_does_not_change_count) {
  // On empty dynamic collection, call reserve(100). Assert count()=0. Add one
  // item, assert count()=1. [cite: 168, 169]
  DynCollection c;
  c.reserve(100);
  ASSERT_EQ(c.count(), 0);

  c.add();
  ASSERT_EQ(c.count(), 1);
}

// --- Test Suite Execution ---

TEST_SUITE(
    // 7.1
    RUN_TEST(add_single_item), RUN_TEST(add_writes_zero_fields),
    RUN_TEST(write_and_read_fields), RUN_TEST(multiple_adds_unique_ids),
    // 7.2
    RUN_TEST(contains_live_id), RUN_TEST(contains_removed_id),
    RUN_TEST(contains_never_issued_id),
    // 7.3
    RUN_TEST(remove_single_item), RUN_TEST(remove_last_item_no_swap),
    RUN_TEST(remove_middle_item_swap), RUN_TEST(remove_first_item),
    RUN_TEST(remove_stale_id_is_noop),
    // 7.4
    RUN_TEST(id_stability_across_removals),
    RUN_TEST(data_index_changes_after_swap),
    // 7.5
    RUN_TEST(stale_id_after_slot_reuse), RUN_TEST(new_id_after_reuse_is_valid),
    // 7.6
    RUN_TEST(emplace_callback_receives_correct_id),
    RUN_TEST(emplace_writes_are_visible),
    // 7.7
    RUN_TEST(add_n_correct_count), RUN_TEST(add_n_all_ids_valid),
    // 7.8
    RUN_TEST(for_each_index_visits_all), RUN_TEST(for_each_id_visits_all),
    RUN_TEST(for_each_after_removal),
    // 7.9
    RUN_TEST(clear_resets_count), RUN_TEST(clear_invalidates_ids),
    RUN_TEST(add_after_clear),
    // 7.10
    RUN_TEST(interleaved_add_remove),
    // 8.
    RUN_TEST(grows_beyond_initial_capacity),
    RUN_TEST(reserve_does_not_change_count))
