#include "tests/test_helper.hpp"
#include "rbc/BroadPhase.hpp"

// =============================================================================
// BROAD PHASE TESTS
//
// These tests verify the SAP broad phase produces correct candidate pairs.
// Each test checks:
//   1. Correct hit/no-hit detection between AABBs
//   2. Handle lifecycle (insert, move, remove, reuse)
//   3. Temporal coherence (small moves don't trigger unnecessary re-sorts)
// =============================================================================

// ── Helpers ───────────────────────────────────────────────────────────────────

static rbc::AABB make_aabb(float cx, float cy, float cz, float half)
{
    return {m3d::vec3(cx - half, cy - half, cz - half),
            m3d::vec3(cx + half, cy + half, cz + half)};
}

static bool has_pair(const std::vector<rbc::BroadPhasePair> &pairs, uint32_t a, uint32_t b)
{
    uint32_t lo = (a < b) ? a : b;
    uint32_t hi = (a < b) ? b : a;
    for (const auto &p : pairs)
        if (p.id_a == lo && p.id_b == hi)
            return true;
    return false;
}

// ── Empty / Single ────────────────────────────────────────────────────────────

TEST(empty_scene_no_pairs)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);
    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty());
}

TEST(single_object_no_pairs)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::broad_phase_insert(bp, 0, make_aabb(0, 0, 0, 1.0f));
    rbc::broad_phase_update(bp);

    ASSERT_TRUE(bp.pairs.empty());
}

// ── Overlap detection ─────────────────────────────────────────────────────────

TEST(two_overlapping_objects_one_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::broad_phase_insert(bp, 0, make_aabb(0, 0, 0, 1.0f));
    rbc::broad_phase_insert(bp, 1, make_aabb(1, 0, 0, 1.0f)); // overlaps on X

    rbc::broad_phase_update(bp);

    ASSERT_EQ(bp.pairs.size(), 1u);
    ASSERT_TRUE(has_pair(bp.pairs, 0, 1));
}

TEST(two_separated_objects_no_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp); // fat_margin = 0.1, gap must be > 0.2

    rbc::broad_phase_insert(bp, 0, make_aabb(-5, 0, 0, 1.0f));
    rbc::broad_phase_insert(bp, 1, make_aabb( 5, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);

    ASSERT_TRUE(bp.pairs.empty());
}

TEST(three_objects_only_adjacent_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::broad_phase_insert(bp, 0, make_aabb( 0, 0, 0, 1.0f));
    rbc::broad_phase_insert(bp, 1, make_aabb( 1, 0, 0, 1.0f)); // overlaps 0
    rbc::broad_phase_insert(bp, 2, make_aabb(20, 0, 0, 1.0f)); // isolated

    rbc::broad_phase_update(bp);

    ASSERT_EQ(bp.pairs.size(), 1u);
    ASSERT_TRUE(has_pair(bp.pairs, 0, 1));
}

TEST(all_three_overlapping)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    // All three overlap each other
    rbc::broad_phase_insert(bp, 0, make_aabb(0.0f, 0, 0, 2.0f));
    rbc::broad_phase_insert(bp, 1, make_aabb(1.0f, 0, 0, 2.0f));
    rbc::broad_phase_insert(bp, 2, make_aabb(0.5f, 0, 0, 2.0f));

    rbc::broad_phase_update(bp);

    ASSERT_EQ(bp.pairs.size(), 3u);
    ASSERT_TRUE(has_pair(bp.pairs, 0, 1));
    ASSERT_TRUE(has_pair(bp.pairs, 0, 2));
    ASSERT_TRUE(has_pair(bp.pairs, 1, 2));
}

// ── Handle lifecycle ──────────────────────────────────────────────────────────

TEST(remove_eliminates_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::BPHandle h0 = rbc::broad_phase_insert(bp, 0, make_aabb(0, 0, 0, 1.0f));
    rbc::BPHandle h1 = rbc::broad_phase_insert(bp, 1, make_aabb(1, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);
    ASSERT_EQ(bp.pairs.size(), 1u);

    rbc::broad_phase_remove(bp, h1);
    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty());
}

TEST(handle_slot_reused_after_remove)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::BPHandle h0 = rbc::broad_phase_insert(bp, 42, make_aabb(0, 0, 0, 1.0f));
    rbc::broad_phase_remove(bp, h0);

    rbc::BPHandle h1 = rbc::broad_phase_insert(bp, 99, make_aabb(0, 0, 0, 1.0f));
    ASSERT_EQ(h1, h0); // slot was recycled

    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty()); // only one live object
}

// ── Move / temporal coherence ─────────────────────────────────────────────────

TEST(move_into_overlap_creates_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::BPHandle h0 = rbc::broad_phase_insert(bp, 0, make_aabb(-10, 0, 0, 1.0f));
    rbc::BPHandle h1 = rbc::broad_phase_insert(bp, 1, make_aabb( 10, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty());

    rbc::broad_phase_move(bp, h0, make_aabb(9.5f, 0, 0, 1.0f));
    rbc::broad_phase_update(bp);

    ASSERT_FALSE(bp.pairs.empty());
    ASSERT_TRUE(has_pair(bp.pairs, 0, 1));
}

TEST(move_out_of_overlap_removes_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::BPHandle h0 = rbc::broad_phase_insert(bp, 0, make_aabb(0, 0, 0, 1.0f));
    rbc::BPHandle h1 = rbc::broad_phase_insert(bp, 1, make_aabb(1, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);
    ASSERT_EQ(bp.pairs.size(), 1u);

    rbc::broad_phase_move(bp, h1, make_aabb(10, 0, 0, 1.0f));
    rbc::broad_phase_update(bp);

    ASSERT_TRUE(bp.pairs.empty());
}

TEST(small_move_within_fat_aabb_does_not_set_dirty)
{
    rbc::BroadPhaseConfig cfg;
    cfg.fat_margin = 0.5f;

    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp, cfg);

    rbc::BPHandle h = rbc::broad_phase_insert(bp, 0, make_aabb(0, 0, 0, 1.0f));
    rbc::broad_phase_update(bp); // clears dirty

    // Tiny nudge — stays well inside the fat envelope
    rbc::broad_phase_move(bp, h, make_aabb(0.01f, 0, 0, 1.0f));

    ASSERT_FALSE(bp.dirty);
}

TEST(large_move_outside_fat_aabb_sets_dirty)
{
    rbc::BroadPhaseConfig cfg;
    cfg.fat_margin = 0.1f;

    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp, cfg);

    rbc::BPHandle h = rbc::broad_phase_insert(bp, 0, make_aabb(0, 0, 0, 1.0f));
    rbc::broad_phase_update(bp); // clears dirty

    // Move far enough to escape the fat envelope
    rbc::broad_phase_move(bp, h, make_aabb(5.0f, 0, 0, 1.0f));

    ASSERT_TRUE(bp.dirty);
}

// ── AABB utility ops ──────────────────────────────────────────────────────────

TEST(aabb_overlap_touching_edge)
{
    // AABBs that share exactly one face — should still count as overlapping
    rbc::AABB a = make_aabb(0, 0, 0, 1.0f); // max.x = 1
    rbc::AABB b = make_aabb(2, 0, 0, 1.0f); // min.x = 1
    ASSERT_TRUE(rbc::aabb_overlap(a, b));
}

TEST(aabb_no_overlap_on_y_axis)
{
    rbc::AABB a = make_aabb(0,  0, 0, 1.0f);
    rbc::AABB b = make_aabb(0, 10, 0, 1.0f); // separated on Y
    ASSERT_FALSE(rbc::aabb_overlap(a, b));
}

TEST(aabb_expand_increases_size)
{
    rbc::AABB a = make_aabb(0, 0, 0, 1.0f);
    rbc::AABB expanded = rbc::aabb_expand(a, 0.5);

    ASSERT_NEAR(expanded.min.x, -1.5, 1e-6);
    ASSERT_NEAR(expanded.max.x,  1.5, 1e-6);
    ASSERT_NEAR(expanded.min.y, -1.5, 1e-6);
    ASSERT_NEAR(expanded.max.y,  1.5, 1e-6);
}

// ── Velocity-expanded (swept) move ────────────────────────────────────────────

// Object A is stationary. Object B starts far to the left and moves fast enough
// to the right that its swept AABB reaches A within the same tick — even though
// its *current* tight AABB doesn't touch A yet.
TEST(swept_move_detects_fast_approaching_object)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    // A sits at x = [4, 6]  (center 5, half 1)
    rbc::BPHandle hA = rbc::broad_phase_insert(bp, 0, make_aabb(5, 0, 0, 1.0f));
    // B sits at x = [-2, 0] (center -1, half 1) — clearly separated from A
    rbc::BPHandle hB = rbc::broad_phase_insert(bp, 1, make_aabb(-1, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty()); // sanity: no overlap at rest

    // B moves right at 50 m/s, dt = 0.1 s → displacement = 5 units.
    // Swept AABB of B grows from [−2, 0] to [−2.1, 5.1] (fat_margin=0.1 on left).
    // That reaches A's fat AABB [3.9, 6.1] → should emit a pair.
    rbc::broad_phase_move_swept(bp, hB,
                                make_aabb(-1, 0, 0, 1.0f),
                                m3d::vec3(50, 0, 0),
                                0.1);
    rbc::broad_phase_update(bp);

    ASSERT_TRUE(has_pair(bp.pairs, 0, 1));
}

// Same setup but the velocity is too small to bridge the gap — no pair expected.
TEST(swept_move_slow_object_no_false_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbc::BPHandle hA = rbc::broad_phase_insert(bp, 0, make_aabb(5, 0, 0, 1.0f));
    rbc::BPHandle hB = rbc::broad_phase_insert(bp, 1, make_aabb(-1, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty());

    // B moves at 1 m/s, dt = 0.016 s → displacement = 0.016 units — nowhere near A.
    rbc::broad_phase_move_swept(bp, hB,
                                make_aabb(-1, 0, 0, 1.0f),
                                m3d::vec3(1, 0, 0),
                                0.016);
    rbc::broad_phase_update(bp);

    ASSERT_TRUE(bp.pairs.empty());
}

// Velocity pointing AWAY from the other object should not inflate the AABB
// toward it, so no pair should be emitted.
TEST(swept_move_velocity_away_no_pair)
{
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    // A at x = [4, 6], B at x = [-2, 0] — separated on X.
    rbc::BPHandle hA = rbc::broad_phase_insert(bp, 0, make_aabb( 5, 0, 0, 1.0f));
    rbc::BPHandle hB = rbc::broad_phase_insert(bp, 1, make_aabb(-1, 0, 0, 1.0f));

    rbc::broad_phase_update(bp);
    ASSERT_TRUE(bp.pairs.empty());

    // B moves LEFT (negative X) at high speed — away from A.
    // The max side of B's AABB should NOT expand toward A.
    rbc::broad_phase_move_swept(bp, hB,
                                make_aabb(-1, 0, 0, 1.0f),
                                m3d::vec3(-50, 0, 0),
                                0.1);
    rbc::broad_phase_update(bp);

    ASSERT_TRUE(bp.pairs.empty());
}

// Zero velocity should behave identically to a plain broad_phase_move()
// with only the uniform fat_margin applied.
TEST(swept_move_zero_velocity_matches_standard_move)
{
    rbc::BroadPhaseConfig cfg;
    cfg.fat_margin = 0.2f;

    rbc::BroadPhaseState bp_swept, bp_standard;
    rbc::broad_phase_init(bp_swept,   cfg);
    rbc::broad_phase_init(bp_standard, cfg);

    rbc::AABB tight = make_aabb(0, 0, 0, 1.0f);

    rbc::BPHandle hs = rbc::broad_phase_insert(bp_swept,    0, tight);
    rbc::BPHandle hm = rbc::broad_phase_insert(bp_standard, 0, tight);

    rbc::broad_phase_move_swept  (bp_swept,    hs, tight, m3d::vec3(0, 0, 0), 0.016);
    rbc::broad_phase_move        (bp_standard, hm, tight);

    // Both should produce the same fat_aabb.
    ASSERT_NEAR(bp_swept.objects[hs].fat_aabb.min.x,
                bp_standard.objects[hm].fat_aabb.min.x, 1e-5);
    ASSERT_NEAR(bp_swept.objects[hs].fat_aabb.max.x,
                bp_standard.objects[hm].fat_aabb.max.x, 1e-5);
}

TEST_SUITE(
    RUN_TEST(empty_scene_no_pairs),
    RUN_TEST(single_object_no_pairs),
    RUN_TEST(two_overlapping_objects_one_pair),
    RUN_TEST(two_separated_objects_no_pair),
    RUN_TEST(three_objects_only_adjacent_pair),
    RUN_TEST(all_three_overlapping),
    RUN_TEST(remove_eliminates_pair),
    RUN_TEST(handle_slot_reused_after_remove),
    RUN_TEST(move_into_overlap_creates_pair),
    RUN_TEST(move_out_of_overlap_removes_pair),
    RUN_TEST(small_move_within_fat_aabb_does_not_set_dirty),
    RUN_TEST(large_move_outside_fat_aabb_sets_dirty),
    RUN_TEST(aabb_overlap_touching_edge),
    RUN_TEST(aabb_no_overlap_on_y_axis),
    RUN_TEST(aabb_expand_increases_size),
    RUN_TEST(swept_move_detects_fast_approaching_object),
    RUN_TEST(swept_move_slow_object_no_false_pair),
    RUN_TEST(swept_move_velocity_away_no_pair),
    RUN_TEST(swept_move_zero_velocity_matches_standard_move)
)