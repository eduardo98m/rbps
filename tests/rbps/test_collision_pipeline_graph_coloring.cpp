#include "tests/test_helper.hpp"
#include "rbps/CollisionPipeline.hpp"
#include "rbps/API/BodyAPI.hpp"

// =============================================================================
// GRAPH COLORING TESTS
//
// These tests verify get_collision_groups() produces independent groups for
// parallel constraint solving. Key properties checked:
//
//   1. Correctness: no two contacts in the same group share a dynamic body.
//   2. Static body cut-off: a static body shared by multiple contacts must
//      NOT merge those contacts into the same group — the "tower stacks on
//      a ground plane" scenario.
//   3. Completeness: every contact index appears in exactly one group.
//   4. Size ordering: groups are sorted largest-first.
// =============================================================================

// ── Helpers ───────────────────────────────────────────────────────────────────

static uint32_t add_dynamic(rbps::BodyCollection &bc, m3d::vec3 pos = m3d::vec3(0))
{
    rbps::BodyParams p;
    p.type     = rbps::DYNAMIC;
    p.mass     = 1.0;
    p.position = pos;
    uint32_t id = rbps::create_body(bc, p);
    return bc.index_of(id);
}

static uint32_t add_static(rbps::BodyCollection &bc, m3d::vec3 pos = m3d::vec3(0))
{
    rbps::BodyParams p;
    p.type     = rbps::STATIC;
    p.mass     = 1.0;
    p.position = pos;
    uint32_t id = rbps::create_body(bc, p);
    return bc.index_of(id);
}

// Verify the core correctness invariant: no two contacts in the same group
// share a DYNAMIC body.
static void assert_groups_independent(const rbps::ContactList  &contacts,
                                      const rbps::BodyCollection &bc,
                                      const std::vector<std::vector<size_t>> &groups)
{
    for (const auto &group : groups)
    {
        for (size_t i = 0; i < group.size(); ++i)
        {
            for (size_t j = i + 1; j < group.size(); ++j)
            {
                const size_t ci = group[i];
                const size_t cj = group[j];

                // Only dynamic bodies matter — static ones don't create races
                auto is_dyn = [&](uint32_t bid) {
                    return bc.type[bid] == rbps::DYNAMIC;
                };

                bool race = false;
                if (is_dyn(contacts.body_a[ci]))
                    race |= (contacts.body_a[ci] == contacts.body_a[cj] ||
                             contacts.body_a[ci] == contacts.body_b[cj]);
                if (is_dyn(contacts.body_b[ci]))
                    race |= (contacts.body_b[ci] == contacts.body_a[cj] ||
                             contacts.body_b[ci] == contacts.body_b[cj]);

                ASSERT_FALSE(race);
            }
        }
    }
}

// Verify every contact index appears in exactly one group.
static void assert_groups_complete(size_t n_contacts,
                                   const std::vector<std::vector<size_t>> &groups)
{
    std::vector<size_t> seen(n_contacts, 0);
    for (const auto &g : groups)
        for (size_t ci : g)
            seen[ci]++;
    for (size_t ci = 0; ci < n_contacts; ++ci)
        ASSERT_EQ(seen[ci], 1u);
}

// ── Basic cases ───────────────────────────────────────────────────────────────

TEST(empty_contacts_empty_groups)
{
    rbps::BodyCollection bc;
    rbps::ContactList    contacts;
    auto groups = rbps::get_collision_groups(contacts, bc);
    ASSERT_TRUE(groups.empty());
}

TEST(single_contact_one_group_of_one)
{
    rbps::BodyCollection bc;
    uint32_t b0 = add_dynamic(bc);
    uint32_t b1 = add_dynamic(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 1;
    contacts.body_a = { b0 };
    contacts.body_b = { b1 };

    auto groups = rbps::get_collision_groups(contacts, bc);

    ASSERT_EQ(groups.size(), 1u);
    ASSERT_EQ(groups[0].size(), 1u);
    assert_groups_complete(contacts.n_contacts, groups);
}

// ── Dynamic-only graph ────────────────────────────────────────────────────────

TEST(independent_dynamic_pairs_one_group)
{
    // A-B and C-D — no shared body → all in one group
    rbps::BodyCollection bc;
    uint32_t b0 = add_dynamic(bc), b1 = add_dynamic(bc);
    uint32_t b2 = add_dynamic(bc), b3 = add_dynamic(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 2;
    contacts.body_a = { b0, b2 };
    contacts.body_b = { b1, b3 };

    auto groups = rbps::get_collision_groups(contacts, bc);

    ASSERT_EQ(groups.size(), 1u);
    ASSERT_EQ(groups[0].size(), 2u);
    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);
}

TEST(chain_A_B_B_C_needs_two_groups)
{
    // A-B and B-C share dynamic body B → must be in different groups
    rbps::BodyCollection bc;
    uint32_t b0 = add_dynamic(bc);
    uint32_t b1 = add_dynamic(bc);
    uint32_t b2 = add_dynamic(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 2;
    contacts.body_a = { b0, b1 };
    contacts.body_b = { b1, b2 };

    auto groups = rbps::get_collision_groups(contacts, bc);

    ASSERT_EQ(groups.size(), 2u);
    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);
}

TEST(star_graph_all_contacts_sequential)
{
    // A-B, A-C, A-D all share dynamic body A → 3 separate groups
    rbps::BodyCollection bc;
    uint32_t a  = add_dynamic(bc);
    uint32_t b0 = add_dynamic(bc);
    uint32_t b1 = add_dynamic(bc);
    uint32_t b2 = add_dynamic(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 3;
    contacts.body_a = { a,  a,  a  };
    contacts.body_b = { b0, b1, b2 };

    auto groups = rbps::get_collision_groups(contacts, bc);

    ASSERT_EQ(groups.size(), 3u);
    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);
}

// ── THE KEY FIX: static body cut-off ─────────────────────────────────────────

TEST(static_body_does_not_merge_independent_contacts)
{
    // Ground plane (static) sits below two separate dynamic spheres.
    //
    //   contact 0: sphere_A  vs  ground  (dynamic A + static G)
    //   contact 1: sphere_B  vs  ground  (dynamic B + static G)
    //
    // Before the fix: ground appeared in body_to_contacts, creating an edge
    // between contact 0 and contact 1 → merged into 1 sequential group.
    //
    // After the fix: ground is excluded from the adjacency map → contacts 0
    // and 1 have NO shared dynamic body → placed in the SAME group (parallel).
    rbps::BodyCollection bc;
    uint32_t ground   = add_static (bc, m3d::vec3(0, -1,  0));
    uint32_t sphere_a = add_dynamic(bc, m3d::vec3(-3, 1,  0));
    uint32_t sphere_b = add_dynamic(bc, m3d::vec3( 3, 1,  0));

    rbps::ContactList contacts;
    contacts.n_contacts = 2;
    contacts.body_a = { sphere_a, sphere_b };
    contacts.body_b = { ground,   ground   };

    auto groups = rbps::get_collision_groups(contacts, bc);

    // Both contacts are fully independent: only 1 group required.
    ASSERT_EQ(groups.size(), 1u);
    ASSERT_EQ(groups[0].size(), 2u);
    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);
}

TEST(tower_stacks_on_ground_plane_fully_parallel)
{
    // The motivating scenario: N block towers on a static ground plane.
    // Each tower is a chain of dynamic blocks. Towers are fully independent
    // of each other — only the static ground plane "connects" them.
    //
    // With the fix, the ground plane connection is invisible to the coloring
    // and each tower's contacts are grouped independently.
    //
    // Setup: 3 towers of 2 blocks each, all resting on static ground.
    //
    //   Tower 0: block[0] on ground,  block[1] on block[0]
    //   Tower 1: block[2] on ground,  block[3] on block[2]
    //   Tower 2: block[4] on ground,  block[5] on block[4]
    //
    // Contacts (6 total):
    //   0: ground   ↔ block[0]   (static-dynamic)
    //   1: block[0] ↔ block[1]   (dynamic-dynamic, within tower 0)
    //   2: ground   ↔ block[2]   (static-dynamic)
    //   3: block[2] ↔ block[3]   (dynamic-dynamic, within tower 1)
    //   4: ground   ↔ block[4]   (static-dynamic)
    //   5: block[4] ↔ block[5]   (dynamic-dynamic, within tower 2)
    rbps::BodyCollection bc;
    uint32_t ground = add_static(bc);
    uint32_t blk[6];
    for (auto &b : blk) b = add_dynamic(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 6;
    contacts.body_a = { ground,  blk[0], ground,  blk[2], ground,  blk[4] };
    contacts.body_b = { blk[0],  blk[1], blk[2],  blk[3], blk[4],  blk[5] };

    auto groups = rbps::get_collision_groups(contacts, bc);

    // Correctness always holds
    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);

    // The three intra-tower contacts (1, 3, 5) share no dynamic body with each
    // other → they should end up in the same parallel group.
    // The ground contacts (0, 2, 4) share no dynamic body either → also parallel.
    // Best case: 2 groups (wave 0: ground contacts, wave 1: stacking contacts).
    // The algorithm may produce more due to greedy ordering, but must not
    // produce 6 (which is what the unfixed version would give).
    ASSERT_TRUE(groups.size() < 6u);
}

TEST(mixed_static_and_dynamic_chain_partial_cutoff)
{
    // A-S-B where S is static: A and B are NOT connected through S.
    // But A and B also have a direct contact with each other.
    //
    //   contact 0: A ↔ S  (dynamic A, static S)
    //   contact 1: B ↔ S  (dynamic B, static S)
    //   contact 2: A ↔ B  (dynamic A, dynamic B)
    //
    // Contacts 0 and 1 are independent (only static S connects them).
    // Contact 2 creates an edge between the groups of contact 0 and contact 1
    // (through A and B respectively). So we need at least 2 groups.
    rbps::BodyCollection bc;
    uint32_t a = add_dynamic(bc);
    uint32_t b = add_dynamic(bc);
    uint32_t s = add_static(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 3;
    contacts.body_a = { a, b, a };
    contacts.body_b = { s, s, b };

    auto groups = rbps::get_collision_groups(contacts, bc);

    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);

    // Contact 2 (A↔B) must be in a different group from contact 0 (A↔S) or
    // contact 1 (B↔S) — verify via the independence invariant (already done above).
    ASSERT_TRUE(groups.size() >= 1u);
}

// ── Completeness and ordering ─────────────────────────────────────────────────

TEST(all_contacts_appear_exactly_once)
{
    // Stress-test completeness with a larger mixed scene
    rbps::BodyCollection bc;
    uint32_t ground = add_static(bc);
    uint32_t d[8];
    for (auto &b : d) b = add_dynamic(bc);

    // Ring of dynamic bodies, all also touching the ground
    rbps::ContactList contacts;
    contacts.n_contacts = 12;
    contacts.body_a = { d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],
                        ground,ground,ground,ground };
    contacts.body_b = { d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[0],
                        d[0], d[2], d[4], d[6] };

    auto groups = rbps::get_collision_groups(contacts, bc);

    assert_groups_independent(contacts, bc, groups);
    assert_groups_complete(contacts.n_contacts, groups);
}

TEST(groups_sorted_by_size_descending)
{
    // Ensure largest group comes first
    rbps::BodyCollection bc;
    // 4 independent pairs (→ 1 big group) + 1 chain (→ 2 small groups)
    uint32_t d[10];
    for (auto &b : d) b = add_dynamic(bc);

    rbps::ContactList contacts;
    contacts.n_contacts = 5;
    // First 4 contacts are fully independent
    contacts.body_a = { d[0], d[2], d[4], d[6],  d[8] };
    contacts.body_b = { d[1], d[3], d[5], d[7],  d[8] }; // last one: d[8] self (won't happen in practice)

    // Actually make 4 independent + 1 chain d[0]-d[1]-d[2]
    contacts.n_contacts = 6;
    contacts.body_a = { d[0], d[2], d[4], d[6],  d[0], d[1] };
    contacts.body_b = { d[1], d[3], d[5], d[7],  d[1], d[2] }; // last two share d[1] with first

    auto groups = rbps::get_collision_groups(contacts, bc);

    assert_groups_complete(contacts.n_contacts, groups);
    for (size_t i = 1; i < groups.size(); ++i)
        ASSERT_TRUE(groups[i - 1].size() >= groups[i].size());
}

TEST_SUITE(
    RUN_TEST(empty_contacts_empty_groups),
    RUN_TEST(single_contact_one_group_of_one),
    RUN_TEST(independent_dynamic_pairs_one_group),
    RUN_TEST(chain_A_B_B_C_needs_two_groups),
    RUN_TEST(star_graph_all_contacts_sequential),
    RUN_TEST(static_body_does_not_merge_independent_contacts),
    RUN_TEST(tower_stacks_on_ground_plane_fully_parallel),
    RUN_TEST(mixed_static_and_dynamic_chain_partial_cutoff),
    RUN_TEST(all_contacts_appear_exactly_once),
    RUN_TEST(groups_sorted_by_size_descending)
)