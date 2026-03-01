#include "tests/test_helper.hpp"
#include "rbps/CollisionPipeline.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbc/BroadPhase.hpp"

// =============================================================================
// COLLISION PIPELINE TESTS
//
// These tests verify the full pipeline: broad phase → narrow phase → ContactList
// and the graph coloring step for parallel solving.
//
// Each test constructs a minimal BodyCollection + ColliderCollection, runs
// the pipeline, and checks the ContactList output.
// =============================================================================

// ── Helpers ───────────────────────────────────────────────────────────────────

// Creates a body at `pos` and returns its array index (== ivc slot for fresh bc)
static uint32_t add_dynamic_body(rbps::BodyCollection &bc, m3d::vec3 pos)
{
    rbps::BodyParams p;
    p.type     = rbps::DYNAMIC;
    p.mass     = 1.0;
    p.position = pos;
    ivc::ID id = rbps::create_body(bc, p);
    return static_cast<uint32_t>(ivc::index(bc._ivc, id));
}

static uint32_t add_static_body(rbps::BodyCollection &bc, m3d::vec3 pos)
{
    rbps::BodyParams p;
    p.type     = rbps::STATIC;
    p.mass     = 1.0;
    p.position = pos;
    ivc::ID id = rbps::create_body(bc, p);
    return static_cast<uint32_t>(ivc::index(bc._ivc, id));
}

static m3d::tf tf_at(float x, float y, float z)
{
    m3d::tf tf;
    tf.pos = m3d::vec3(x, y, z);
    tf.rot = m3d::quat(1, 0, 0, 0);
    return tf;
}

// Adds a sphere collider owned by `body_idx` at the body's current position.
static void add_sphere_collider(rbps::ColliderCollection &cc,
                                rbc::BroadPhaseState     &bp,
                                rbps::BodyCollection     &bc,
                                uint32_t                  body_idx,
                                float                     radius,
                                bool                      is_static = false)
{
    rbps::ColliderParams p;
    p.shape     = rbc::Sphere(radius);
    p.body_id   = body_idx;
    p.is_static = is_static;

    m3d::tf init_tf = tf_at(bc.position[body_idx].x,
                             bc.position[body_idx].y,
                             bc.position[body_idx].z);
    rbps::collider_add(cc, bp, p, init_tf);
}

// ── Empty scene ───────────────────────────────────────────────────────────────

TEST(pipeline_empty_scene_no_contacts)
{
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 0u);
}

// ── Two overlapping dynamic spheres ───────────────────────────────────────────

TEST(pipeline_two_overlapping_spheres_one_contact)
{
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    // Bodies centers 1.5 apart, radii 1.0 each → 0.5 penetration
    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(0,   0, 0));
    uint32_t b1 = add_dynamic_body(bc, m3d::vec3(1.5, 0, 0));

    add_sphere_collider(cc, bp, bc, b0, 1.0f);
    add_sphere_collider(cc, bp, bc, b1, 1.0f);

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 1u);
    ASSERT_EQ(contacts.body_a[0], b0);
    ASSERT_EQ(contacts.body_b[0], b1);
    ASSERT_NEAR(contacts.penetration_depth[0], 0.5, 0.05);
    ASSERT_NEAR(m3d::length(contacts.normal[0]), 1.0, 0.001);
}

TEST(pipeline_two_separated_spheres_no_contact)
{
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(-5, 0, 0));
    uint32_t b1 = add_dynamic_body(bc, m3d::vec3( 5, 0, 0));

    add_sphere_collider(cc, bp, bc, b0, 1.0f);
    add_sphere_collider(cc, bp, bc, b1, 1.0f);

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 0u);
}

// ── Static-static filter ──────────────────────────────────────────────────────

TEST(pipeline_static_static_overlap_no_contact)
{
    // Two static bodies whose AABBs overlap — pipeline must discard them.
    // No GJK/EPA call should be made, so no contact even though shapes overlap.
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    uint32_t b0 = add_static_body(bc, m3d::vec3(0,   0, 0));
    uint32_t b1 = add_static_body(bc, m3d::vec3(1.0, 0, 0));

    add_sphere_collider(cc, bp, bc, b0, 1.0f, /*is_static=*/true);
    add_sphere_collider(cc, bp, bc, b1, 1.0f, /*is_static=*/true);

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 0u);
}

// ── Same-body filter ──────────────────────────────────────────────────────────

TEST(pipeline_same_body_two_colliders_no_self_contact)
{
    // One body with two sphere colliders offset in opposite directions.
    // They will overlap each other's AABBs, but sharing a body_id must
    // prevent a contact being emitted.
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(0, 0, 0));

    // Both colliders belong to b0 but are offset slightly
    rbps::ColliderParams p;
    p.shape     = rbc::Sphere(1.0);
    p.body_id   = b0;
    p.local_pos = m3d::vec3(-0.3f, 0, 0);
    rbps::collider_add(cc, bp, p, tf_at(0, 0, 0));

    p.local_pos = m3d::vec3( 0.3f, 0, 0);
    rbps::collider_add(cc, bp, p, tf_at(0, 0, 0));

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 0u);
}

// ── Material mixing ───────────────────────────────────────────────────────────

TEST(pipeline_contact_material_mixing)
{
    // Verify restitution uses min, friction uses average.
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(0,   0, 0));
    uint32_t b1 = add_dynamic_body(bc, m3d::vec3(1.5, 0, 0));

    rbps::ColliderParams p0;
    p0.shape            = rbc::Sphere(1.0);
    p0.body_id          = b0;
    p0.restitution      = 0.8;
    p0.static_friction  = 0.6;
    p0.dynamic_friction = 0.4;
    rbps::collider_add(cc, bp, p0, tf_at(0, 0, 0));

    rbps::ColliderParams p1;
    p1.shape            = rbc::Sphere(1.0);
    p1.body_id          = b1;
    p1.restitution      = 0.4;
    p1.static_friction  = 0.2;
    p1.dynamic_friction = 0.2;
    rbps::collider_add(cc, bp, p1, tf_at(1.5f, 0, 0));

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 1u);
    ASSERT_NEAR(contacts.restitution[0],      0.4, 1e-5); // min(0.8, 0.4)
    ASSERT_NEAR(contacts.static_friction[0],  0.4, 1e-5); // avg(0.6, 0.2)
    ASSERT_NEAR(contacts.dynamic_friction[0], 0.3, 1e-5); // avg(0.4, 0.2)
}

// ── ContactList solver state initialisation ───────────────────────────────────

TEST(pipeline_solver_fields_initialised_to_zero)
{
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(0,   0, 0));
    uint32_t b1 = add_dynamic_body(bc, m3d::vec3(1.5, 0, 0));

    add_sphere_collider(cc, bp, bc, b0, 1.0f);
    add_sphere_collider(cc, bp, bc, b1, 1.0f);

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);

    ASSERT_EQ(contacts.n_contacts, 1u);
    ASSERT_NEAR(contacts.normal_lambda[0],  0.0, 1e-10);
    ASSERT_NEAR(contacts.tangent_lambda[0], 0.0, 1e-10);
    ASSERT_NEAR(m3d::length(contacts.normal_force[0]),  0.0, 1e-10);
    ASSERT_NEAR(m3d::length(contacts.tangent_force[0]), 0.0, 1e-10);
}

// ── Repeated pipeline calls ───────────────────────────────────────────────────

TEST(pipeline_clear_between_calls)
{
    // Calling the pipeline twice should NOT accumulate contacts — it clears.
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(0,   0, 0));
    uint32_t b1 = add_dynamic_body(bc, m3d::vec3(1.5, 0, 0));

    add_sphere_collider(cc, bp, bc, b0, 1.0f);
    add_sphere_collider(cc, bp, bc, b1, 1.0f);

    rbps::ContactList contacts;
    rbps::CollisionPipelineConfig cfg;
    cfg.use_velocity_expansion = false;

    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);
    ASSERT_EQ(contacts.n_contacts, 1u);

    // Second call — must still be exactly 1, not 2
    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);
    ASSERT_EQ(contacts.n_contacts, 1u);
}

// ── Graph coloring ────────────────────────────────────────────────────────────

TEST(graph_coloring_empty_contacts)
{
    rbps::ContactList contacts;
    auto groups = rbps::get_collision_groups(contacts);
    ASSERT_TRUE(groups.empty());
}

TEST(graph_coloring_independent_contacts_one_group)
{
    // Bodies: A-B, C-D — no shared body → all in one group (1 color needed)
    rbps::ContactList contacts;
    contacts.n_contacts = 2;
    contacts.body_a = { 0, 2 };
    contacts.body_b = { 1, 3 };

    auto groups = rbps::get_collision_groups(contacts);

    // All contacts can be solved in parallel: exactly 1 group, 2 contacts
    ASSERT_EQ(groups.size(), 1u);
    ASSERT_EQ(groups[0].size(), 2u);
}

TEST(graph_coloring_chain_needs_two_groups)
{
    // Chain: A-B, B-C  — contact 0 and 1 share body B → need 2 colors
    rbps::ContactList contacts;
    contacts.n_contacts = 2;
    contacts.body_a = { 0, 1 };  // A, B
    contacts.body_b = { 1, 2 };  // B, C  ← body 1 (B) shared

    auto groups = rbps::get_collision_groups(contacts);

    ASSERT_EQ(groups.size(), 2u);
    // Each group has exactly 1 contact
    ASSERT_EQ(groups[0].size(), 1u);
    ASSERT_EQ(groups[1].size(), 1u);
}

TEST(graph_coloring_all_contacts_share_one_body)
{
    // Star graph: A-B, A-C, A-D — all share body A → 3 colors needed
    // (no two of these contacts can be in the same group)
    rbps::ContactList contacts;
    contacts.n_contacts = 3;
    contacts.body_a = { 0, 0, 0 };   // A, A, A
    contacts.body_b = { 1, 2, 3 };   // B, C, D

    auto groups = rbps::get_collision_groups(contacts);

    ASSERT_EQ(groups.size(), 3u);
    // Each group has exactly 1 contact
    for (const auto &g : groups)
        ASSERT_EQ(g.size(), 1u);
}

TEST(graph_coloring_groups_sorted_by_size_descending)
{
    // 4 contacts: A-B, C-D, E-F (independent trio) + G-H,G-I (chain pair)
    // The independent trio (3) should come before the chain (1+1).
    rbps::ContactList contacts;
    contacts.n_contacts = 5;
    contacts.body_a = { 0, 2, 4,   6, 6  };
    contacts.body_b = { 1, 3, 5,   7, 8  };

    auto groups = rbps::get_collision_groups(contacts);

    // First group must be the largest
    ASSERT_TRUE(groups[0].size() >= groups.back().size());
}

TEST(graph_coloring_each_group_has_no_shared_body)
{
    // Verify correctness guarantee: within any group, no two contacts share a body.
    rbps::BodyCollection     bc;
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState     bp;
    rbc::broad_phase_init(bp);

    // 4 bodies in a ring: 0-1, 1-2, 2-3, 3-0
    uint32_t b0 = add_dynamic_body(bc, m3d::vec3(0,  1, 0));
    uint32_t b1 = add_dynamic_body(bc, m3d::vec3(1,  0, 0));
    uint32_t b2 = add_dynamic_body(bc, m3d::vec3(0, -1, 0));
    uint32_t b3 = add_dynamic_body(bc, m3d::vec3(-1, 0, 0));

    // Build contacts manually (no need to run the pipeline for this check)
    rbps::ContactList contacts;
    contacts.n_contacts = 4;
    contacts.body_a = { b0, b1, b2, b3 };
    contacts.body_b = { b1, b2, b3, b0 };

    auto groups = rbps::get_collision_groups(contacts);

    // Verify: within each group no two contacts share a body
    for (const auto &group : groups)
    {
        for (size_t i = 0; i < group.size(); ++i)
        {
            for (size_t j = i + 1; j < group.size(); ++j)
            {
                size_t ci = group[i];
                size_t cj = group[j];
                bool shared = (contacts.body_a[ci] == contacts.body_a[cj]) ||
                              (contacts.body_a[ci] == contacts.body_b[cj]) ||
                              (contacts.body_b[ci] == contacts.body_a[cj]) ||
                              (contacts.body_b[ci] == contacts.body_b[cj]);
                ASSERT_FALSE(shared);
            }
        }
    }
}

TEST_SUITE(
    RUN_TEST(pipeline_empty_scene_no_contacts),
    RUN_TEST(pipeline_two_overlapping_spheres_one_contact),
    RUN_TEST(pipeline_two_separated_spheres_no_contact),
    RUN_TEST(pipeline_static_static_overlap_no_contact),
    RUN_TEST(pipeline_same_body_two_colliders_no_self_contact),
    RUN_TEST(pipeline_contact_material_mixing),
    RUN_TEST(pipeline_solver_fields_initialised_to_zero),
    RUN_TEST(pipeline_clear_between_calls),
    RUN_TEST(graph_coloring_empty_contacts),
    RUN_TEST(graph_coloring_independent_contacts_one_group),
    RUN_TEST(graph_coloring_chain_needs_two_groups),
    RUN_TEST(graph_coloring_all_contacts_share_one_body),
    RUN_TEST(graph_coloring_groups_sorted_by_size_descending),
    RUN_TEST(graph_coloring_each_group_has_no_shared_body)
)