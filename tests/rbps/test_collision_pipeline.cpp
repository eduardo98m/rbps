#include "tests/test_helper.hpp"
#include "rbps/CollisionPipeline.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbc/BroadPhase.hpp"

// =============================================================================
// COLLISION PIPELINE TESTS
// =============================================================================

// ── Helpers ───────────────────────────────────────────────────────────────────

static uint32_t add_dynamic_body(rbps::BodyCollection &bc, m3d::vec3 pos)
{
    rbps::BodyParams p;
    p.type     = rbps::DYNAMIC;
    p.mass     = 1.0;
    p.position = pos;
    uint32_t id = rbps::create_body(bc, p);
    return bc.index_of(id);
}

static uint32_t add_static_body(rbps::BodyCollection &bc, m3d::vec3 pos)
{
    rbps::BodyParams p;
    p.type     = rbps::STATIC;
    p.mass     = 1.0;
    p.position = pos;
    uint32_t id = rbps::create_body(bc, p);
    return bc.index_of(id);
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

// ── Tests ─────────────────────────────────────────────────────────────────────

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

TEST(pipeline_two_overlapping_spheres_one_contact)
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
    ASSERT_NEAR(contacts.restitution[0],      0.4, 1e-5);
    ASSERT_NEAR(contacts.static_friction[0],  0.4, 1e-5);
    ASSERT_NEAR(contacts.dynamic_friction[0], 0.3, 1e-5);
}

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
    rbps::run_collision_pipeline(cc, bp, bc, 0.016, contacts, cfg);
    ASSERT_EQ(contacts.n_contacts, 1u);
}

TEST_SUITE(
    RUN_TEST(pipeline_empty_scene_no_contacts),
    RUN_TEST(pipeline_two_overlapping_spheres_one_contact),
    RUN_TEST(pipeline_two_separated_spheres_no_contact),
    RUN_TEST(pipeline_static_static_overlap_no_contact),
    RUN_TEST(pipeline_same_body_two_colliders_no_self_contact),
    RUN_TEST(pipeline_contact_material_mixing),
    RUN_TEST(pipeline_solver_fields_initialised_to_zero),
    RUN_TEST(pipeline_clear_between_calls)
)