#include "tests/test_helper.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbc/BroadPhase.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

// =============================================================================
// COLLIDER API TESTS
//
// These tests verify the ColliderAPI: add, remove, handle lifecycle,
// world transform computation, and broad phase synchronisation.
//
// NOTE: create_collider() currently has an identical ternary for is_static —
//       both branches call broad_phase_insert with the same arguments.
//       A TODO comment marks the spot; test_static_flag_stored verifies
//       the field is at least stored correctly in the meantime.
// =============================================================================

// ── Helpers ───────────────────────────────────────────────────────────────────

static rbps::BodyCollection make_body_collection_with_one_body(
    m3d::vec3 pos = m3d::vec3(0, 0, 0))
{
    rbps::BodyCollection bc;
    rbps::BodyParams p;
    p.type = rbps::DYNAMIC;
    p.mass = 1.0;
    p.position = pos;
    rbps::create_body(bc, p);
    return bc;
}

static m3d::tf identity_tf()
{
    m3d::tf tf;
    tf.pos = m3d::vec3(0, 0, 0);
    tf.rot = m3d::quat(1, 0, 0, 0);
    return tf;
}

static m3d::tf tf_at(float x, float y, float z)
{
    m3d::tf tf;
    tf.pos = m3d::vec3(x, y, z);
    tf.rot = m3d::quat(1, 0, 0, 0);
    return tf;
}

// ── create_collider ──────────────────────────────────────────────────────────────

TEST(add_increases_collider_count)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    ASSERT_EQ(cc.count(), 0u);

    rbps::ColliderParams p;
    p.shape   = rbc::Sphere(1.0);
    p.body_id = 0;

    rbps::create_collider(cc, bp, p, identity_tf());
    ASSERT_EQ(cc.count(), 1u);

    rbps::create_collider(cc, bp, p, identity_tf());
    ASSERT_EQ(cc.count(), 2u);
}

TEST(add_stores_shape_correctly)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape = rbc::Sphere(2.5);

    u_int32_t id = rbps::create_collider(cc, bp, p, identity_tf());
    size_t  idx = cc.index_of(id);

    ASSERT_TRUE(cc.shape[idx].is<rbc::Sphere>());
    ASSERT_NEAR(cc.shape[idx].get<rbc::Sphere>().radius, 2.5, 1e-6);
}

TEST(add_stores_material_properties)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape            = rbc::Sphere(1.0);
    p.restitution      = 0.8;
    p.static_friction  = 0.4;
    p.dynamic_friction = 0.3;

    u_int32_t id  = rbps::create_collider(cc, bp, p, identity_tf());
    size_t  idx = cc.index_of(id);

    ASSERT_NEAR(cc.restitution[idx],      0.8, 1e-6);
    ASSERT_NEAR(cc.static_friction[idx],  0.4, 1e-6);
    ASSERT_NEAR(cc.dynamic_friction[idx], 0.3, 1e-6);
}

TEST(add_stores_body_id)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape   = rbc::Sphere(1.0);
    p.body_id = 42u;

    u_int32_t id  = rbps::create_collider(cc, bp, p, identity_tf());
    size_t  idx = cc.index_of(id);

    ASSERT_EQ(cc.body_id[idx], 42u);
}

TEST(add_stores_local_offset)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape     = rbc::Sphere(1.0);
    p.local_pos = m3d::vec3(1, 2, 3);

    u_int32_t id  = rbps::create_collider(cc, bp, p, identity_tf());
    size_t  idx = cc.index_of(id);

    ASSERT_NEAR(cc.local_pos[idx].x, 1.0, 1e-6);
    ASSERT_NEAR(cc.local_pos[idx].y, 2.0, 1e-6);
    ASSERT_NEAR(cc.local_pos[idx].z, 3.0, 1e-6);
}

TEST(add_static_flag_stored)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p_dyn;
    p_dyn.shape     = rbc::Sphere(1.0);
    p_dyn.is_static = false;

    rbps::ColliderParams p_sta;
    p_sta.shape     = rbc::Sphere(1.0);
    p_sta.is_static = true;

    u_int32_t dyn_id = rbps::create_collider(cc, bp, p_dyn, identity_tf());
    u_int32_t sta_id = rbps::create_collider(cc, bp, p_sta, identity_tf());

    ASSERT_FALSE(cc.is_static[cc.index_of(dyn_id)]);
    ASSERT_TRUE (cc.is_static[cc.index_of(sta_id)]);
}

TEST(add_registers_aabb_in_broad_phase)
{
    // After adding a collider the broad phase endpoint list should contain
    // 2 entries per object (min + max on the sweep axis).
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape = rbc::Sphere(1.0);

    ASSERT_EQ(bp.endpoints.size(), 0u);
    rbps::create_collider(cc, bp, p, identity_tf());
    ASSERT_EQ(bp.endpoints.size(), 2u); // min + max
    rbps::create_collider(cc, bp, p, identity_tf());
    ASSERT_EQ(bp.endpoints.size(), 4u);
}

// ── collider_remove ───────────────────────────────────────────────────────────

TEST(remove_decreases_collider_count)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape = rbc::Sphere(1.0);

    u_int32_t id = rbps::create_collider(cc, bp, p, identity_tf());
    ASSERT_EQ(cc.count(), 1u);

    rbps::collider_remove(cc, bp, id);
    ASSERT_EQ(cc.count(), 0u);
}

TEST(remove_unregisters_from_broad_phase)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape = rbc::Sphere(1.0);

    u_int32_t id = rbps::create_collider(cc, bp, p, identity_tf());
    ASSERT_EQ(bp.endpoints.size(), 2u);

    rbps::collider_remove(cc, bp, id);
    ASSERT_EQ(bp.endpoints.size(), 0u);
}

TEST(remove_middle_leaves_others_intact)
{
    // Add A, B, C — remove B — A and C must still be queryable
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape = rbc::Sphere(1.0);

    u_int32_t id_a = rbps::create_collider(cc, bp, p, tf_at(-5, 0, 0));
    u_int32_t id_b = rbps::create_collider(cc, bp, p, tf_at( 0, 0, 0));
    u_int32_t id_c = rbps::create_collider(cc, bp, p, tf_at( 5, 0, 0));

    rbps::collider_remove(cc, bp, id_b);

    ASSERT_EQ(cc.count(), 2u);

    // ivc stable IDs: a and c are still resolvable
    size_t idx_a = cc.index_of(id_a);
    size_t idx_c = cc.index_of(id_c);

    ASSERT_TRUE(cc.shape[idx_a].is<rbc::Sphere>());
    ASSERT_TRUE(cc.shape[idx_c].is<rbc::Sphere>());
}

TEST(remove_then_add_reuses_bp_slot)
{
    // After remove + add, the broad phase free list recycles the slot.
    // Total endpoint count should stay at 2 (one active collider).
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape = rbc::Sphere(1.0);

    u_int32_t first = rbps::create_collider(cc, bp, p, identity_tf());
    rbc::BPHandle first_bph = cc.bp_handle[cc.index_of(first)];

    rbps::collider_remove(cc, bp, first);

    u_int32_t second = rbps::create_collider(cc, bp, p, identity_tf());
    rbc::BPHandle second_bph = cc.bp_handle[cc.index_of(second)];

    // BPHandle should be recycled (same slot)
    ASSERT_EQ(second_bph, first_bph);
    ASSERT_EQ(bp.endpoints.size(), 2u);
}

// ── collider_world_tf ─────────────────────────────────────────────────────────

TEST(world_tf_no_local_offset)
{
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape     = rbc::Sphere(1.0);
    p.local_pos = m3d::vec3(0, 0, 0);
    p.local_rot = m3d::quat(1, 0, 0, 0);

    u_int32_t id  = rbps::create_collider(cc, bp, p, identity_tf());
    size_t  idx = cc.index_of(id);

    m3d::tf world = rbps::collider_world_tf(cc, idx,
                                            m3d::vec3(3, 4, 5),
                                            m3d::quat(1, 0, 0, 0));

    ASSERT_NEAR(world.pos.x, 3.0, 1e-5);
    ASSERT_NEAR(world.pos.y, 4.0, 1e-5);
    ASSERT_NEAR(world.pos.z, 5.0, 1e-5);
}

TEST(world_tf_with_local_offset)
{
    // Body at (10, 0, 0), collider offset (0, 2, 0) → world pos (10, 2, 0)
    rbps::ColliderCollection cc;
    rbc::BroadPhaseState bp;
    rbc::broad_phase_init(bp);

    rbps::ColliderParams p;
    p.shape     = rbc::Sphere(1.0);
    p.local_pos = m3d::vec3(0, 2, 0);
    p.local_rot = m3d::quat(1, 0, 0, 0);

    u_int32_t id  = rbps::create_collider(cc, bp, p, identity_tf());
    size_t  idx = cc.index_of(id);

    m3d::tf world = rbps::collider_world_tf(cc, idx,
                                            m3d::vec3(10, 0, 0),
                                            m3d::quat(1, 0, 0, 0));

    ASSERT_NEAR(world.pos.x, 10.0, 1e-5);
    ASSERT_NEAR(world.pos.y,  2.0, 1e-5);
    ASSERT_NEAR(world.pos.z,  0.0, 1e-5);
}

TEST_SUITE(
    RUN_TEST(add_increases_collider_count),
    RUN_TEST(add_stores_shape_correctly),
    RUN_TEST(add_stores_material_properties),
    RUN_TEST(add_stores_body_id),
    RUN_TEST(add_stores_local_offset),
    RUN_TEST(add_static_flag_stored),
    RUN_TEST(add_registers_aabb_in_broad_phase),
    RUN_TEST(remove_decreases_collider_count),
    RUN_TEST(remove_unregisters_from_broad_phase),
    RUN_TEST(remove_middle_leaves_others_intact),
    RUN_TEST(remove_then_add_reuses_bp_slot),
    RUN_TEST(world_tf_no_local_offset),
    RUN_TEST(world_tf_with_local_offset)
)