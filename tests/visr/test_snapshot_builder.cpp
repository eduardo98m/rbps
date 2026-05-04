#include "tests/test_helper.hpp"
#include "tests/visr/visr_test_helper.hpp"
#include "visr/SnapshotBuilder.hpp"

using namespace visr;

// ── Helpers ───────────────────────────────────────────────────────────────────

static FrameSnapshot snap0(const rbps::World &w)
{
    return build_snapshot(w, 0, 0.0);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(empty_world)
{
    rbps::World w;
    auto snap = snap0(w);
    ASSERT_TRUE(snap.bodies.empty());
    ASSERT_TRUE(snap.colliders.empty());
    ASSERT_TRUE(snap.contacts.empty());
    ASSERT_TRUE(snap.joints.empty());
    ASSERT_TRUE(snap.constraints.empty());
    ASSERT_EQ(snap.frame_index, 0u);
    ASSERT_NEAR(snap.sim_time, 0.0);
}

TEST(frame_metadata)
{
    rbps::World w;
    w.timestep = 1.0 / 60.0;
    w.substeps = 10;
    auto snap = build_snapshot(w, 42, 3.14);
    ASSERT_EQ(snap.frame_index, 42u);
    ASSERT_NEAR(snap.sim_time, 3.14);
    ASSERT_NEAR(snap.timestep, 1.0 / 60.0);
    ASSERT_EQ(snap.substeps, 10);
}

TEST(single_dynamic_body_position_and_type)
{
    rbps::World w;
    const uint32_t id = make_dynamic_body(w, {1.0, 2.0, 3.0}, 5.0);
    auto snap = snap0(w);
    ASSERT_EQ(snap.bodies.size(), 1u);
    ASSERT_EQ(snap.bodies[0].id, id);
    ASSERT_FALSE(snap.bodies[0].is_static);
    ASSERT_NEAR(snap.bodies[0].position.x, 1.0);
    ASSERT_NEAR(snap.bodies[0].position.y, 2.0);
    ASSERT_NEAR(snap.bodies[0].position.z, 3.0);
    ASSERT_NEAR(snap.bodies[0].mass, 5.0);
}

TEST(static_body_flag)
{
    rbps::World w;
    make_static_body(w, {0, -5, 0});
    auto snap = snap0(w);
    ASSERT_EQ(snap.bodies.size(), 1u);
    ASSERT_TRUE(snap.bodies[0].is_static);
}

TEST(multiple_bodies_distinct_ids)
{
    rbps::World w;
    const uint32_t id0 = make_dynamic_body(w, {0, 0, 0});
    const uint32_t id1 = make_dynamic_body(w, {1, 0, 0});
    const uint32_t id2 = make_static_body (w, {0,-5, 0});
    auto snap = snap0(w);
    ASSERT_EQ(snap.bodies.size(), 3u);
    int f0 = 0, f1 = 0, f2 = 0;
    for (auto &b : snap.bodies)
    {
        if (b.id == id0) ++f0;
        if (b.id == id1) ++f1;
        if (b.id == id2) ++f2;
    }
    ASSERT_EQ(f0, 1); ASSERT_EQ(f1, 1); ASSERT_EQ(f2, 1);
}

TEST(collider_body_linkage)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    make_box_collider(w, bid);
    auto snap = snap0(w);
    ASSERT_EQ(snap.colliders.size(), 1u);
    ASSERT_EQ(snap.colliders[0].body_id, bid);
}

TEST(shape_sphere)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    make_sphere_collider(w, bid, 1.25);
    auto snap = snap0(w);
    const auto *s = std::get_if<SpherSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(s != nullptr);
    ASSERT_NEAR(s->radius, 1.25, 1e-6);
}

TEST(shape_box)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    make_box_collider(w, bid, {1.0, 2.0, 3.0});
    auto snap = snap0(w);
    const auto *b = std::get_if<BoxSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(b != nullptr);
    ASSERT_NEAR(b->half_extents.x, 1.0, 1e-6);
    ASSERT_NEAR(b->half_extents.y, 2.0, 1e-6);
    ASSERT_NEAR(b->half_extents.z, 3.0, 1e-6);
}

TEST(shape_capsule)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    rbps::ColliderParams cp{};
    cp.body_id   = bid;
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape     = rbc::Shape(rbc::Capsule{0.7, 0.3});
    cp.restitution = cp.static_friction = cp.dynamic_friction = 0.3;
    w.create_collider(cp);
    auto snap = snap0(w);
    const auto *c = std::get_if<CapsuleSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(c != nullptr);
    ASSERT_NEAR(c->half_height, 0.7, 1e-6);
    ASSERT_NEAR(c->radius,      0.3, 1e-6);
}

TEST(shape_plane)
{
    rbps::World w;
    const uint32_t bid = make_static_body(w);
    rbps::ColliderParams cp{};
    cp.body_id   = bid;
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape     = rbc::Shape(rbc::Plane{{0, 1, 0}, -2.0});
    cp.restitution = cp.static_friction = cp.dynamic_friction = 0.3;
    w.create_collider(cp);
    auto snap = snap0(w);
    const auto *p = std::get_if<PlaneSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(p != nullptr);
    ASSERT_NEAR(p->normal.y,  1.0,  1e-6);
    ASSERT_NEAR(p->distance, -2.0,  1e-6);
}

TEST(shape_cone)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    rbps::ColliderParams cp{};
    cp.body_id   = bid;
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape     = rbc::Shape(rbc::Cone{0.8, 0.4});
    cp.restitution = cp.static_friction = cp.dynamic_friction = 0.3;
    w.create_collider(cp);
    auto snap = snap0(w);
    const auto *c = std::get_if<ConeSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(c != nullptr);
    ASSERT_NEAR(c->height, 0.8, 1e-6);
    ASSERT_NEAR(c->radius, 0.4, 1e-6);
}

TEST(shape_ellipsoid)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    rbps::ColliderParams cp{};
    cp.body_id   = bid;
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape     = rbc::Shape(rbc::Ellipsoid{{1.0, 2.0, 0.5}});
    cp.restitution = cp.static_friction = cp.dynamic_friction = 0.3;
    w.create_collider(cp);
    auto snap = snap0(w);
    const auto *e = std::get_if<EllipsoidSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(e != nullptr);
    ASSERT_NEAR(e->semi_axes.x, 1.0, 1e-6);
    ASSERT_NEAR(e->semi_axes.y, 2.0, 1e-6);
    ASSERT_NEAR(e->semi_axes.z, 0.5, 1e-6);
}

TEST(shape_cylinder)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    rbps::ColliderParams cp{};
    cp.body_id   = bid;
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape     = rbc::Shape(rbc::Cylinder{1.0, 0.5});
    cp.restitution = cp.static_friction = cp.dynamic_friction = 0.3;
    w.create_collider(cp);
    auto snap = snap0(w);
    const auto *c = std::get_if<CylinderSnap>(&snap.colliders[0].shape);
    ASSERT_TRUE(c != nullptr);
    ASSERT_NEAR(c->base_radius, 0.5, 1e-6);
    ASSERT_NEAR(c->half_height, 1.0, 1e-6);
}

TEST(collider_world_transform_with_offset)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w, {3.0, 0.0, 0.0});
    rbps::ColliderParams cp{};
    cp.body_id   = bid;
    cp.local_pos = m3d::vec3{0.5, 0.0, 0.0};
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape     = rbc::Shape(rbc::Sphere{0.5});
    cp.restitution = cp.static_friction = cp.dynamic_friction = 0.3;
    w.create_collider(cp);
    auto snap = snap0(w);
    // world_pos = body{3,0,0} + rotate(identity, local{0.5,0,0}) = {3.5,0,0}
    ASSERT_NEAR(snap.colliders[0].world_pos.x, 3.5, 1e-6);
    ASSERT_NEAR(snap.colliders[0].world_pos.y, 0.0, 1e-6);
}

TEST(collider_material_properties)
{
    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    rbps::ColliderParams cp{};
    cp.body_id          = bid;
    cp.local_rot        = m3d::quat{1, 0, 0, 0};
    cp.shape            = rbc::Shape(rbc::Sphere{0.5});
    cp.restitution      = 0.75;
    cp.static_friction  = 0.6;
    cp.dynamic_friction = 0.4;
    w.create_collider(cp);
    auto snap = snap0(w);
    ASSERT_NEAR(snap.colliders[0].restitution,      0.75, 1e-6);
    ASSERT_NEAR(snap.colliders[0].static_friction,  0.6,  1e-6);
    ASSERT_NEAR(snap.colliders[0].dynamic_friction, 0.4,  1e-6);
}

TEST(build_snapshot_does_not_mutate_world)
{
    rbps::World w;
    make_dynamic_body(w, {0,1,0});
    make_dynamic_body(w, {1,0,0});
    const uint32_t count_before = w.bodies.count();
    snap0(w);
    ASSERT_EQ(w.bodies.count(), count_before);
}

TEST_SUITE(
    RUN_TEST(empty_world),
    RUN_TEST(frame_metadata),
    RUN_TEST(single_dynamic_body_position_and_type),
    RUN_TEST(static_body_flag),
    RUN_TEST(multiple_bodies_distinct_ids),
    RUN_TEST(collider_body_linkage),
    RUN_TEST(shape_sphere),
    RUN_TEST(shape_box),
    RUN_TEST(shape_capsule),
    RUN_TEST(shape_plane),
    RUN_TEST(shape_cone),
    RUN_TEST(shape_ellipsoid),
    RUN_TEST(shape_cylinder),
    RUN_TEST(collider_world_transform_with_offset),
    RUN_TEST(collider_material_properties),
    RUN_TEST(build_snapshot_does_not_mutate_world)
)