// ============================================================================
//  tests/visr/test_snapshot_builder.cpp
//
//  Tests for visr::build_snapshot() — verifies that World state is correctly
//  converted to FrameSnapshot with the right IDs, values, and shape types.
// ============================================================================

#include "tests/visr/visr_test_helper.hpp"
#include "visr/SnapshotBuilder.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

static visr::FrameSnapshot snap_at_frame0(const rbps::World &w)
{
    return visr::build_snapshot(w, 0, 0.0);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

static void test_empty_world()
{
    VISR_TEST("empty world → empty snapshot");

    rbps::World w;
    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.bodies.empty());
    VISR_ASSERT(snap.colliders.empty());
    VISR_ASSERT(snap.contacts.empty());
    VISR_ASSERT(snap.joints.empty());
    VISR_ASSERT(snap.constraints.empty());
    VISR_ASSERT(snap.frame_index == 0);
    VISR_APPROX_EQ(snap.sim_time, 0.0, 1e-9);

    VISR_PASS();
}

static void test_frame_metadata()
{
    VISR_TEST("frame_index and sim_time are passed through");

    rbps::World w;
    w.timestep = 1.0 / 60.0;
    w.substeps = 10;

    auto snap = visr::build_snapshot(w, 42, 3.14);

    VISR_ASSERT(snap.frame_index == 42);
    VISR_APPROX_EQ(snap.sim_time, 3.14, 1e-9);
    VISR_APPROX_EQ(snap.timestep, 1.0 / 60.0, 1e-9);
    VISR_ASSERT(snap.substeps == 10);

    VISR_PASS();
}

static void test_single_dynamic_body()
{
    VISR_TEST("single dynamic body: id, position, type");

    rbps::World w;
    const m3d::vec3 pos{1.0, 2.0, 3.0};
    const uint32_t id = make_dynamic_body(w, pos, 5.0);

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.bodies.size() == 1);

    const auto &b = snap.bodies[0];
    VISR_ASSERT(b.id == id);
    VISR_ASSERT(!b.is_static);
    VISR_APPROX_EQ(b.position.x, 1.0, 1e-9);
    VISR_APPROX_EQ(b.position.y, 2.0, 1e-9);
    VISR_APPROX_EQ(b.position.z, 3.0, 1e-9);
    VISR_APPROX_EQ(b.mass, 5.0, 1e-9);

    VISR_PASS();
}

static void test_static_body_flag()
{
    VISR_TEST("static body: is_static flag is true");

    rbps::World w;
    make_static_body(w, {0, -5, 0});

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.bodies.size() == 1);
    VISR_ASSERT(snap.bodies[0].is_static);

    VISR_PASS();
}

static void test_multiple_bodies_count()
{
    VISR_TEST("three bodies: snapshot count and IDs are distinct");

    rbps::World w;
    const uint32_t id0 = make_dynamic_body(w, {0, 0, 0});
    const uint32_t id1 = make_dynamic_body(w, {1, 0, 0});
    const uint32_t id2 = make_static_body (w, {0, -5, 0});

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.bodies.size() == 3);

    // All IDs should be represented exactly once
    int found0 = 0, found1 = 0, found2 = 0;
    for (auto &b : snap.bodies)
    {
        if (b.id == id0) ++found0;
        if (b.id == id1) ++found1;
        if (b.id == id2) ++found2;
    }
    VISR_ASSERT(found0 == 1);
    VISR_ASSERT(found1 == 1);
    VISR_ASSERT(found2 == 1);

    VISR_PASS();
}

static void test_collider_body_linkage()
{
    VISR_TEST("collider.body_id matches the body that owns it");

    rbps::World w;
    const uint32_t body_id = make_dynamic_body(w, {0, 0, 0});
    make_box_collider(w, body_id);

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.colliders.size() == 1);
    VISR_ASSERT(snap.colliders[0].body_id == body_id);

    VISR_PASS();
}

static void test_shape_sphere()
{
    VISR_TEST("sphere collider: ShapeSnap holds radius");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    make_sphere_collider(w, bid, 1.25);

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.colliders.size() == 1);
    const auto *s = std::get_if<visr::SpherSnap>(&snap.colliders[0].shape);
    VISR_ASSERT_MSG(s != nullptr, "expected SpherSnap variant");
    VISR_APPROX_EQ(s->radius, 1.25, 1e-6);

    VISR_PASS();
}

static void test_shape_box()
{
    VISR_TEST("box collider: ShapeSnap holds half_extents");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);
    make_box_collider(w, bid, {1.0, 2.0, 3.0});

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.colliders.size() == 1);
    const auto *b = std::get_if<visr::BoxSnap>(&snap.colliders[0].shape);
    VISR_ASSERT_MSG(b != nullptr, "expected BoxSnap variant");
    VISR_APPROX_EQ(b->half_extents.x, 1.0, 1e-6);
    VISR_APPROX_EQ(b->half_extents.y, 2.0, 1e-6);
    VISR_APPROX_EQ(b->half_extents.z, 3.0, 1e-6);

    VISR_PASS();
}

static void test_shape_capsule()
{
    VISR_TEST("capsule collider: ShapeSnap holds radius and half_height");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);

    rbps::ColliderParams cp{};
    cp.body_id  = bid;
    cp.local_pos= m3d::vec3{0,0,0};
    cp.local_rot= m3d::quat{0,0,0,1};
    cp.shape    = rbc::Shape(rbc::Capsule{0.7, 0.3}); // half_height=0.7, radius=0.3
    cp.restitution = 0.3f; cp.static_friction = 0.5f; cp.dynamic_friction = 0.3f;
    w.create_collider(cp);

    auto snap = snap_at_frame0(w);

    const auto *c = std::get_if<visr::CapsuleSnap>(&snap.colliders[0].shape);
    VISR_ASSERT_MSG(c != nullptr, "expected CapsuleSnap variant");
    VISR_APPROX_EQ(c->radius,      0.3, 1e-6);
    VISR_APPROX_EQ(c->half_height, 0.7, 1e-6);

    VISR_PASS();
}

static void test_shape_plane()
{
    VISR_TEST("plane collider: ShapeSnap holds normal and d");

    rbps::World w;
    const uint32_t bid = make_static_body(w);

    rbps::ColliderParams cp{};
    cp.body_id  = bid;
    cp.local_pos= m3d::vec3{0,0,0};
    cp.local_rot= m3d::quat{0,0,0,1};
    cp.shape    = rbc::Shape(rbc::Plane{{0,1,0}, -2.0});
    cp.restitution = 0.3f; cp.static_friction = 0.5f; cp.dynamic_friction = 0.3f;
    w.create_collider(cp);

    auto snap = snap_at_frame0(w);

    const auto *p = std::get_if<visr::PlaneSnap>(&snap.colliders[0].shape);
    VISR_ASSERT_MSG(p != nullptr, "expected PlaneSnap variant");
    VISR_APPROX_EQ(p->normal.y, 1.0,  1e-6);
    VISR_APPROX_EQ(p->distance, -2.0, 1e-6);

    VISR_PASS();
}

static void test_shape_cone()
{
    VISR_TEST("cone collider: ShapeSnap holds base_radius and half_height");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);

    rbps::ColliderParams cp{};
    cp.body_id  = bid;
    cp.local_pos= m3d::vec3{0,0,0};
    cp.local_rot= m3d::quat{0,0,0,1};
    cp.shape    = rbc::Shape(rbc::Cone{0.8, 0.4}); // half_height=0.8, base_radius=0.4
    cp.restitution = 0.3f; cp.static_friction = 0.5f; cp.dynamic_friction = 0.3f;
    w.create_collider(cp);

    auto snap = snap_at_frame0(w);

    const auto *c = std::get_if<visr::ConeSnap>(&snap.colliders[0].shape);
    VISR_ASSERT_MSG(c != nullptr, "expected ConeSnap variant");
    VISR_APPROX_EQ(c->radius, 0.4, 1e-6); // base_radius maps to radius
    VISR_APPROX_EQ(c->height, 0.8, 1e-6); // half_height maps to height

    VISR_PASS();
}

static void test_shape_ellipsoid()
{
    VISR_TEST("ellipsoid collider: ShapeSnap holds semi_axes");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);

    rbps::ColliderParams cp{};
    cp.body_id  = bid;
    cp.local_pos= m3d::vec3{0,0,0};
    cp.local_rot= m3d::quat{0,0,0,1};
    cp.shape    = rbc::Shape(rbc::Ellipsoid{{1.0, 2.0, 0.5}});
    cp.restitution = 0.3f; cp.static_friction = 0.5f; cp.dynamic_friction = 0.3f;
    w.create_collider(cp);

    auto snap = snap_at_frame0(w);

    const auto *e = std::get_if<visr::EllipsoidSnap>(&snap.colliders[0].shape);
    VISR_ASSERT_MSG(e != nullptr, "expected EllipsoidSnap variant");
    VISR_APPROX_EQ(e->semi_axes.x, 1.0, 1e-6);
    VISR_APPROX_EQ(e->semi_axes.y, 2.0, 1e-6);
    VISR_APPROX_EQ(e->semi_axes.z, 0.5, 1e-6);

    VISR_PASS();
}

static void test_collider_world_transform()
{
    VISR_TEST("collider world_pos reflects body position + local offset");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w, {3.0, 0.0, 0.0});

    rbps::ColliderParams cp{};
    cp.body_id  = bid;
    cp.local_pos= m3d::vec3{0.5, 0.0, 0.0}; // offset in local space
    cp.local_rot= m3d::quat{0,0,0,1};
    cp.shape    = rbc::Shape(rbc::Sphere{0.5});
    cp.restitution = 0.3f; cp.static_friction = 0.5f; cp.dynamic_friction = 0.3f;
    w.create_collider(cp);

    auto snap = snap_at_frame0(w);

    VISR_ASSERT(snap.colliders.size() == 1);
    // world_pos = body_pos{3,0,0} + rotate(identity, local_pos{0.5,0,0}) = {3.5,0,0}
    VISR_APPROX_EQ(snap.colliders[0].world_pos.x, 3.5, 1e-6);
    VISR_APPROX_EQ(snap.colliders[0].world_pos.y, 0.0, 1e-6);

    VISR_PASS();
}

static void test_material_properties()
{
    VISR_TEST("collider material props (restitution, friction) are copied");

    rbps::World w;
    const uint32_t bid = make_dynamic_body(w);

    rbps::ColliderParams cp{};
    cp.body_id          = bid;
    cp.local_pos        = m3d::vec3{0,0,0};
    cp.local_rot        = m3d::quat{0,0,0,1};
    cp.shape            = rbc::Shape(rbc::Sphere{0.5});
    cp.restitution      = 0.75;
    cp.static_friction  = 0.6;
    cp.dynamic_friction = 0.4;
    w.create_collider(cp);

    auto snap = snap_at_frame0(w);

    VISR_APPROX_EQ(snap.colliders[0].restitution,      0.75, 1e-6);
    VISR_APPROX_EQ(snap.colliders[0].static_friction,  0.6,  1e-6);
    VISR_APPROX_EQ(snap.colliders[0].dynamic_friction, 0.4,  1e-6);

    VISR_PASS();
}

static void test_snapshot_does_not_mutate_world()
{
    VISR_TEST("build_snapshot is const — body count unchanged after snap");

    rbps::World w;
    make_dynamic_body(w, {0,1,0});
    make_dynamic_body(w, {1,0,0});

    const uint32_t count_before = w.bodies.count();
    snap_at_frame0(w);
    VISR_ASSERT(w.bodies.count() == count_before);

    VISR_PASS();
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    std::printf("=== test_snapshot_builder ===\n");

    test_empty_world();
    test_frame_metadata();
    test_single_dynamic_body();
    test_static_body_flag();
    test_multiple_bodies_count();
    test_collider_body_linkage();
    test_shape_sphere();
    test_shape_box();
    test_shape_capsule();
    test_shape_plane();
    test_shape_cone();
    test_shape_ellipsoid();
    test_collider_world_transform();
    test_material_properties();
    test_snapshot_does_not_mutate_world();

    std::printf("All tests passed.\n");
    return 0;
}