#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"

// Plane at Y=0, normal=(0,1,0).
static m3d::tf plane_tf() { m3d::tf t; t.pos = m3d::vec3(0,0,0); return t; }
static rbc::Shape make_plane_y() { return rbc::Plane(m3d::vec3(0,1,0), 0.0); }

TEST(box_plane_above_no_hit)
{
    // Unit box (half=1) centred at Y=2. Lowest face at Y=1 → above plane.
    rbc::Shape bA = rbc::Box(m3d::vec3(1,1,1));
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 2, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Plane>::test(
        bA.get<rbc::Box>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);
    ASSERT_FALSE(hit);
}

TEST(box_plane_face_penetrating)
{
    // Unit box centred at Y=0.7. Lowest face at Y=-0.3 → penetration = 0.3.
    rbc::Shape bA = rbc::Box(m3d::vec3(1,1,1));
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0.7, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Plane>::test(
        bA.get<rbc::Box>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 0.3, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

TEST(box_plane_corner_touching_no_hit)
{
    // Box rotated 45° around Z, half=1. Corner reaches sqrt(2) ≈ 1.414 below centre.
    // Place centre at Y = sqrt(2): corner at Y=0, exactly touching → no hit.
    rbc::Shape bA = rbc::Box(m3d::vec3(1,1,1));
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, m3d::sqrt(2.0), 0);
    tfA.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 4.0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Plane>::test(
        bA.get<rbc::Box>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);
    ASSERT_FALSE(hit);
}

TEST(box_plane_rotated_penetrating)
{
    // Same box rotated 45° around Z, centre at Y=1.2. Corner reaches 1.2-sqrt(2)≈-0.214.
    // Penetration ≈ 0.214.
    rbc::Shape bA = rbc::Box(m3d::vec3(1,1,1));
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 1.2, 0);
    tfA.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 4.0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Plane>::test(
        bA.get<rbc::Box>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    m3d::scalar expected_pen = m3d::sqrt(2.0) - 1.2;
    ASSERT_NEAR(c.points[0].penetration_depth, expected_pen, 0.01);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(box_plane_fully_below)
{
    // Box centre at Y=-3. Deepest point at Y=-4. Penetration = 4.0.
    rbc::Shape bA = rbc::Box(m3d::vec3(1,1,1));
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, -3, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Plane>::test(
        bA.get<rbc::Box>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 4.0, 0.001);
}

TEST_SUITE(
    RUN_TEST(box_plane_above_no_hit),
    RUN_TEST(box_plane_face_penetrating),
    RUN_TEST(box_plane_corner_touching_no_hit),
    RUN_TEST(box_plane_rotated_penetrating),
    RUN_TEST(box_plane_fully_below),
)