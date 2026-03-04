#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"

// Plane at Y=0, normal=(0,1,0).
static m3d::tf plane_tf() { m3d::tf t; t.pos = m3d::vec3(0,0,0); return t; }
static rbc::Shape make_plane_y() { return rbc::Plane(m3d::vec3(0,1,0), 0.0); }

TEST(capsule_plane_above_no_hit)
{
    // Capsule half_height=1, radius=0.5, upright Y-axis, centre at Y=2.
    // Lowest point = 2 - 1 - 0.5 = 0.5 > 0 → no hit.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 2, 0);

    rbc::Contact c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Plane>::test(
        cA.get<rbc::Capsule>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);
    ASSERT_FALSE(hit);
}

TEST(capsule_plane_upright_penetrating)
{
    // Centre at Y=1.0. Lowest cap-sphere centre at Y=0, radius=0.5 → pen=0.5.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 1.0, 0);

    rbc::Contact c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Plane>::test(
        cA.get<rbc::Capsule>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.penetration_depth, 0.5, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

TEST(capsule_plane_horizontal_penetrating)
{
    // Capsule lying horizontal (along X), centre at Y=0.3. Radius=0.5.
    // Lowest point = Y = 0.3 - 0.5 = -0.2 → pen = 0.2.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0.3, 0);
    tfA.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 2.0); // axis → X

    rbc::Contact c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Plane>::test(
        cA.get<rbc::Capsule>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.penetration_depth, 0.2, 0.01);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(capsule_plane_tilted_45_penetrating)
{
    // Capsule tilted 45° around Z. The deepest point is the bottom endcap sphere
    // projected along -Y. With half_height=1, radius=0.5:
    //   bottom_endpoint_y = centre_y - half_height * sin(45°) = centre_y - 0.707
    //   deepest_y = bottom_endpoint_y - radius = centre_y - 1.207
    // Set centre_y = 1.0 → pen = 1.207 - 1.0 = 0.207.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 1.0, 0);
    tfA.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 4.0);

    rbc::Contact c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Plane>::test(
        cA.get<rbc::Capsule>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    m3d::scalar expected_pen = 1.0 * m3d::sin(m3d::PI / 4.0) + 0.5 - 1.0;
    ASSERT_NEAR(c.penetration_depth, expected_pen, 0.01);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST_SUITE(
    RUN_TEST(capsule_plane_above_no_hit),
    RUN_TEST(capsule_plane_upright_penetrating),
    RUN_TEST(capsule_plane_horizontal_penetrating),
    RUN_TEST(capsule_plane_tilted_45_penetrating),
)