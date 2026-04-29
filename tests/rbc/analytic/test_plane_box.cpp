#include "tests/test_helper.hpp"
#include "tests/math3d_helpers.hpp"
#include "tests/rbc/collision_helpers.hpp"

// Plane at Y=0, normal=(0,1,0).
static rbc::Shape make_plane_y() { return rbc::Plane(m3d::vec3(0, 1, 0), 0.0); }

TEST(box_plane_above_no_hit)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), pB = make_plane_y();
    auto tfA = test::tf_at(0, 2, 0);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Box, rbc::Plane>(bA, tfA, pB, tfP, c)));
}

TEST(box_plane_face_penetrating)
{
    // Lowest face at Y=-0.3 → penetration = 0.3.
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), pB = make_plane_y();
    auto tfA = test::tf_at(0, 0.7, 0);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Plane>(bA, tfA, pB, tfP, c)));
    ASSERT_NEAR(test::depth(c), 0.3, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST(box_plane_corner_touching_no_hit)
{
    // Rotated 45° around Z; lowest corner at Y=0 → just touches.
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), pB = make_plane_y();
    m3d::tf tfA = test::tf_at(0, m3d::sqrt(2.0), 0);
    tfA.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kQuarterPi);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Box, rbc::Plane>(bA, tfA, pB, tfP, c)));
}

TEST(box_plane_rotated_penetrating)
{
    // Same rotated box, centre Y=1.2. Penetration ≈ √2 - 1.2 ≈ 0.214.
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), pB = make_plane_y();
    m3d::tf tfA = test::tf_at(0, 1.2, 0);
    tfA.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kQuarterPi);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Plane>(bA, tfA, pB, tfP, c)));
    ASSERT_NEAR(test::depth(c), m3d::sqrt(2.0) - 1.2, 0.01);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(box_plane_fully_below)
{
    // Box centre at Y=-3. Deepest point Y=-4. Penetration = 4.0.
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), pB = make_plane_y();
    auto tfA = test::tf_at(0, -3, 0);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Plane>(bA, tfA, pB, tfP, c)));
    ASSERT_NEAR(test::depth(c), 4.0, 0.001);
}

TEST_SUITE(
    RUN_TEST(box_plane_above_no_hit),
    RUN_TEST(box_plane_face_penetrating),
    RUN_TEST(box_plane_corner_touching_no_hit),
    RUN_TEST(box_plane_rotated_penetrating),
    RUN_TEST(box_plane_fully_below),
)
