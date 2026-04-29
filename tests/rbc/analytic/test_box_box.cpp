#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"

// =============================================================================
// BOX vs BOX  (SAT)
// =============================================================================

TEST(box_box_separated_x)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(2.5, 0, 0); // gap of 0.5

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Box, rbc::Box>(bA, tfA, bB, tfB, c)));
}

TEST(box_box_face_overlap_x)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(1.6, 0, 0); // overlap = 0.4

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Box>(bA, tfA, bB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(bA, tfA, bB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.4, 0.01);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.01);
}

TEST(box_box_face_overlap_z)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 0, 1.8); // overlap = 0.2

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Box>(bA, tfA, bB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(bA, tfA, bB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.2, 0.01);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.z), 1.0, 0.01);
}

TEST(box_box_minimum_axis_chosen)
{
    // Three overlapping axes — SAT must pick the smallest one (Z = 0.2)
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0.5, 0.5, 1.8);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Box>(bA, tfA, bB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(bA, tfA, bB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.2, 0.01);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.z), 1.0, 0.01);
}

TEST(box_box_deep_penetration)
{
    // One box mostly inside the other
    rbc::Shape bA = rbc::Box(m3d::vec3(3, 3, 3)), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0.2, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Box, rbc::Box>(bA, tfA, bB, tfB, c)));
    ASSERT_TRUE(test::depth(c) > 0.0);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST_SUITE(
    RUN_TEST(box_box_separated_x),
    RUN_TEST(box_box_face_overlap_x),
    RUN_TEST(box_box_face_overlap_z),
    RUN_TEST(box_box_minimum_axis_chosen),
    RUN_TEST(box_box_deep_penetration))
