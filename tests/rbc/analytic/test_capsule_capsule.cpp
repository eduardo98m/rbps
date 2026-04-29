#include "tests/test_helper.hpp"
#include "tests/math3d_helpers.hpp"
#include "tests/rbc/collision_helpers.hpp"

// All capsules: half_height=1, radius=0.5, local axis=Y.

TEST(capsule_capsule_separated_parallel)
{
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(3, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Capsule, rbc::Capsule>(cA, tfA, cB, tfB, c)));
}

TEST(capsule_capsule_overlap_parallel)
{
    // Parallel capsules 0.8 apart on X. rsum = 1.0 → overlap = 0.2.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0.8, 0, 0);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Capsule>(cA, tfA, cB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(cA, tfA, cB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.2, 0.02);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NORMAL_UNIT(analytic.normal);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.05);
}

TEST(capsule_capsule_overlap_perpendicular)
{
    // cA along Y at origin. cB along X (rotated 90° around Z) at origin.
    // Axes intersect at origin → deep overlap = rsum.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 0, 0);
    m3d::tf tfB = test::tf_at(0, 0, 0);
    tfB.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kHalfPi);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Capsule>(cA, tfA, cB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(cA, tfA, cB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 1.0, 0.05);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NORMAL_UNIT(analytic.normal);
}

TEST(capsule_capsule_overlap_crossing_offset)
{
    // cA along Y, cB along X offset on Z by 0.6. Skew distance 0.6 < rsum 1.0.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 0, 0);
    m3d::tf tfB = test::tf_at(0, 0, 0.6);
    tfB.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kHalfPi);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Capsule>(cA, tfA, cB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(cA, tfA, cB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.4, 0.02);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
}

TEST(capsule_capsule_endpoint_to_endpoint)
{
    // Two parallel Y-axis capsules, top of cA = (0,1,0), bottom of cB = (0,1.4,0).
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 2.4, 0);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Capsule>(cA, tfA, cB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(cA, tfA, cB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NORMAL_UNIT(analytic.normal);
    ASSERT_NEAR(std::abs(analytic.normal.y), 1.0, 0.05);
}

TEST(capsule_capsule_separated_endpoint_gap)
{
    // Centres 3.5 apart on Y. Gap between caps = 0.5.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 3.5, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Capsule, rbc::Capsule>(cA, tfA, cB, tfB, c)));
}

TEST_SUITE(
    RUN_TEST(capsule_capsule_separated_parallel),
    RUN_TEST(capsule_capsule_overlap_parallel),
    RUN_TEST(capsule_capsule_overlap_perpendicular),
    RUN_TEST(capsule_capsule_overlap_crossing_offset),
    RUN_TEST(capsule_capsule_endpoint_to_endpoint),
    RUN_TEST(capsule_capsule_separated_endpoint_gap),
)
