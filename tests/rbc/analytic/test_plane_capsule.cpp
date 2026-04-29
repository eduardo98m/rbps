#include "tests/test_helper.hpp"
#include "tests/math3d_helpers.hpp"
#include "tests/rbc/collision_helpers.hpp"

static rbc::Shape make_plane_y() { return rbc::Plane(m3d::vec3(0, 1, 0), 0.0); }

TEST(capsule_plane_above_no_hit)
{
    // Lowest point = 2 - 1 - 0.5 = 0.5 → no hit.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), pB = make_plane_y();
    auto tfA = test::tf_at(0, 2, 0);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Capsule, rbc::Plane>(cA, tfA, pB, tfP, c)));
}

TEST(capsule_plane_upright_penetrating)
{
    // Centre Y=1.0. Bottom cap-sphere centre at Y=0, r=0.5 → pen=0.5.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), pB = make_plane_y();
    auto tfA = test::tf_at(0, 1.0, 0);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Plane>(cA, tfA, pB, tfP, c)));
    ASSERT_NEAR(test::depth(c), 0.5, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST(capsule_plane_horizontal_penetrating)
{
    // Capsule along X, centre Y=0.3, r=0.5 → pen=0.2.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), pB = make_plane_y();
    m3d::tf tfA = test::tf_at(0, 0.3, 0);
    tfA.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kHalfPi);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Plane>(cA, tfA, pB, tfP, c)));
    ASSERT_NEAR(test::depth(c), 0.2, 0.01);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(capsule_plane_tilted_45_penetrating)
{
    // Capsule tilted 45° around Z, centre Y=1.0.
    // pen = sin(45°) * half_height + radius - centre_y = √2/2 + 0.5 - 1.0 ≈ 0.207.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), pB = make_plane_y();
    m3d::tf tfA = test::tf_at(0, 1.0, 0);
    tfA.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kQuarterPi);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Plane>(cA, tfA, pB, tfP, c)));
    const m3d::scalar expected = m3d::sin(test::kQuarterPi) + 0.5 - 1.0;
    ASSERT_NEAR(test::depth(c), expected, 0.01);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST_SUITE(
    RUN_TEST(capsule_plane_above_no_hit),
    RUN_TEST(capsule_plane_upright_penetrating),
    RUN_TEST(capsule_plane_horizontal_penetrating),
    RUN_TEST(capsule_plane_tilted_45_penetrating),
)
