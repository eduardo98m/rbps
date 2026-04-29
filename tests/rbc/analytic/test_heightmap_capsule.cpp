#include "tests/test_helper.hpp"
#include "tests/math3d_helpers.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "tests/rbc/heightmap_helpers.hpp"

TEST(capsule_heightmap_upright_above_no_hit)
{
    // Centre Y=2: bottom cap at Y=0.5 → above the Y=0 flat map.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(1, 2, 1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Capsule, rbc::Heightmap>(cA, tfA, hmB, tfB, c)));
}

TEST(capsule_heightmap_upright_penetrating)
{
    // Centre at Y=1.0. Bottom endcap sphere at Y=0, radius=0.5 → pen=0.5.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(1, 1.0, 1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Heightmap>(cA, tfA, hmB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 0.5, 0.05);
    ASSERT_NEAR(c.normal.y, 1.0, 0.05);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST(capsule_heightmap_horizontal_penetrating)
{
    // Capsule lying along X, centre at (1, 0.3, 1). Radius=0.5 → pen=0.2.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape cA = rbc::Capsule(0.5, 0.5), hmB = rbc::Heightmap(hd);
    m3d::tf tfA = test::tf_at(1, 0.3, 1);
    tfA.rot = test::axis_quat(m3d::vec3(0, 0, 1), test::kHalfPi);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Capsule, rbc::Heightmap>(cA, tfA, hmB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 0.2, 0.03);
    ASSERT_NEAR(c.normal.y, 1.0, 0.05);
}

TEST(capsule_heightmap_outside_grid_no_hit)
{
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape cA = rbc::Capsule(1.0, 0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(10, 0.3, 1); // outside XZ footprint
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Capsule, rbc::Heightmap>(cA, tfA, hmB, tfB, c)));
}

TEST_SUITE(
    RUN_TEST(capsule_heightmap_upright_above_no_hit),
    RUN_TEST(capsule_heightmap_upright_penetrating),
    RUN_TEST(capsule_heightmap_horizontal_penetrating),
    RUN_TEST(capsule_heightmap_outside_grid_no_hit),
)
