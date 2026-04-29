#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "tests/rbc/heightmap_helpers.hpp"

TEST(sphere_heightmap_flat_above_no_hit)
{
    // Sphere radius=0.5, centre at (1, 1.0, 1). Lowest at Y=0.5 → above flat hmap.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape sA = rbc::Sphere(0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(1, 1.0, 1);
    auto tfB = test::tf_at(0, 0, 0); // heightmap ignores tf

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Heightmap>(sA, tfA, hmB, tfB, c)));
}

TEST(sphere_heightmap_flat_penetrating)
{
    // Sphere radius=0.5, centre at (1, 0.3, 1). Pen = 0.5 - 0.3 = 0.2.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape sA = rbc::Sphere(0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(1, 0.3, 1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Heightmap>(sA, tfA, hmB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 0.2, 0.02);
    ASSERT_NEAR(c.normal.y, 1.0, 0.05);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST(sphere_heightmap_elevated_flat_above_no_hit)
{
    // Heightmap at height=2. Sphere centre at (1, 3.0, 1). Lowest at Y=2.5 > 2.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3(2.0f));
    rbc::Shape sA = rbc::Sphere(0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(1, 3.0, 1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Heightmap>(sA, tfA, hmB, tfB, c)));
}

TEST(sphere_heightmap_elevated_flat_penetrating)
{
    // Heightmap at height=2. Sphere radius=0.5, centre at (1, 2.3, 1). Pen=0.2.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3(2.0f));
    rbc::Shape sA = rbc::Sphere(0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(1, 2.3, 1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Heightmap>(sA, tfA, hmB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 0.2, 0.02);
}

TEST(sphere_heightmap_outside_grid_no_hit)
{
    // Sphere at X=10 — outside the 3×3 grid footprint [0..2] — should not hit.
    test::ScopedHeightmap hd(test::make_flat_heightmap_3x3());
    rbc::Shape sA = rbc::Sphere(0.5), hmB = rbc::Heightmap(hd);
    auto tfA = test::tf_at(10, 0.3, 1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Heightmap>(sA, tfA, hmB, tfB, c)));
}

TEST_SUITE(
    RUN_TEST(sphere_heightmap_flat_above_no_hit),
    RUN_TEST(sphere_heightmap_flat_penetrating),
    RUN_TEST(sphere_heightmap_elevated_flat_above_no_hit),
    RUN_TEST(sphere_heightmap_elevated_flat_penetrating),
    RUN_TEST(sphere_heightmap_outside_grid_no_hit),
)
