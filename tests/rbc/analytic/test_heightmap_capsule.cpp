#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Heightmap.hpp"

static rbc::HeightmapData *make_flat_heightmap(float height = 0.0f)
{
    static float h[9];
    for (int i = 0; i < 9; ++i) h[i] = height;
    return rbc::heightmap_data_create(h, 3, 3,
        m3d::vec3(1.0, 1.0, 1.0),
        m3d::vec3(0.0, 0.0, 0.0));
}

TEST(capsule_heightmap_upright_above_no_hit)
{
    // Centre Y=2: bottom cap at Y=0.5 → above the Y=0 flat map.
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape cA  = rbc::Capsule(1.0, 0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 2, 1);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Heightmap>::test(
        cA.get<rbc::Capsule>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);
    ASSERT_FALSE(hit);

    rbc::heightmap_data_destroy(hd);
}

TEST(capsule_heightmap_upright_penetrating)
{
    // Centre at Y=1.0. Bottom endcap sphere at Y=0, radius=0.5 → pen=0.5.
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape cA  = rbc::Capsule(1.0, 0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 1.0, 1);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Heightmap>::test(
        cA.get<rbc::Capsule>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 0.5, 0.05);
    ASSERT_NEAR(c.normal.y, 1.0, 0.05);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);

    rbc::heightmap_data_destroy(hd);
}

TEST(capsule_heightmap_horizontal_penetrating)
{
    // Capsule lying along X, centre at (1, 0.3, 1). Radius=0.5 → pen=0.2.
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape cA  = rbc::Capsule(0.5, 0.5); // shorter so both endpoints are over the grid
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA;
    tfA.pos = m3d::vec3(1, 0.3, 1);
    tfA.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 2.0);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Heightmap>::test(
        cA.get<rbc::Capsule>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 0.2, 0.03);
    ASSERT_NEAR(c.normal.y, 1.0, 0.05);

    rbc::heightmap_data_destroy(hd);
}

TEST(capsule_heightmap_outside_grid_no_hit)
{
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape cA  = rbc::Capsule(1.0, 0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(10, 0.3, 1); // outside XZ footprint
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Heightmap>::test(
        cA.get<rbc::Capsule>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);
    ASSERT_FALSE(hit);

    rbc::heightmap_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(capsule_heightmap_upright_above_no_hit),
    RUN_TEST(capsule_heightmap_upright_penetrating),
    RUN_TEST(capsule_heightmap_horizontal_penetrating),
    RUN_TEST(capsule_heightmap_outside_grid_no_hit),
)