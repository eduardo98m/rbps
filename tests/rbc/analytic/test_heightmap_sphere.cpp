#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Heightmap.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────
// 3×3 flat heightmap at height 0. Cell size 1×1 in XZ.
// Spans X=[0,2], Z=[0,2].
static rbc::HeightmapData *make_flat_heightmap(float height = 0.0f)
{
    static float h[9]; // 3×3
    for (int i = 0; i < 9; ++i) h[i] = height;
    return rbc::heightmap_data_create(h, 3, 3,
        m3d::vec3(1.0, 1.0, 1.0), // scale: cell=1, height_scale=1
        m3d::vec3(0.0, 0.0, 0.0)  // origin
    );
}

// 3×3 heightmap with a hill at the centre (index [1,1] = 1.0, rest = 0).
static rbc::HeightmapData *make_hill_heightmap()
{
    static float h[9] = {
        0, 0, 0,
        0, 1, 0,
        0, 0, 0,
    };
    return rbc::heightmap_data_create(h, 3, 3,
        m3d::vec3(1.0, 1.0, 1.0),
        m3d::vec3(0.0, 0.0, 0.0));
}

TEST(sphere_heightmap_flat_above_no_hit)
{
    // Sphere radius=0.5, centre at (1, 1.0, 1). Lowest at Y=0.5 → above flat hmap.
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape sA = rbc::Sphere(0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 1.0, 1);
    m3d::tf tfB; // heightmap ignores tf

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Heightmap>::test(
        sA.get<rbc::Sphere>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);
    ASSERT_FALSE(hit);

    rbc::heightmap_data_destroy(hd);
}

TEST(sphere_heightmap_flat_penetrating)
{
    // Sphere radius=0.5, centre at (1, 0.3, 1). Pen = 0.5 - 0.3 = 0.2.
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape sA  = rbc::Sphere(0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 0.3, 1);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Heightmap>::test(
        sA.get<rbc::Sphere>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 0.2, 0.02);
    ASSERT_NEAR(c.normal.y, 1.0, 0.05);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);

    rbc::heightmap_data_destroy(hd);
}

TEST(sphere_heightmap_elevated_flat_above_no_hit)
{
    // Heightmap at height=2. Sphere centre at (1, 3.0, 1). Lowest at Y=2.5 > 2.
    rbc::HeightmapData *hd = make_flat_heightmap(2.0f);
    rbc::Shape sA  = rbc::Sphere(0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 3.0, 1);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Heightmap>::test(
        sA.get<rbc::Sphere>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);
    ASSERT_FALSE(hit);

    rbc::heightmap_data_destroy(hd);
}

TEST(sphere_heightmap_elevated_flat_penetrating)
{
    // Heightmap at height=2. Sphere radius=0.5, centre at (1, 2.3, 1). Pen=0.2.
    rbc::HeightmapData *hd = make_flat_heightmap(2.0f);
    rbc::Shape sA  = rbc::Sphere(0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 2.3, 1);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Heightmap>::test(
        sA.get<rbc::Sphere>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 0.2, 0.02);

    rbc::heightmap_data_destroy(hd);
}

TEST(sphere_heightmap_outside_grid_no_hit)
{
    // Sphere at X=10 — outside the 3×3 grid footprint [0..2] — should not hit.
    rbc::HeightmapData *hd = make_flat_heightmap();
    rbc::Shape sA  = rbc::Sphere(0.5);
    rbc::Shape hmB = rbc::Heightmap(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(10, 0.3, 1);
    m3d::tf tfB;

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Heightmap>::test(
        sA.get<rbc::Sphere>(), tfA, hmB.get<rbc::Heightmap>(), tfB, c);
    ASSERT_FALSE(hit);

    rbc::heightmap_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(sphere_heightmap_flat_above_no_hit),
    RUN_TEST(sphere_heightmap_flat_penetrating),
    RUN_TEST(sphere_heightmap_elevated_flat_above_no_hit),
    RUN_TEST(sphere_heightmap_elevated_flat_penetrating),
    RUN_TEST(sphere_heightmap_outside_grid_no_hit),
)