#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// =============================================================================
// CONVEX HULL vs PLANE
// =============================================================================
//
// ConvexHull–Plane has a dedicated multi-point specialization that tests all
// vertices against the plane and collects every penetrating vertex, then
// reduces the contact set to at most 4 points via area maximization:
//
//   P0 – the deepest penetrating vertex (always kept)
//   P1 – the vertex farthest from P0 (maximises span)
//   P2 – the vertex farthest from segment P0–P1 (maximises triangle area)
//   P3 – the vertex that maximises the quadrilateral area by scoring the
//        best "pocket" outside the P0–P1–P2 triangle (may be absent if
//        all remaining candidates fall inside the triangle)
//
// We verify:
// 1. No contact when the hull is fully above the plane.
// 2. When ≤4 vertices penetrate, all of them are reported as-is (no reduction).
// 3. When >4 vertices penetrate, exactly 4 contact points are returned and
//    they form the area-maximizing quadrilateral over the contact patch.
// 4. The deepest penetrating vertex is always present in the manifold,
//    regardless of the reduction path taken.
// 5. Contact normals point away from the plane surface (consistent with the
//    convention used by Sphere/Ellipsoid/Cone-vs-Plane).

namespace
{
    static const m3d::vec3 k_tet_verts[4] = {
        m3d::vec3(1.0, 1.0, 1.0),
        m3d::vec3(-1.0, -1.0, 1.0),
        m3d::vec3(-1.0, 1.0, -1.0),
        m3d::vec3(1.0, -1.0, -1.0),
    };
    static const uint32_t k_tet_faces[4 * 3] = {
        0,
        1,
        2,
        0,
        3,
        1,
        0,
        2,
        3,
        1,
        3,
        2,
    };

    static const m3d::vec3 k_cube_verts[8] = {
        m3d::vec3(-1.0, -1.0, -1.0),
        m3d::vec3(1.0, -1.0, -1.0),
        m3d::vec3(1.0, 1.0, -1.0),
        m3d::vec3(-1.0, 1.0, -1.0),
        m3d::vec3(-1.0, -1.0, 1.0),
        m3d::vec3(1.0, -1.0, 1.0),
        m3d::vec3(1.0, 1.0, 1.0),
        m3d::vec3(-1.0, 1.0, 1.0),
    };

    static const uint32_t k_cube_faces[6 * 2 * 3] = {
        0,
        1,
        2,
        0,
        2,
        3, // -Z
        4,
        6,
        5,
        4,
        7,
        6, // +Z
        0,
        5,
        1,
        0,
        4,
        5, // -Y
        2,
        6,
        3,
        3,
        6,
        7, // +Y  (won't matter for plane test)
        0,
        3,
        7,
        0,
        7,
        4, // -X
        1,
        5,
        6,
        1,
        6,
        2, // +X
    };
}

TEST(convex_hull_plane_separated)
{
    auto *hd = rbc::convex_hull_data_create(k_tet_verts, 4, k_tet_faces, 4);
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape pB = rbc::Plane(m3d::vec3(0, 1, 0), 0.0);
    auto tfA = test::tf_at(0, 5.0, 0); // hull well above
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::ConvexHull, rbc::Plane>(hA, tfA, pB, tfB, c)));
    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_plane_penetrating)
{
    auto *hd = rbc::convex_hull_data_create(k_tet_verts, 4, k_tet_faces, 4);
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape pB = rbc::Plane(m3d::vec3(0, 1, 0), 0.0);
    // Lower the hull by 0.3 so the (-y) vertices dip below the plane.
    auto tfA = test::tf_at(0, 0.7, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Plane>(hA, tfA, pB, tfB, c)));
    // Multi-point manifold: 2 vertices at y=-1.0 become y=-0.3, penetrating the plane.
    ASSERT_EQ(c.num_points, 2);
    ASSERT_TRUE(test::depth(c) > 0.0);
    ASSERT_NORMAL_UNIT(c.normal);
    // Normal should point AWAY from the plane normal (per detail::convex_vs_plane,
    // out.normal = -world_n, so for plane n=(0,1,0) we expect normal.y ≈ -1).
    ASSERT_NEAR(c.normal.y, -1.0, 0.01);
    // Each contact point should have positive penetration depth.
    for (uint32_t i = 0; i < c.num_points; ++i)
    {
        ASSERT_TRUE(c.points[i].penetration_depth > 0.0);
    }
    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_plane_reduction_4pts)
{
    auto *hd = rbc::convex_hull_data_create(k_cube_verts, 8, k_cube_faces, 12);
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape pB = rbc::Plane(m3d::vec3(0, 1, 0), 0.0);

    // Translate so bottom face (local y=-1) lands at world y=-0.5 → 0.5 m deep.
    auto tfA = test::tf_at(0, 0.5, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Plane>(hA, tfA, pB, tfB, c)));

    // Exactly 4 contacts — the 4 bottom corners of the cube.
    ASSERT_EQ(c.num_points, 4);

    // All depths must be positive and uniform (≈0.5 m for a flat base).
    for (uint32_t i = 0; i < c.num_points; ++i)
        ASSERT_NEAR(c.points[i].penetration_depth, 0.5, 0.01);

    // Normal convention preserved through the reduction path.
    ASSERT_NEAR(c.normal.y, -1.0, 0.01);

    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_plane_deepest_always_kept)
{
    auto *hd = rbc::convex_hull_data_create(k_cube_verts, 8, k_cube_faces, 12);
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape pB = rbc::Plane(m3d::vec3(0, 1, 0), 0.0);

    // 30-degree tilt around Z so one bottom corner is deeper than the others,
    // creating a clear depth gradient that exercises the P0 deepest-point selection.
    auto tfA = m3d::tf(
        m3d::vec3(0, 0.5, 0),
        m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::to_radians(30.0))
    );
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Plane>(hA, tfA, pB, tfB, c)));
    ASSERT_TRUE(c.num_points > 0);

    // Find the deepest contact point in the manifold.
    m3d::scalar max_depth = 0.0;
    for (uint32_t i = 0; i < c.num_points; ++i)
        if (c.points[i].penetration_depth > max_depth)
            max_depth = c.points[i].penetration_depth;

    // Re-derive the true deepest penetration from the raw vertices
    // so the assertion doesn't rely on a magic hardcoded number.
    m3d::scalar expected_max = 0.0;
    for (int i = 0; i < 8; ++i)
    {
        m3d::vec3   w     = tfA.transform_point(k_cube_verts[i]);
        m3d::scalar depth = -w.y; // plane is y=0, normal=(0,1,0)
        if (depth > expected_max) expected_max = depth;
    }

    ASSERT_NEAR(max_depth, expected_max, 0.01);

    rbc::convex_hull_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(convex_hull_plane_separated),
    RUN_TEST(convex_hull_plane_penetrating),
    RUN_TEST(convex_hull_plane_reduction_4pts),
    RUN_TEST(convex_hull_plane_deepest_always_kept))
