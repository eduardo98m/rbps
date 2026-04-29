#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// =============================================================================
// CONVEX HULL vs PLANE
// =============================================================================
//
// ConvexHull–Plane is wired through the `RBC_PLANE_SPEC(ConvexHull)`
// specialisation, which produces a single-point manifold via
// `detail::convex_vs_plane`. We verify:
// 1. No contact when the hull is above the plane.
// 2. A single-point contact at the deepest vertex when penetrating.
// 3. The contact normal points away from the plane (matching convention
//    used by Sphere/Ellipsoid/Cone-vs-Plane).

namespace
{
    static const m3d::vec3 k_tet_verts[4] = {
        m3d::vec3( 1.0,  1.0,  1.0),
        m3d::vec3(-1.0, -1.0,  1.0),
        m3d::vec3(-1.0,  1.0, -1.0),
        m3d::vec3( 1.0, -1.0, -1.0),
    };
    static const uint32_t k_tet_faces[4 * 3] = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
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
    // Single-point manifold (matches convex_vs_plane convention).
    ASSERT_EQ(c.num_points, 1);
    ASSERT_TRUE(test::depth(c) > 0.0);
    ASSERT_NORMAL_UNIT(c.normal);
    // Normal should point AWAY from the plane normal (per detail::convex_vs_plane,
    // out.normal = -world_n, so for plane n=(0,1,0) we expect normal.y ≈ -1).
    ASSERT_NEAR(c.normal.y, -1.0, 0.01);
    rbc::convex_hull_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(convex_hull_plane_separated),
    RUN_TEST(convex_hull_plane_penetrating))
