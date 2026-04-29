#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// =============================================================================
// CONVEX HULL vs SPHERE
// =============================================================================
//
// Tetrahedron (vertex set) vs sphere. Confirms that ConvexHull rides the
// generic GJK + EPA pipeline (no analytic specialisation) and produces
// reasonable contact data on penetration / separation.

namespace
{
    // Regular-ish tetrahedron centred on origin, edge length ≈ 2.
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

    rbc::ConvexHullData *make_tet()
    {
        return rbc::convex_hull_data_create(k_tet_verts, 4, k_tet_faces, 4);
    }
}

TEST(convex_hull_sphere_separated)
{
    auto *hd = make_tet();
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape sB = rbc::Sphere(0.5);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(5.0, 0, 0); // far away

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::ConvexHull, rbc::Sphere>(hA, tfA, sB, tfB, c)));
    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_sphere_face_overlap)
{
    auto *hd = make_tet();
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape sB = rbc::Sphere(0.5);
    // Tetrahedron vertex (1,1,1) is the extreme along (+x,+y,+z).
    // Sphere centre at (1.2,1.2,1.2): distance to that vertex is √0.12 ≈ 0.346,
    // well inside radius 0.5 → clear penetration along the diagonal.
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(1.2, 1.2, 1.2);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Sphere>(hA, tfA, sB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(hA, tfA, sB, tfB, reference));

    ASSERT_TRUE(test::depth(analytic) > 0.0);
    ASSERT_NORMAL_UNIT(analytic.normal);
    // Both go through the same GJK fallback, so depths should match closely.
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_sphere_deep_penetration)
{
    auto *hd = make_tet();
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape sB = rbc::Sphere(0.3);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0.0, 0.2, 0.0); // sphere centred near origin (inside the hull)

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Sphere>(hA, tfA, sB, tfB, c)));
    ASSERT_TRUE(test::depth(c) > 0.0);
    ASSERT_NORMAL_UNIT(c.normal);
    rbc::convex_hull_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(convex_hull_sphere_separated),
    RUN_TEST(convex_hull_sphere_face_overlap),
    RUN_TEST(convex_hull_sphere_deep_penetration))
