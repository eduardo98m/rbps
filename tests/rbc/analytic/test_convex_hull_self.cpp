#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// =============================================================================
// CONVEX HULL vs CONVEX HULL
// =============================================================================
//
// Two tetrahedra. Confirms that the (ConvexHull, ConvexHull) cell of the
// dispatcher routes through the generic GJK + EPA + manifold pipeline and
// produces the expected separation / overlap behaviour.

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

    rbc::ConvexHullData *make_tet()
    {
        return rbc::convex_hull_data_create(k_tet_verts, 4, k_tet_faces, 4);
    }
}

TEST(convex_hull_self_separated)
{
    auto *a = make_tet();
    auto *b = make_tet();
    rbc::Shape hA = rbc::ConvexHull(a);
    rbc::Shape hB = rbc::ConvexHull(b);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(5.0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::ConvexHull, rbc::ConvexHull>(hA, tfA, hB, tfB, c)));
    rbc::convex_hull_data_destroy(a);
    rbc::convex_hull_data_destroy(b);
}

TEST(convex_hull_self_overlap)
{
    auto *a = make_tet();
    auto *b = make_tet();
    rbc::Shape hA = rbc::ConvexHull(a);
    rbc::Shape hB = rbc::ConvexHull(b);
    auto tfA = test::tf_at(0, 0, 0);
    // Translate B so its (1,1,1) vertex pokes into A.
    auto tfB = test::tf_at(1.0, 1.0, 1.0);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::ConvexHull>(hA, tfA, hB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(hA, tfA, hB, tfB, reference));

    ASSERT_TRUE(test::depth(analytic) > 0.0);
    ASSERT_NORMAL_UNIT(analytic.normal);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    rbc::convex_hull_data_destroy(a);
    rbc::convex_hull_data_destroy(b);
}

TEST(convex_hull_self_deep_penetration)
{
    auto *a = make_tet();
    auto *b = make_tet();
    rbc::Shape hA = rbc::ConvexHull(a);
    rbc::Shape hB = rbc::ConvexHull(b);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0.1, 0.0, 0.0); // nearly co-located

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::ConvexHull>(hA, tfA, hB, tfB, c)));
    ASSERT_TRUE(test::depth(c) > 0.0);
    ASSERT_NORMAL_UNIT(c.normal);
    rbc::convex_hull_data_destroy(a);
    rbc::convex_hull_data_destroy(b);
}

TEST_SUITE(
    RUN_TEST(convex_hull_self_separated),
    RUN_TEST(convex_hull_self_overlap),
    RUN_TEST(convex_hull_self_deep_penetration))
