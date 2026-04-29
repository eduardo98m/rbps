#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// =============================================================================
// CONVEX HULL vs BOX
// =============================================================================
//
// Encode a unit cube as a ConvexHull (8 verts, 12 triangles) and pit it
// against a real `rbc::Box`. This sanity-checks that the ConvexHull GJK
// path agrees with results that the SAT-based Box-Box path is known to
// produce. Depths should match within GJK/EPA tolerance.

namespace
{
    // Unit cube, half-extent 1.0, CCW-wound triangles (outward normals).
    static const m3d::vec3 k_cube_verts[8] = {
        m3d::vec3(-1, -1, -1),
        m3d::vec3( 1, -1, -1),
        m3d::vec3( 1,  1, -1),
        m3d::vec3(-1,  1, -1),
        m3d::vec3(-1, -1,  1),
        m3d::vec3( 1, -1,  1),
        m3d::vec3( 1,  1,  1),
        m3d::vec3(-1,  1,  1),
    };
    static const uint32_t k_cube_faces[12 * 3] = {
        // -Z face
        0, 2, 1,  0, 3, 2,
        // +Z face
        4, 5, 6,  4, 6, 7,
        // -Y face
        0, 1, 5,  0, 5, 4,
        // +Y face
        2, 3, 7,  2, 7, 6,
        // -X face
        0, 4, 7,  0, 7, 3,
        // +X face
        1, 2, 6,  1, 6, 5,
    };

    rbc::ConvexHullData *make_cube_hull()
    {
        return rbc::convex_hull_data_create(k_cube_verts, 8, k_cube_faces, 12);
    }
}

TEST(convex_hull_box_separated)
{
    auto *hd = make_cube_hull();
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(2.5, 0, 0); // gap of 0.5

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::ConvexHull, rbc::Box>(hA, tfA, bB, tfB, c)));
    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_box_face_overlap)
{
    auto *hd = make_cube_hull();
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(1.6, 0, 0); // overlap = 0.4

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Box>(hA, tfA, bB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(hA, tfA, bB, tfB, reference));

    ASSERT_TRUE(test::depth(analytic) > 0.0);
    ASSERT_NORMAL_UNIT(analytic.normal);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    rbc::convex_hull_data_destroy(hd);
}

TEST(convex_hull_box_deep_penetration)
{
    auto *hd = make_cube_hull();
    rbc::Shape hA = rbc::ConvexHull(hd);
    rbc::Shape bB = rbc::Box(m3d::vec3(0.5, 0.5, 0.5));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0.2, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::ConvexHull, rbc::Box>(hA, tfA, bB, tfB, c)));
    ASSERT_TRUE(test::depth(c) > 0.0);
    ASSERT_NORMAL_UNIT(c.normal);
    rbc::convex_hull_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(convex_hull_box_separated),
    RUN_TEST(convex_hull_box_face_overlap),
    RUN_TEST(convex_hull_box_deep_penetration))
