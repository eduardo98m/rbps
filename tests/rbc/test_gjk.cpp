#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// --- Existing Tests ---
TEST(spheres_separated)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(3.0, 0, 0);

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_FALSE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

TEST(spheres_overlapping)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(1.5, 0, 0);

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_TRUE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

TEST(box_sphere_overlapping)
{
    rbc::Shape boxA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape sphereB = rbc::Sphere(1.0);
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(1.8, 0, 0);

    rbc::MinkowskiDiff md(&boxA, &sphereB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_TRUE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

// --- New Tests ---

TEST(box_box_separated)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));

    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(2.5, 0, 0); // Separated by 0.5

    rbc::MinkowskiDiff md(&bA, &bB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_FALSE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

TEST(box_box_overlapping)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));

    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(1.5, 0.5, 0.5); // Overlapping on all axes

    rbc::MinkowskiDiff md(&bA, &bB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_TRUE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

TEST(diagonal_separated)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape sB = rbc::Sphere(1.0);

    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    // Box corner is at (1,1,1). Sphere is at (2.5, 2.5, 2.5) -> definitely separated.
    m3d::tf tfB;
    tfB.pos = m3d::vec3(2.5, 2.5, 2.5);

    rbc::MinkowskiDiff md(&bA, &sB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_FALSE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

TEST(deep_penetration)
{
    // One shape entirely inside another
    rbc::Shape bA = rbc::Box(m3d::vec3(5, 5, 5)); // Giant box
    rbc::Shape sB = rbc::Sphere(1.0);             // Small sphere

    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0.1, 0.2, -0.1); // Slightly offset from center

    rbc::MinkowskiDiff md(&bA, &sB, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_TRUE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);
}

// Regression: project_tetrahedron_origin used to call project_triangle_origin
// with the unmodified rank-4 simplex, which made the triangle code see the
// "back face" (B, C, D) regardless of which face had been chosen — silently
// dropping the most-recently-added support and producing false positives for
// clearly separated pairs.
//
// Box (0.5)^3 vs Tetrahedron (verts ±0.6) at (0, 1.17, 0) rotated by
// RPY(47.5°, -33°, -32°) is one such case: the Minkowski difference's max-Y
// extent is ≈ -0.19 (origin sits 0.19 m above MD). After the fix GJK must
// report Valid (separated) with distance ≈ 0.19.
TEST(gjk_box_tet_rotated_separated)
{
    static const m3d::vec3 tet_verts[4] = {
        m3d::vec3( 0.6,  0.6,  0.6),
        m3d::vec3(-0.6, -0.6,  0.6),
        m3d::vec3(-0.6,  0.6, -0.6),
        m3d::vec3( 0.6, -0.6, -0.6),
    };
    static const uint32_t tet_faces[12] = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    rbc::ConvexHullData *hd = rbc::convex_hull_data_create(tet_verts, 4, tet_faces, 4);

    rbc::Shape sa = rbc::Box(m3d::vec3(0.5, 0.5, 0.5));
    rbc::Shape sb = rbc::ConvexHull(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(0.0, 0.0, 0.0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0.0, 1.17, 0.0);
    const double d2r = 0.017453292519943295;
    tfB.rot = m3d::quat::from_rpy(47.5 * d2r, -33.0 * d2r, -32.0 * d2r);

    rbc::MinkowskiDiff md(&sa, &sb, tfA, tfB);
    rbc::GJK gjk;
    rbc::GJK::Status s = gjk.evaluate(md, tfB.pos - tfA.pos);

    // Must NOT report Inside — the shapes do not actually overlap.
    ASSERT_TRUE(s == rbc::GJK::Valid);
    // Closest-approach distance is roughly the MD's max-Y extent (~0.19).
    ASSERT_NEAR(gjk.distance, 0.19, 0.05);

    rbc::convex_hull_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(spheres_separated),
    RUN_TEST(spheres_overlapping),
    RUN_TEST(box_sphere_overlapping),
    RUN_TEST(box_box_separated),
    RUN_TEST(box_box_overlapping),
    RUN_TEST(diagonal_separated),
    RUN_TEST(deep_penetration),
    RUN_TEST(gjk_box_tet_rotated_separated))