#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/ConvexHull.hpp"

// --- Existing Tests ---
TEST(epa_sphere_penetration)
{
    rbc::Shape sA = rbc::Sphere(1.0); 
    rbc::Shape sB = rbc::Sphere(1.0); 
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.5, 0, 0); 

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, tfB.pos - tfA.pos);
    
    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    ASSERT_TRUE(status == rbc::EPA::Valid);
    ASSERT_NEAR(epa.depth, 0.5, 0.05);
    ASSERT_NEAR(std::abs(epa.normal.x), 1.0, 0.05);
}

TEST(epa_box_sphere_penetration)
{
    rbc::Shape boxA = rbc::Box(m3d::vec3(1, 1, 1)); 
    rbc::Shape sphereB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 1.7, 0); 

    rbc::MinkowskiDiff md(&boxA, &sphereB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, tfB.pos - tfA.pos);

    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    ASSERT_TRUE(status == rbc::EPA::Valid);
    ASSERT_NEAR(epa.depth, 0.3, 0.05);
    ASSERT_NEAR(std::abs(epa.normal.y), 1.0, 0.05); 
}

TEST(epa_box_box_face)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)); 
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1)); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.6, 0, 0); // Overlap of 0.4 on X axis

    rbc::MinkowskiDiff md(&bA, &bB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, tfB.pos - tfA.pos);

    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    ASSERT_TRUE(status == rbc::EPA::Valid);
    ASSERT_NEAR(epa.depth, 0.4, 0.01);
    ASSERT_NEAR(std::abs(epa.normal.x), 1.0, 0.01);
}

TEST(epa_box_box_corner_offset)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1)); 
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1)); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    // B is offset heavily on Y and Z, but only slightly on X.
    // The *shortest* way out of the collision is along the Z axis!
    // Overlaps: X = 2.0 - 0.5 = 1.5. Y = 2.0 - 0.5 = 1.5. Z = 2.0 - 1.8 = 0.2.
    m3d::tf tfB; tfB.pos = m3d::vec3(0.5, 0.5, 1.8); 

    rbc::MinkowskiDiff md(&bA, &bB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, tfB.pos - tfA.pos);

    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    ASSERT_TRUE(status == rbc::EPA::Valid);
    ASSERT_NEAR(epa.depth, 0.2, 0.01);         // Smallest overlap is 0.2
    ASSERT_NEAR(std::abs(epa.normal.z), 1.0, 0.01); // Normal must be Z axis
}

TEST(epa_diagonal_spheres)
{
    rbc::Shape sA = rbc::Sphere(1.0); 
    rbc::Shape sB = rbc::Sphere(1.0); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    // Sphere B is offset diagonally.
    // Distance = sqrt(1^2 + 1^2 + 1^2) = sqrt(3) ≈ 1.732
    m3d::tf tfB; tfB.pos = m3d::vec3(1.0, 1.0, 1.0); 

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, tfB.pos - tfA.pos);

    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    ASSERT_TRUE(status == rbc::EPA::Valid);
    
    m3d::scalar expected_dist = m3d::sqrt(3.0);
    m3d::scalar expected_depth = 2.0 - expected_dist; // 2.0 combined radii
    
    ASSERT_NEAR(epa.depth, expected_depth, 0.05);

    // Normal should be parallel to the vector between centers (1,1,1)
    m3d::vec3 expected_normal = m3d::normalize(tfB.pos - tfA.pos);
    ASSERT_NEAR(std::abs(epa.normal.x), std::abs(expected_normal.x), 0.05);
    ASSERT_NEAR(std::abs(epa.normal.y), std::abs(expected_normal.y), 0.05);
    ASSERT_NEAR(std::abs(epa.normal.z), std::abs(expected_normal.z), 0.05);
}

// Regression: vertex-on-face tangent contact between Box and Tetrahedron.
// GJK terminates with rank == 3 (origin lies on a triangle of the Minkowski
// difference) and EPA used to segfault dereferencing a NULL adjacency in
// expand(). After the fix EPA must promote the rank-3 simplex into a
// tetrahedron and either return Valid or Failed cleanly — never crash.
TEST(epa_rank3_box_tet_tangent)
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
    m3d::tf tfB; tfB.pos = m3d::vec3(0.0, 1.1, 0.0);

    rbc::MinkowskiDiff md(&sa, &sb, tfA, tfB);
    rbc::GJK gjk;
    rbc::GJK::Status gjk_status = gjk.evaluate(md, tfB.pos - tfA.pos);

    // GJK should detect the touching contact (rank may be 3 or 4).
    ASSERT_TRUE(gjk_status == rbc::GJK::Inside);

    // The crucial assertion: EPA must return — no SEGV.
    // Either Valid (with depth ≈ 0) or Failed (tangent, no volume) is acceptable.
    rbc::EPA epa;
    rbc::EPA::Status epa_status = epa.evaluate(gjk, md);
    ASSERT_TRUE(epa_status == rbc::EPA::Valid ||
                epa_status == rbc::EPA::Failed);

    rbc::convex_hull_data_destroy(hd);
}

// Regression: GJK delivers a rank-4 simplex with duplicate vertices for
// some Box-vs-Tetrahedron rotated overlaps. project_tetrahedron_origin
// reports "origin enclosed" without checking that the four vertices are
// distinct, and EPA's rank-4 face-build then produces a zero-area cross
// product, returns null from new_face(), and aborts with Failed.
//
// The fix is to dedup the simplex in EPA::evaluate: 4 → 3 unique then
// promote via the rank-3 path. After the fix EPA must converge with a
// real penetration depth and an upward-pointing normal (tetrahedron
// sitting on top of the box).
TEST(epa_rank4_dup_vertex_box_tet_rotated)
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
    tfB.pos = m3d::vec3(0.0, 0.91, 0.0);
    tfB.rot = m3d::quat(0.87515037, 0.2995468562, -0.3563325264, -0.1319493896);

    rbc::MinkowskiDiff md(&sa, &sb, tfA, tfB);
    rbc::GJK gjk;
    ASSERT_TRUE(gjk.evaluate(md, tfB.pos - tfA.pos) == rbc::GJK::Inside);

    rbc::EPA epa;
    ASSERT_TRUE(epa.evaluate(gjk, md) == rbc::EPA::Valid);

    // Tetrahedron is above the box → normal points roughly +Y.
    ASSERT_NEAR(epa.normal.y, 1.0, 0.05);
    ASSERT_NORMAL_UNIT(epa.normal);
    // Some real penetration was recovered (not a tangent).
    ASSERT_TRUE(epa.depth > 0.01);

    rbc::convex_hull_data_destroy(hd);
}

TEST_SUITE(
    RUN_TEST(epa_sphere_penetration),
    RUN_TEST(epa_box_sphere_penetration),
    RUN_TEST(epa_box_box_face),
    RUN_TEST(epa_box_box_corner_offset),
    RUN_TEST(epa_diagonal_spheres),
    RUN_TEST(epa_rank3_box_tet_tangent),
    RUN_TEST(epa_rank4_dup_vertex_box_tet_rotated)
)