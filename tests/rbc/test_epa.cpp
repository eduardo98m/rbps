#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/gjk/ContactManifoldGenerator.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/ConvexHull.hpp"
#include <cmath>

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

// ──────────────────────────────────────────────────────────────────────
//  Manifold face-selection regressions
// ──────────────────────────────────────────────────────────────────────

namespace {
    // Hex-prism (12 verts, 20 triangles), CCW-from-outside winding so
    // convex_hull_data_create produces outward face normals.
    inline rbc::ConvexHullData *make_hex_prism()
    {
        static m3d::vec3 verts[12];
        static uint32_t  faces[20 * 3];
        static bool initialized = false;
        if (!initialized)
        {
            for (int i = 0; i < 6; ++i)
            {
                const float angle = i * (6.283185307f / 6.0f);
                const float x = std::cos(angle) * 0.8f;
                const float z = std::sin(angle) * 0.8f;
                verts[i]     = m3d::vec3(x,  0.5f, z);
                verts[i + 6] = m3d::vec3(x, -0.5f, z);
            }
            int f = 0;
            // Top cap (+Y outward)
            for (int i = 1; i < 5; ++i)
            { faces[f++] = 0; faces[f++] = i + 1; faces[f++] = i; }
            // Bottom cap (-Y outward)
            for (int i = 1; i < 5; ++i)
            { faces[f++] = 6; faces[f++] = i + 6; faces[f++] = i + 7; }
            // Sides
            for (int i = 0; i < 6; ++i)
            {
                const int next = (i + 1) % 6;
                faces[f++] = i;    faces[f++] = next;     faces[f++] = i + 6;
                faces[f++] = next; faces[f++] = next + 6; faces[f++] = i + 6;
            }
            initialized = true;
        }
        return rbc::convex_hull_data_create(verts, 12, faces, 20);
    }
}

// Two hex-prisms stacked with a small overlap. Before fixing the preset
// winding, face_corners picked the BOTTOM cap of A (because all stored
// face normals were inward); after the fix it picks the top cap and
// every manifold point lies on Y ≈ 0.5 (A's top face plane).
TEST(manifold_hex_prism_stack_face_face)
{
    rbc::ConvexHullData *hd = make_hex_prism();
    rbc::Shape sa = rbc::ConvexHull(hd);
    rbc::Shape sb = rbc::ConvexHull(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(0.0, 0.0, 0.0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0.0, 0.95, 0.0);

    rbc::ContactManifold mfd;
    ASSERT_TRUE(rbc::gjk_epa_manifold(sa, tfA, sb, tfB, mfd));
    ASSERT_TRUE(mfd.num_points >= 1);
    // Normal points up (A→B convention)
    ASSERT_NEAR(mfd.normal.y, 1.0, 0.05);
    ASSERT_NORMAL_UNIT(mfd.normal);
    // Every manifold point must lie in the contact zone between A's top
    // (y=0.5) and B's bottom (y=0.45). Pre-fix the points sat near
    // y = -0.5 (the wrong, opposite-side cap of A).
    for (int i = 0; i < mfd.num_points; ++i)
    {
        ASSERT_TRUE(mfd.points[i].position.y >= 0.40);
        ASSERT_TRUE(mfd.points[i].position.y <= 0.55);
    }

    rbc::convex_hull_data_destroy(hd);
}

// Tetrahedron (verts ±0.6 cube-corners) suspended above a Box (half=1)
// so that one tet vertex penetrates the box's top face. No tet face is
// parallel to the contact normal — the alignment check should reject the
// face-face path and emit a single-point manifold at the EPA contact.
TEST(manifold_tet_on_box_vertex_contact)
{
    static const m3d::vec3 tet_verts[4] = {
        m3d::vec3( 0.6,  0.6,  0.6),
        m3d::vec3(-0.6, -0.6,  0.6),
        m3d::vec3(-0.6,  0.6, -0.6),
        m3d::vec3( 0.6, -0.6, -0.6),
    };
    static const uint32_t tet_faces[12] = {
        0, 2, 1,
        0, 1, 3,
        0, 3, 2,
        1, 2, 3,
    };
    rbc::ConvexHullData *hd = rbc::convex_hull_data_create(tet_verts, 4, tet_faces, 4);

    rbc::Shape sa = rbc::Box(m3d::vec3(1.0, 1.0, 1.0));
    rbc::Shape sb = rbc::ConvexHull(hd);
    m3d::tf tfA; tfA.pos = m3d::vec3(0.0, 0.0, 0.0);
    // Tet at (0, 1.5, 0) — its lowest vertex is at y = 1.5 - 0.6 = 0.9,
    // penetrating the box's top face (y = 1.0) by 0.1.
    m3d::tf tfB; tfB.pos = m3d::vec3(0.0, 1.5, 0.0);

    rbc::ContactManifold mfd;
    ASSERT_TRUE(rbc::gjk_epa_manifold(sa, tfA, sb, tfB, mfd));
    // Edge / vertex contact → single-point manifold, not a phantom-laced
    // face-face clip.
    ASSERT_TRUE(mfd.num_points == 1);
    // The single point sits roughly at the box's top face (y ≈ 1.0).
    ASSERT_NEAR(mfd.points[0].position.y, 1.0, 0.15);

    rbc::convex_hull_data_destroy(hd);
}

// Box rotated 45° around Z, lowered onto another box so its bottom
// "diamond" edge contacts box A's top face along a single line. Box A
// has a parallel face (+Y top) but box B's chosen face sits at 45°,
// so the alignment check rejects the face-face path.
TEST(manifold_box_box_edge_contact)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1.0, 1.0, 1.0));
    rbc::Shape bB = rbc::Box(m3d::vec3(1.0, 1.0, 1.0));
    m3d::tf tfA; tfA.pos = m3d::vec3(0.0, 0.0, 0.0);
    m3d::tf tfB;
    // Lowest world-Y of a 45°-Z-rotated unit box is at y = -sqrt(2) ≈ -1.414;
    // place B's centre so that lowest point sits 0.1 below A's top face.
    tfB.pos = m3d::vec3(0.0, 1.0 + 1.414 - 0.1, 0.0);
    const double d2r = 0.017453292519943295;
    tfB.rot = m3d::quat::from_rpy(0.0, 0.0, 45.0 * d2r);

    rbc::ContactManifold mfd;
    ASSERT_TRUE(rbc::gjk_epa_manifold(bA, tfA, bB, tfB, mfd));
    // Edge contact → single-point fallback (one shape lacks a parallel face).
    ASSERT_TRUE(mfd.num_points == 1);
    // Point sits roughly on A's top face plane.
    ASSERT_NEAR(mfd.points[0].position.y, 1.0, 0.15);
}

TEST_SUITE(
    RUN_TEST(epa_sphere_penetration),
    RUN_TEST(epa_box_sphere_penetration),
    RUN_TEST(epa_box_box_face),
    RUN_TEST(epa_box_box_corner_offset),
    RUN_TEST(epa_diagonal_spheres),
    RUN_TEST(epa_rank3_box_tet_tangent),
    RUN_TEST(epa_rank4_dup_vertex_box_tet_rotated),
    RUN_TEST(manifold_hex_prism_stack_face_face),
    RUN_TEST(manifold_tet_on_box_vertex_contact),
    RUN_TEST(manifold_box_box_edge_contact)
)