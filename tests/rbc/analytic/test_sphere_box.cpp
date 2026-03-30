#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

// Helper: runs GJK+EPA on two shapes and returns the contact.
// Used to cross-validate analytic results.
static bool gjk_reference(const rbc::Shape &sa, const m3d::tf &tfa,
                          const rbc::Shape &sb, const m3d::tf &tfb,
                          rbc::ContactManifold &out)
{
    rbc::MinkowskiDiff md(&sa, &sb, tfa, tfb);
    m3d::vec3 guess = tfb.pos - tfa.pos;
    if (m3d::length_sq(guess) < m3d::EPSILON)
        guess = m3d::vec3(1.0, 0.0, 0.0);

    rbc::GJK gjk;
    if (gjk.evaluate(md, guess) != rbc::GJK::Inside)
        return false;

    rbc::EPA epa;
    if (epa.evaluate(gjk, md) != rbc::EPA::Valid)
        return false;

    out.normal = epa.normal;
    out.num_points = 1;
    out.points[0].penetration_depth = epa.depth;
    out.points[0].position = epa.contact_point;
    return true;
}

TEST(sphere_box_separated)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(4.0, 0, 0); // clearly separated

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, c);

    ASSERT_FALSE(hit);
}

TEST(sphere_box_face_overlap_x)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(1.8, 0, 0); // sphere overlaps box face by 0.2

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, analytic);
    bool hit_r = gjk_reference(sA, tfA, bB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.05);
}

TEST(sphere_box_face_overlap_y)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 1.7, 0); // overlap ≈ 0.3

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, analytic);
    bool hit_r = gjk_reference(sA, tfA, bB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, 0.3, 0.05);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.y), 1.0, 0.05);
}

TEST(sphere_box_corner_region_no_hit)
{
    // Sphere near the corner but not close enough to touch it.
    // Box corner is at (1,1,1). Sphere centre at (2,2,2).
    // Distance to corner = sqrt(3 * 1^2) ≈ 1.732 > radius 1.0 → no hit.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(2, 2, 2);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, c);

    ASSERT_FALSE(hit);
}

TEST(sphere_box_corner_region_hit)
{
    // Sphere close enough to the corner to just touch it.
    // Box corner at (1,1,1). Sphere centre at (1.5,1.5,1.5).
    // Distance to corner = sqrt(3 * 0.25) ≈ 0.866 < radius 1.0 → hit.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(1.5, 1.5, 1.5);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 1.0 - 0.866, 0.01); // radius - dist_to_corner
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

TEST(sphere_box_centre_inside_box)
{
    // Sphere centre is fully inside the box
    rbc::Shape sA = rbc::Sphere(0.3);
    rbc::Shape bB = rbc::Box(m3d::vec3(2, 2, 2));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0.1, 0.1, 0.1);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, c);

    ASSERT_TRUE(hit);
    // Normal must be unit length
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
    // Penetration must be positive
    ASSERT_TRUE(c.points[0].penetration_depth > 0.0);
}

TEST(box_sphere_symmetry)
{
    // CollisionAlgorithm<Box, Sphere> must give the same depth as <Sphere, Box>
    // but with flipped normal
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA;
    tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 1.7, 0);

    rbc::ContactManifold c_sb, c_bs;
    rbc::CollisionAlgorithm<rbc::Sphere, rbc::Box>::test(
        sA.get<rbc::Sphere>(), tfA, bB.get<rbc::Box>(), tfB, c_sb);
    rbc::CollisionAlgorithm<rbc::Box, rbc::Sphere>::test(
        bB.get<rbc::Box>(), tfB, sA.get<rbc::Sphere>(), tfA, c_bs);

    ASSERT_NEAR(c_sb.points[0].penetration_depth, c_bs.points[0].penetration_depth, 0.001);
    // Normals should be opposite
    ASSERT_NEAR(c_sb.normal.x, -c_bs.normal.x, 0.001);
    ASSERT_NEAR(c_sb.normal.y, -c_bs.normal.y, 0.001);
    ASSERT_NEAR(c_sb.normal.z, -c_bs.normal.z, 0.001);
}

TEST_SUITE(
    RUN_TEST(sphere_box_separated),
    RUN_TEST(sphere_box_face_overlap_x),
    RUN_TEST(sphere_box_face_overlap_y),
    RUN_TEST(sphere_box_corner_region_no_hit),
    RUN_TEST(sphere_box_corner_region_hit),
    RUN_TEST(sphere_box_centre_inside_box),
    RUN_TEST(box_sphere_symmetry), )