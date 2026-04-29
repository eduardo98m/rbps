#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

static bool gjk_reference(const rbc::Shape &sa, const m3d::tf &tfa,
                           const rbc::Shape &sb, const m3d::tf &tfb,
                           rbc::ContactManifold &out)
{
    rbc::MinkowskiDiff md(&sa, &sb, tfa, tfb);
    m3d::vec3 guess = tfb.pos - tfa.pos;
    if (m3d::length_sq(guess) < m3d::EPSILON) guess = m3d::vec3(1.0, 0.0, 0.0);
    rbc::GJK gjk;
    if (gjk.evaluate(md, guess) != rbc::GJK::Inside) return false;
    rbc::EPA epa;
    if (epa.evaluate(gjk, md) != rbc::EPA::Valid) return false;
    out.normal                       = epa.normal;
    out.num_points                   = 1;
    out.points[0].penetration_depth  = epa.depth;
    out.points[0].position           = epa.contact_point;
    return true;
}

// Capsule half_height=1 radius=0.5, axis=Y. Total tip-to-tip = 3.0.

TEST(sphere_capsule_separated)
{
    // Sphere centre at (0,4,0), capsule centre at origin. Closest point on
    // capsule axis = (0,1,0). Distance = 3.0 > sphere_r(1) + cap_r(0.5).
    rbc::Shape sA  = rbc::Sphere(1.0);
    rbc::Shape cB  = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 4, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Capsule>::test(
        sA.get<rbc::Sphere>(), tfA, cB.get<rbc::Capsule>(), tfB, c);
    ASSERT_FALSE(hit);
}

TEST(sphere_capsule_overlap_shaft)
{
    // Sphere at (1.2, 0, 0), capsule on Y axis. Closest point on capsule axis
    // is (0,0,0). Distance = 1.2. rsum = 1+0.5 = 1.5 → overlap = 0.3.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(1.2, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0,   0, 0);

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Capsule>::test(
        sA.get<rbc::Sphere>(), tfA, cB.get<rbc::Capsule>(), tfB, analytic);
    bool hit_r = gjk_reference(sA, tfA, cB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, 0.3, 0.02);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(m3d::length(analytic.normal), 1.0, 0.001);
    // Normal must be roughly along X
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.05);
}

TEST(sphere_capsule_overlap_endpoint)
{
    // Sphere at (0, 3.0, 0). Capsule top endpoint at (0,1,0).
    // Distance from sphere centre to endpoint = 2.0.
    // rsum = 1.0 + 0.5 = 1.5. 2.0 > 1.5 → no hit.
    // Move closer: sphere at (0, 2.0, 0). Distance = 1.0 < 1.5 → hit, depth = 0.5.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 2.0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 0,   0);

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Capsule>::test(
        sA.get<rbc::Sphere>(), tfA, cB.get<rbc::Capsule>(), tfB, analytic);
    bool hit_r = gjk_reference(sA, tfA, cB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, 0.5, 0.02);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(m3d::length(analytic.normal), 1.0, 0.001);
}

TEST(sphere_capsule_no_hit_endpoint)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 3.0, 0); // dist = 2.0 > rsum 1.5
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 0,   0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Capsule>::test(
        sA.get<rbc::Sphere>(), tfA, cB.get<rbc::Capsule>(), tfB, c);
    ASSERT_FALSE(hit);
}

TEST(sphere_capsule_symmetry)
{
    // CollisionAlgorithm<Capsule, Sphere> must give same depth, flipped normal.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(1.2, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0,   0, 0);

    rbc::ContactManifold c_sc, c_cs;
    rbc::CollisionAlgorithm<rbc::Sphere, rbc::Capsule>::test(
        sA.get<rbc::Sphere>(), tfA, cB.get<rbc::Capsule>(), tfB, c_sc);
    rbc::CollisionAlgorithm<rbc::Capsule, rbc::Sphere>::test(
        cB.get<rbc::Capsule>(), tfB, sA.get<rbc::Sphere>(), tfA, c_cs);

    ASSERT_NEAR(c_sc.points[0].penetration_depth, c_cs.points[0].penetration_depth, 0.001);
    ASSERT_NEAR(c_sc.normal.x, -c_cs.normal.x, 0.001);
    ASSERT_NEAR(c_sc.normal.y, -c_cs.normal.y, 0.001);
    ASSERT_NEAR(c_sc.normal.z, -c_cs.normal.z, 0.001);
}

TEST_SUITE(
    RUN_TEST(sphere_capsule_separated),
    RUN_TEST(sphere_capsule_overlap_shaft),
    RUN_TEST(sphere_capsule_overlap_endpoint),
    RUN_TEST(sphere_capsule_no_hit_endpoint),
    RUN_TEST(sphere_capsule_symmetry),
)