#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

// Helper: runs GJK+EPA on two shapes and returns the contact.
// Used to cross-validate analytic results.
static bool gjk_reference(const rbc::Shape& sa, const m3d::tf& tfa,
                           const rbc::Shape& sb, const m3d::tf& tfb,
                           rbc::ContactManifold& out)
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

TEST(sphere_sphere_separated)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(3.0, 0, 0);   // gap of 1.0

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Sphere>::test(
        sA.get<rbc::Sphere>(), tfA, sB.get<rbc::Sphere>(), tfB, c);

    ASSERT_FALSE(hit);
}

TEST(sphere_sphere_touching)
{
    // Exactly touching — should NOT count as penetrating
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(2.0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Sphere>::test(
        sA.get<rbc::Sphere>(), tfA, sB.get<rbc::Sphere>(), tfB, c);

    ASSERT_FALSE(hit);
}

TEST(sphere_sphere_overlapping_depth)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.5, 0, 0);   // overlap = 0.5

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Sphere>::test(
        sA.get<rbc::Sphere>(), tfA, sB.get<rbc::Sphere>(), tfB, analytic);
    bool hit_r = gjk_reference(sA, tfA, sB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, 0.5,              0.01);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0,             0.01);
    ASSERT_NEAR(std::abs(analytic.normal.x), std::abs(reference.normal.x), 0.05);
}

TEST(sphere_sphere_diagonal_overlap)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.0, 1.0, 0);  // dist = sqrt(2) ≈ 1.414

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Sphere>::test(
        sA.get<rbc::Sphere>(), tfA, sB.get<rbc::Sphere>(), tfB, analytic);
    bool hit_r = gjk_reference(sA, tfA, sB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
}

TEST(sphere_sphere_coincident_centres)
{
    // Centres at exactly the same position — tests the zero-length guard
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Sphere>::test(
        sA.get<rbc::Sphere>(), tfA, sB.get<rbc::Sphere>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 2.0, 0.01);   // full combined radius
    // Normal must be unit length even though centres coincide
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}


TEST_SUITE(
    RUN_TEST(sphere_sphere_separated),
    RUN_TEST(sphere_sphere_touching),
    RUN_TEST(sphere_sphere_overlapping_depth),
    RUN_TEST(sphere_sphere_diagonal_overlap),
    RUN_TEST(sphere_sphere_coincident_centres),
)