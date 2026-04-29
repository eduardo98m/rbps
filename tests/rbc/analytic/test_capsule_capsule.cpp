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

// All capsules: half_height=1, radius=0.5, local axis=Y.

TEST(capsule_capsule_separated_parallel)
{
    // Two parallel Y-axis capsules, 3 units apart on X.
    // Closest points on axes: (0,0,0) and (3,0,0). Distance = 3 > rsum 1.0.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(3, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Capsule>::test(
        cA.get<rbc::Capsule>(), tfA, cB.get<rbc::Capsule>(), tfB, c);
    ASSERT_FALSE(hit);
}

TEST(capsule_capsule_overlap_parallel)
{
    // Parallel capsules 0.8 apart on X. rsum = 1.0 → overlap = 0.2.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0.8, 0, 0);

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Capsule>::test(
        cA.get<rbc::Capsule>(), tfA, cB.get<rbc::Capsule>(), tfB, analytic);
    bool hit_r = gjk_reference(cA, tfA, cB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, 0.2, 0.02);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(m3d::length(analytic.normal), 1.0, 0.001);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.05);
}

TEST(capsule_capsule_overlap_perpendicular)
{
    // cA along Y at origin. cB along X at (0, 0, 0).
    // Closest approach: both pass through origin, distance = 0 → deep overlap = rsum.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    // Rotate cB 90° around Z so its axis becomes X
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 0, 0);
    tfB.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 2.0);

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Capsule>::test(
        cA.get<rbc::Capsule>(), tfA, cB.get<rbc::Capsule>(), tfB, analytic);
    bool hit_r = gjk_reference(cA, tfA, cB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    // Overlap should equal sum of radii (axes cross at origin)
    ASSERT_NEAR(analytic.points[0].penetration_depth, 1.0, 0.05);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(m3d::length(analytic.normal), 1.0, 0.001);
}

TEST(capsule_capsule_overlap_crossing_offset)
{
    // cA along Y at origin. cB along X, offset to (0, 0, 0.6).
    // Closest distance between skew axes = 0.6. rsum = 1.0 → overlap = 0.4.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB;
    tfB.pos = m3d::vec3(0, 0, 0.6);
    tfB.rot = m3d::quat::from_axis_angle(m3d::vec3(0, 0, 1), m3d::PI / 2.0);

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Capsule>::test(
        cA.get<rbc::Capsule>(), tfA, cB.get<rbc::Capsule>(), tfB, analytic);
    bool hit_r = gjk_reference(cA, tfA, cB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, 0.4, 0.02);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
}

TEST(capsule_capsule_endpoint_to_endpoint)
{
    // Tips of two capsules facing each other: A top cap at (0,1,0), B bottom cap at (0,-1+3,0)=(0,2,0).
    // Distance = 2.0 - (0.5+0.5) = 1.0... Let me be precise:
    // cA top endpoint = (0,1,0). cB centre at (0, 2.4, 0), bottom endpoint = (0, 1.4, 0).
    // Distance from (0,1,0) to (0,1.4,0) = 0.4. rsum = 1.0 → overlap = 0.6.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0,   0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 2.4, 0);

    rbc::ContactManifold analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Capsule>::test(
        cA.get<rbc::Capsule>(), tfA, cB.get<rbc::Capsule>(), tfB, analytic);
    bool hit_r = gjk_reference(cA, tfA, cB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.points[0].penetration_depth, reference.points[0].penetration_depth, 0.05);
    ASSERT_NEAR(m3d::length(analytic.normal), 1.0, 0.001);
    // Normal should be roughly along Y
    ASSERT_NEAR(std::abs(analytic.normal.y), 1.0, 0.05);
}

TEST(capsule_capsule_separated_endpoint_gap)
{
    // Centres 3.5 apart on Y. Tip-to-tip distance = 3.5 - 2*(1.0) = 1.5.
    // After subtracting radii: 1.5 - 1.0 = 0.5 gap → no hit.
    rbc::Shape cA = rbc::Capsule(1.0, 0.5);
    rbc::Shape cB = rbc::Capsule(1.0, 0.5);
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 3.5, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Capsule, rbc::Capsule>::test(
        cA.get<rbc::Capsule>(), tfA, cB.get<rbc::Capsule>(), tfB, c);
    ASSERT_FALSE(hit);
}

TEST_SUITE(
    RUN_TEST(capsule_capsule_separated_parallel),
    RUN_TEST(capsule_capsule_overlap_parallel),
    RUN_TEST(capsule_capsule_overlap_perpendicular),
    RUN_TEST(capsule_capsule_overlap_crossing_offset),
    RUN_TEST(capsule_capsule_endpoint_to_endpoint),
    RUN_TEST(capsule_capsule_separated_endpoint_gap),
)