#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"

// =============================================================================
// DISPATCHER TESTS
//
// These tests verify that dispatch() routes each shape-pair combination to the
// correct algorithm and produces consistent results. Each test checks:
//   1. The correct hit/no-hit result
//   2. That depth and normal are physically reasonable
//   3. Where relevant, that symmetry holds (swapping A and B flips the normal)
// =============================================================================

// ── Sphere vs Sphere ──────────────────────────────────────────────────────────

TEST(dispatch_sphere_sphere_no_hit)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(3.0, 0, 0);

    rbc::Contact c;
    ASSERT_FALSE(rbc::dispatch(sA, tfA, sB, tfB, c));
}

TEST(dispatch_sphere_sphere_hit)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.5, 0, 0);

    rbc::Contact c;
    ASSERT_TRUE(rbc::dispatch(sA, tfA, sB, tfB, c));
    ASSERT_NEAR(c.penetration_depth, 0.5, 0.01);
    ASSERT_NEAR(std::abs(c.normal.x), 1.0, 0.01);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

// ── Sphere vs Box ─────────────────────────────────────────────────────────────

TEST(dispatch_sphere_box_no_hit)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(4.0, 0, 0);

    rbc::Contact c;
    ASSERT_FALSE(rbc::dispatch(sA, tfA, bB, tfB, c));
}

TEST(dispatch_sphere_box_hit)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,    0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 1.7, 0);

    rbc::Contact c;
    ASSERT_TRUE(rbc::dispatch(sA, tfA, bB, tfB, c));
    ASSERT_NEAR(c.penetration_depth, 0.3, 0.05);
    ASSERT_NEAR(std::abs(c.normal.y), 1.0, 0.05);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

// ── Box vs Sphere (symmetric path) ───────────────────────────────────────────

TEST(dispatch_box_sphere_hit)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 1.7, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0,    0, 0);

    rbc::Contact c;
    ASSERT_TRUE(rbc::dispatch(bA, tfA, sB, tfB, c));
    ASSERT_NEAR(c.penetration_depth, 0.3, 0.05);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

TEST(dispatch_sphere_box_symmetry)
{
    // Swapping A and B should give the same depth and opposite normals
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfS; tfS.pos = m3d::vec3(0,    0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 1.7, 0);

    rbc::Contact c1, c2;
    rbc::dispatch(sA, tfS, bB, tfB, c1);   // Sphere first
    rbc::dispatch(bB, tfB, sA, tfS, c2);   // Box first

    ASSERT_NEAR(c1.penetration_depth, c2.penetration_depth, 0.001);
    ASSERT_NEAR(c1.normal.x, -c2.normal.x, 0.001);
    ASSERT_NEAR(c1.normal.y, -c2.normal.y, 0.001);
    ASSERT_NEAR(c1.normal.z, -c2.normal.z, 0.001);
}

// ── Box vs Box ────────────────────────────────────────────────────────────────

TEST(dispatch_box_box_no_hit)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(2.5, 0, 0);

    rbc::Contact c;
    ASSERT_FALSE(rbc::dispatch(bA, tfA, bB, tfB, c));
}

TEST(dispatch_box_box_hit)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.6, 0, 0);   // overlap = 0.4

    rbc::Contact c;
    ASSERT_TRUE(rbc::dispatch(bA, tfA, bB, tfB, c));
    ASSERT_NEAR(c.penetration_depth, 0.4, 0.01);
    ASSERT_NEAR(std::abs(c.normal.x), 1.0, 0.01);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

// ── Dispatcher routing sanity: same result as calling algorithm directly ──────

TEST(dispatch_matches_direct_call_sphere_sphere)
{
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape sB = rbc::Sphere(1.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.5, 0, 0);

    rbc::Contact c_dispatch, c_direct;
    rbc::dispatch(sA, tfA, sB, tfB, c_dispatch);
    rbc::CollisionAlgorithm<rbc::Sphere, rbc::Sphere>::test(
        sA.get<rbc::Sphere>(), tfA, sB.get<rbc::Sphere>(), tfB, c_direct);

    ASSERT_NEAR(c_dispatch.penetration_depth, c_direct.penetration_depth, 0.001);
    ASSERT_NEAR(c_dispatch.normal.x,          c_direct.normal.x,          0.001);
    ASSERT_NEAR(c_dispatch.normal.y,          c_direct.normal.y,          0.001);
    ASSERT_NEAR(c_dispatch.normal.z,          c_direct.normal.z,          0.001);
}

TEST(dispatch_matches_direct_call_box_box)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.6, 0, 0);

    rbc::Contact c_dispatch, c_direct;
    rbc::dispatch(bA, tfA, bB, tfB, c_dispatch);
    rbc::CollisionAlgorithm<rbc::Box, rbc::Box>::test(
        bA.get<rbc::Box>(), tfA, bB.get<rbc::Box>(), tfB, c_direct);

    ASSERT_NEAR(c_dispatch.penetration_depth, c_direct.penetration_depth, 0.001);
    ASSERT_NEAR(c_dispatch.normal.x,          c_direct.normal.x,          0.001);
    ASSERT_NEAR(c_dispatch.normal.y,          c_direct.normal.y,          0.001);
    ASSERT_NEAR(c_dispatch.normal.z,          c_direct.normal.z,          0.001);
}

TEST_SUITE(
    RUN_TEST(dispatch_sphere_sphere_no_hit),
    RUN_TEST(dispatch_sphere_sphere_hit),
    RUN_TEST(dispatch_sphere_box_no_hit),
    RUN_TEST(dispatch_sphere_box_hit),
    RUN_TEST(dispatch_box_sphere_hit),
    RUN_TEST(dispatch_sphere_box_symmetry),
    RUN_TEST(dispatch_box_box_no_hit),
    RUN_TEST(dispatch_box_box_hit),
    RUN_TEST(dispatch_matches_direct_call_sphere_sphere),
    RUN_TEST(dispatch_matches_direct_call_box_box)
)