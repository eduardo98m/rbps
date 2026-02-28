#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

// Helper: runs GJK+EPA on two shapes and returns the contact.
// Used to cross-validate analytic results.
static bool gjk_reference(const rbc::Shape& sa, const m3d::tf& tfa,
                           const rbc::Shape& sb, const m3d::tf& tfb,
                           rbc::Contact& out)
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

    out.normal            = epa.normal;
    out.penetration_depth = epa.depth;
    out.pos               = epa.contact_point;
    return true;
}

// =============================================================================
// BOX vs BOX  (SAT)
// =============================================================================

TEST(box_box_separated_x)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(2.5, 0, 0);   // gap of 0.5

    rbc::Contact c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Box>::test(
        bA.get<rbc::Box>(), tfA, bB.get<rbc::Box>(), tfB, c);

    ASSERT_FALSE(hit);
}

TEST(box_box_face_overlap_x)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.6, 0, 0);   // overlap = 0.4

    rbc::Contact analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Box, rbc::Box>::test(
        bA.get<rbc::Box>(), tfA, bB.get<rbc::Box>(), tfB, analytic);
    bool hit_r = gjk_reference(bA, tfA, bB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.penetration_depth, 0.4,                         0.01);
    ASSERT_NEAR(analytic.penetration_depth, reference.penetration_depth, 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0,                        0.01);
}

TEST(box_box_face_overlap_z)
{
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0  );
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 0, 1.8);   // overlap = 0.2

    rbc::Contact analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Box, rbc::Box>::test(
        bA.get<rbc::Box>(), tfA, bB.get<rbc::Box>(), tfB, analytic);
    bool hit_r = gjk_reference(bA, tfA, bB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.penetration_depth, 0.2,                         0.01);
    ASSERT_NEAR(analytic.penetration_depth, reference.penetration_depth, 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.z), 1.0,                        0.01);
}

TEST(box_box_minimum_axis_chosen)
{
    // Three overlapping axes — SAT must pick the smallest one (Z = 0.2)
    rbc::Shape bA = rbc::Box(m3d::vec3(1, 1, 1));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0,   0  );
    m3d::tf tfB; tfB.pos = m3d::vec3(0.5, 0.5, 1.8);

    rbc::Contact analytic, reference;
    bool hit_a = rbc::CollisionAlgorithm<rbc::Box, rbc::Box>::test(
        bA.get<rbc::Box>(), tfA, bB.get<rbc::Box>(), tfB, analytic);
    bool hit_r = gjk_reference(bA, tfA, bB, tfB, reference);

    ASSERT_TRUE(hit_a);
    ASSERT_TRUE(hit_r);
    ASSERT_NEAR(analytic.penetration_depth, 0.2,                         0.01);
    ASSERT_NEAR(analytic.penetration_depth, reference.penetration_depth, 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.z), 1.0,                        0.01);
}

TEST(box_box_deep_penetration)
{
    // One box mostly inside the other
    rbc::Shape bA = rbc::Box(m3d::vec3(3, 3, 3));
    rbc::Shape bB = rbc::Box(m3d::vec3(1, 1, 1));
    m3d::tf tfA; tfA.pos = m3d::vec3(0,   0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(0.2, 0, 0);

    rbc::Contact c;
    bool hit = rbc::CollisionAlgorithm<rbc::Box, rbc::Box>::test(
        bA.get<rbc::Box>(), tfA, bB.get<rbc::Box>(), tfB, c);

    ASSERT_TRUE(hit);
    ASSERT_TRUE(c.penetration_depth > 0.0);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

TEST_SUITE(
    RUN_TEST(box_box_separated_x),
    RUN_TEST(box_box_face_overlap_x),
    RUN_TEST(box_box_face_overlap_z),
    RUN_TEST(box_box_minimum_axis_chosen),
    RUN_TEST(box_box_deep_penetration)
)