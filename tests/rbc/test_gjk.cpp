#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"

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

TEST_SUITE(
    RUN_TEST(spheres_separated),
    RUN_TEST(spheres_overlapping),
    RUN_TEST(box_sphere_overlapping),
    RUN_TEST(box_box_separated),
    RUN_TEST(box_box_overlapping),
    RUN_TEST(diagonal_separated),
    RUN_TEST(deep_penetration))