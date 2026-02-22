#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"

TEST(spheres_separated)
{
    rbc::Sphere sA(1.0); 
    rbc::Sphere sB(1.0); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(3.0, 0, 0); // Distance 3.0, Combined radii 2.0

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    m3d::vec3 guess = tfB.pos - tfA.pos;

    rbc::GJK::Status status = gjk.evaluate(md, guess);
    
    // Status should be Valid (separated) or Failed, but NOT Inside
    ASSERT_FALSE(status == rbc::GJK::Inside);
}

TEST(spheres_overlapping)
{
    rbc::Sphere sA(1.0); 
    rbc::Sphere sB(1.0); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.5, 0, 0); // Overlap of 0.5

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    m3d::vec3 guess = tfB.pos - tfA.pos;

    rbc::GJK::Status status = gjk.evaluate(md, guess);
    
    ASSERT_TRUE(status == rbc::GJK::Inside);
}

TEST(box_sphere_overlapping)
{
    rbc::Box boxA(m3d::vec3(1, 1, 1)); // 2x2x2 Box
    rbc::Sphere sphereB(1.0);
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.8, 0, 0); // Sphere edge at 0.8, Box edge at 1.0

    rbc::MinkowskiDiff md(&boxA, &sphereB, tfA, tfB);
    rbc::GJK gjk;
    m3d::vec3 guess = tfB.pos - tfA.pos;

    rbc::GJK::Status status = gjk.evaluate(md, guess);
    
    ASSERT_TRUE(status == rbc::GJK::Inside);
}

TEST_SUITE(
    RUN_TEST(spheres_separated),
    RUN_TEST(spheres_overlapping),
    RUN_TEST(box_sphere_overlapping)
)