#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"

TEST(epa_sphere_penetration)
{
    rbc::Sphere sA(1.0); 
    rbc::Sphere sB(1.0); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.5, 0, 0); // Overlap of 0.5 along X

    rbc::MinkowskiDiff md(&sA, &sB, tfA, tfB);
    rbc::GJK gjk;
    
    // GJK must detect the collision first
    gjk.evaluate(md, m3d::vec3(1, 0, 0));
    std::cout << "GJK rank: " << gjk.active_simplex->rank << "\n";

    
    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    std::cout << "EPA status: " << (int)status 
            << " depth=" << epa.depth 
            << " normal=" << epa.normal.x << "," << epa.normal.y << "," << epa.normal.z << "\n";

    ASSERT_TRUE(status == rbc::EPA::Valid);
    
    // Check Depth: 2.0 (combined radii) - 1.5 (distance) = 0.5
    ASSERT_NEAR(epa.depth, 0.5, 0.01);

    // Check Normal: Should push Body B in +X direction
    ASSERT_NEAR(epa.normal.x, 1.0, 0.01);
    ASSERT_NEAR(epa.normal.y, 0.0, 0.01);
    ASSERT_NEAR(epa.normal.z, 0.0, 0.01);
}

TEST(epa_box_sphere_penetration)
{
    rbc::Box boxA(m3d::vec3(1, 1, 1)); 
    rbc::Sphere sphereB(1.0);
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    // Position sphere center at Y=1.7. 
    // Sphere bottom (1.7 - 1.0) = 0.7. Box top = 1.0. 
    // Overlap = 0.3 along Y.
    m3d::tf tfB; tfB.pos = m3d::vec3(0, 1.7, 0); 

    rbc::MinkowskiDiff md(&boxA, &sphereB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, m3d::vec3(0, 1, 0));

    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    ASSERT_TRUE(status == rbc::EPA::Valid);
    
    ASSERT_NEAR(epa.depth, 0.3, 0.01);
    ASSERT_NEAR(epa.normal.y, 1.0, 0.01); // Normal should point along Y
}

TEST_SUITE(
    RUN_TEST(epa_sphere_penetration),
    RUN_TEST(epa_box_sphere_penetration)
)