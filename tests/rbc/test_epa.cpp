#include "tests/test_helper.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"

// --- Existing Tests ---
TEST(epa_sphere_penetration)
{
    rbc::Sphere sA(1.0); rbc::Sphere sB(1.0); 
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
    rbc::Box boxA(m3d::vec3(1, 1, 1)); 
    rbc::Sphere sphereB(1.0);
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
    rbc::Box bA(m3d::vec3(1, 1, 1)); 
    rbc::Box bB(m3d::vec3(1, 1, 1)); 
    
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);
    m3d::tf tfB; tfB.pos = m3d::vec3(1.6, 0, 0); // Overlap of 0.4 on X axis

    rbc::MinkowskiDiff md(&bA, &bB, tfA, tfB);
    rbc::GJK gjk;
    gjk.evaluate(md, tfB.pos - tfA.pos);

    rbc::EPA epa;
    rbc::EPA::Status status = epa.evaluate(gjk, md);

    std::cout << "GJK rank: " << gjk.active_simplex->rank << "\n";
    std::cout << "EPA status: " << (int)status 
            << " depth=" << epa.depth 
            << " normal=" << epa.normal.x << "," << epa.normal.y << "," << epa.normal.z << "\n";

    ASSERT_TRUE(status == rbc::EPA::Valid);
    ASSERT_NEAR(epa.depth, 0.4, 0.01);
    ASSERT_NEAR(std::abs(epa.normal.x), 1.0, 0.01);
}

TEST(epa_box_box_corner_offset)
{
    rbc::Box bA(m3d::vec3(1, 1, 1)); 
    rbc::Box bB(m3d::vec3(1, 1, 1)); 
    
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
    rbc::Sphere sA(1.0); 
    rbc::Sphere sB(1.0); 
    
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

TEST_SUITE(
    RUN_TEST(epa_sphere_penetration),
    RUN_TEST(epa_box_sphere_penetration),
    RUN_TEST(epa_box_box_face),
    RUN_TEST(epa_box_box_corner_offset),
    RUN_TEST(epa_diagonal_spheres)
)