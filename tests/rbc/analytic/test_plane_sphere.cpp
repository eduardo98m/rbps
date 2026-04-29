#include "tests/test_helper.hpp"
#include "rbc/Dispatcher.hpp"

// Plane tests use geometric reasoning to verify, NOT GJK (planes are infinite,
// GJK is undefined on them). Instead we cross-check normal direction and depth
// against the known geometry.

// Plane at Y=0, normal=(0,1,0). All shapes will be tested against this.
static m3d::tf plane_tf()
{
    m3d::tf tf;
    tf.pos = m3d::vec3(0, 0, 0); // plane surface at Y=0
    return tf;
}

static rbc::Shape make_plane_y()
{
    return rbc::Plane(m3d::vec3(0, 1, 0), 0.0); // normal=Y, d=0
}

TEST(sphere_plane_above_no_hit)
{
    // Sphere of radius 1 with centre at Y=2.0 → lowest point at Y=1.0, above plane.
    rbc::Shape sA  = rbc::Sphere(1.0);
    rbc::Shape pB  = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 2.0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);
    ASSERT_FALSE(hit);
}

TEST(sphere_plane_just_touching_no_hit)
{
    // Sphere of radius 1 with centre at Y=1.0 → lowest point exactly at Y=0. No penetration.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 1.0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);
    ASSERT_FALSE(hit);
}

TEST(sphere_plane_penetrating)
{
    // Sphere radius=1, centre at Y=0.7 → penetration = 1.0 - 0.7 = 0.3.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0.7, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 0.3, 0.001);
    // Normal should push sphere in the +Y direction (out of plane)
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
    ASSERT_NEAR(m3d::length(c.normal), 1.0, 0.001);
}

TEST(sphere_plane_centre_on_plane)
{
    // Sphere radius=1, centre at Y=0. Penetration = radius = 1.0.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 1.0, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(sphere_plane_deep_below)
{
    // Sphere radius=1, centre at Y=-5.0. Penetration = 1.0 + 5.0 = 6.0.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, -5.0, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);

    ASSERT_TRUE(hit);
    ASSERT_NEAR(c.points[0].penetration_depth, 6.0, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(sphere_plane_diagonal_plane)
{
    // Tilted plane: normal = normalize(1,1,0), d=0. Sphere at (1,1,0), radius 1.
    // Distance from centre to plane = dot((1,1,0), normalize(1,1,0)) = sqrt(2) ≈ 1.414.
    // Penetration = radius - dist = 1.0 - 1.414 → no hit.
    m3d::vec3 n = m3d::normalize(m3d::vec3(1, 1, 0));
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape pB = rbc::Plane(n, 0.0);
    m3d::tf tfA; tfA.pos = m3d::vec3(1, 1, 0);

    rbc::ContactManifold c;
    bool hit = rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c);
    ASSERT_FALSE(hit);
}

TEST(sphere_plane_symmetry_plane_sphere)
{
    // Plane vs Sphere should give same depth, flipped normal.
    rbc::Shape sA = rbc::Sphere(1.0);
    rbc::Shape pB = make_plane_y();
    m3d::tf tfA; tfA.pos = m3d::vec3(0, 0.7, 0);

    rbc::ContactManifold c_sp, c_ps;
    rbc::CollisionAlgorithm<rbc::Sphere, rbc::Plane>::test(
        sA.get<rbc::Sphere>(), tfA, pB.get<rbc::Plane>(), plane_tf(), c_sp);
    rbc::CollisionAlgorithm<rbc::Plane, rbc::Sphere>::test(
        pB.get<rbc::Plane>(), plane_tf(), sA.get<rbc::Sphere>(), tfA, c_ps);

    ASSERT_NEAR(c_sp.points[0].penetration_depth, c_ps.points[0].penetration_depth, 0.001);
    ASSERT_NEAR(c_sp.normal.x, -c_ps.normal.x, 0.001);
    ASSERT_NEAR(c_sp.normal.y, -c_ps.normal.y, 0.001);
    ASSERT_NEAR(c_sp.normal.z, -c_ps.normal.z, 0.001);
}

TEST_SUITE(
    RUN_TEST(sphere_plane_above_no_hit),
    RUN_TEST(sphere_plane_just_touching_no_hit),
    RUN_TEST(sphere_plane_penetrating),
    RUN_TEST(sphere_plane_centre_on_plane),
    RUN_TEST(sphere_plane_deep_below),
    RUN_TEST(sphere_plane_diagonal_plane),
    RUN_TEST(sphere_plane_symmetry_plane_sphere),
)