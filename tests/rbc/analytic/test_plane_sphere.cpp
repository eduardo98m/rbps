#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"

// Plane tests use geometric reasoning to verify, NOT GJK (planes are infinite,
// GJK is undefined on them). The plane tf is identity; the plane is defined
// in local space by its (normal, d) pair.

static rbc::Shape make_plane_y()
{
    return rbc::Plane(m3d::vec3(0, 1, 0), 0.0); // normal=Y, d=0
}

TEST(sphere_plane_above_no_hit)
{
    // Sphere of radius 1, centre at Y=2.0 → lowest point at Y=1.0, above plane.
    rbc::Shape sA = rbc::Sphere(1.0), pB = make_plane_y();
    auto tfA = test::tf_at(0, 2.0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfB, c)));
}

TEST(sphere_plane_just_touching_no_hit)
{
    rbc::Shape sA = rbc::Sphere(1.0), pB = make_plane_y();
    auto tfA = test::tf_at(0, 1.0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfB, c)));
}

TEST(sphere_plane_penetrating)
{
    // Sphere radius=1, centre at Y=0.7 → penetration = 0.3.
    rbc::Shape sA = rbc::Sphere(1.0), pB = make_plane_y();
    auto tfA = test::tf_at(0, 0.7, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 0.3, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST(sphere_plane_centre_on_plane)
{
    rbc::Shape sA = rbc::Sphere(1.0), pB = make_plane_y();
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 1.0, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(sphere_plane_deep_below)
{
    rbc::Shape sA = rbc::Sphere(1.0), pB = make_plane_y();
    auto tfA = test::tf_at(0, -5.0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 6.0, 0.001);
    ASSERT_NEAR(c.normal.y, 1.0, 0.001);
}

TEST(sphere_plane_diagonal_plane)
{
    // Tilted plane n=normalize(1,1,0), d=0. Sphere at (1,1,0), r=1. dist=√2 → no hit.
    m3d::vec3 n = m3d::normalize(m3d::vec3(1, 1, 0));
    rbc::Shape sA = rbc::Sphere(1.0), pB = rbc::Plane(n, 0.0);
    auto tfA = test::tf_at(1, 1, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfB, c)));
}

TEST(sphere_plane_symmetry_plane_sphere)
{
    rbc::Shape sA = rbc::Sphere(1.0), pB = make_plane_y();
    auto tfA = test::tf_at(0, 0.7, 0);
    auto tfP = test::tf_at(0, 0, 0);

    rbc::ContactManifold c_sp, c_ps;
    test::collide<rbc::Sphere, rbc::Plane>(sA, tfA, pB, tfP, c_sp);
    test::collide<rbc::Plane, rbc::Sphere>(pB, tfP, sA, tfA, c_ps);

    ASSERT_NEAR(test::depth(c_sp), test::depth(c_ps), 0.001);
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
