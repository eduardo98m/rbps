#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"

// Capsule half_height=1 radius=0.5, axis=Y. Total tip-to-tip = 3.0.

TEST(sphere_capsule_separated)
{
    // Sphere centre at (0,4,0), capsule centre at origin. Closest point on
    // capsule axis = (0,1,0). Distance = 3.0 > sphere_r(1) + cap_r(0.5).
    rbc::Shape sA = rbc::Sphere(1.0), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 4, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Capsule>(sA, tfA, cB, tfB, c)));
}

TEST(sphere_capsule_overlap_shaft)
{
    // Sphere at (1.2, 0, 0). Distance to capsule axis = 1.2 < rsum (1.5).
    rbc::Shape sA = rbc::Sphere(1.0), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(1.2, 0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Capsule>(sA, tfA, cB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(sA, tfA, cB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.3, 0.02);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NORMAL_UNIT(analytic.normal);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.05);
}

TEST(sphere_capsule_overlap_endpoint)
{
    // Sphere at (0, 2.0, 0). Capsule top endpoint at (0,1,0). Distance = 1.0.
    // rsum = 1.5 → depth = 0.5.
    rbc::Shape sA = rbc::Sphere(1.0), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 2.0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Capsule>(sA, tfA, cB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(sA, tfA, cB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.5, 0.02);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NORMAL_UNIT(analytic.normal);
}

TEST(sphere_capsule_no_hit_endpoint)
{
    rbc::Shape sA = rbc::Sphere(1.0), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(0, 3.0, 0); // dist = 2.0 > rsum 1.5
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Capsule>(sA, tfA, cB, tfB, c)));
}

TEST(sphere_capsule_symmetry)
{
    rbc::Shape sA = rbc::Sphere(1.0), cB = rbc::Capsule(1.0, 0.5);
    auto tfA = test::tf_at(1.2, 0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c_sc, c_cs;
    test::collide<rbc::Sphere, rbc::Capsule>(sA, tfA, cB, tfB, c_sc);
    test::collide<rbc::Capsule, rbc::Sphere>(cB, tfB, sA, tfA, c_cs);

    ASSERT_NEAR(test::depth(c_sc), test::depth(c_cs), 0.001);
    ASSERT_NEAR(c_sc.normal.x, -c_cs.normal.x, 0.001);
    ASSERT_NEAR(c_sc.normal.y, -c_cs.normal.y, 0.001);
    ASSERT_NEAR(c_sc.normal.z, -c_cs.normal.z, 0.001);
}

TEST_SUITE(
    RUN_TEST(sphere_capsule_separated),
    RUN_TEST(sphere_capsule_overlap_shaft),
    RUN_TEST(sphere_capsule_overlap_endpoint),
    RUN_TEST(sphere_capsule_no_hit_endpoint),
    RUN_TEST(sphere_capsule_symmetry),
)
