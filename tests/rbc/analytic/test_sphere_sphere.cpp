#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"

TEST(sphere_sphere_separated)
{
    rbc::Shape sA = rbc::Sphere(1.0), sB = rbc::Sphere(1.0);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(3.0, 0, 0); // gap of 1.0

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Sphere>(sA, tfA, sB, tfB, c)));
}

TEST(sphere_sphere_touching)
{
    // Exactly touching — should NOT count as penetrating
    rbc::Shape sA = rbc::Sphere(1.0), sB = rbc::Sphere(1.0);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(2.0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Sphere>(sA, tfA, sB, tfB, c)));
}

TEST(sphere_sphere_overlapping_depth)
{
    rbc::Shape sA = rbc::Sphere(1.0), sB = rbc::Sphere(1.0);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(1.5, 0, 0); // overlap = 0.5

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Sphere>(sA, tfA, sB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(sA, tfA, sB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.5, 0.01);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.01);
    ASSERT_NEAR(std::abs(analytic.normal.x), std::abs(reference.normal.x), 0.05);
}

TEST(sphere_sphere_diagonal_overlap)
{
    rbc::Shape sA = rbc::Sphere(1.0), sB = rbc::Sphere(1.0);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(1.0, 1.0, 0); // dist = sqrt(2) ≈ 1.414

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Sphere>(sA, tfA, sB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(sA, tfA, sB, tfB, reference));
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
}

TEST(sphere_sphere_coincident_centres)
{
    // Centres at exactly the same position — tests the zero-length guard
    rbc::Shape sA = rbc::Sphere(1.0), sB = rbc::Sphere(1.0);
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Sphere>(sA, tfA, sB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 2.0, 0.01); // full combined radius
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST_SUITE(
    RUN_TEST(sphere_sphere_separated),
    RUN_TEST(sphere_sphere_touching),
    RUN_TEST(sphere_sphere_overlapping_depth),
    RUN_TEST(sphere_sphere_diagonal_overlap),
    RUN_TEST(sphere_sphere_coincident_centres),
)
