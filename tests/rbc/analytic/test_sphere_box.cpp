#include "tests/test_helper.hpp"
#include "tests/rbc/collision_helpers.hpp"

TEST(sphere_box_separated)
{
    rbc::Shape sA = rbc::Sphere(1.0), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(4.0, 0, 0); // clearly separated

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, c)));
}

TEST(sphere_box_face_overlap_x)
{
    rbc::Shape sA = rbc::Sphere(1.0), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(1.8, 0, 0); // sphere overlaps box face by 0.2

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(sA, tfA, bB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.x), 1.0, 0.05);
}

TEST(sphere_box_face_overlap_y)
{
    rbc::Shape sA = rbc::Sphere(1.0), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 1.7, 0); // overlap ≈ 0.3

    rbc::ContactManifold analytic, reference;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, analytic)));
    ASSERT_TRUE(test::gjk_reference(sA, tfA, bB, tfB, reference));

    ASSERT_NEAR(test::depth(analytic), 0.3, 0.05);
    ASSERT_NEAR(test::depth(analytic), test::depth(reference), 0.05);
    ASSERT_NEAR(std::abs(analytic.normal.y), 1.0, 0.05);
}

TEST(sphere_box_corner_region_no_hit)
{
    // Sphere near the corner but not close enough to touch it.
    // Box corner is at (1,1,1). Sphere centre at (2,2,2).
    // Distance to corner = sqrt(3) ≈ 1.732 > radius 1.0 → no hit.
    rbc::Shape sA = rbc::Sphere(1.0), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(2, 2, 2);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_FALSE((test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, c)));
}

TEST(sphere_box_corner_region_hit)
{
    // Box corner at (1,1,1). Sphere centre at (1.5,1.5,1.5).
    // Distance to corner = sqrt(0.75) ≈ 0.866 < radius 1.0 → hit.
    rbc::Shape sA = rbc::Sphere(1.0), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(1.5, 1.5, 1.5);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, c)));
    ASSERT_NEAR(test::depth(c), 1.0 - 0.866, 0.01);
    ASSERT_NORMAL_UNIT(c.normal);
}

TEST(sphere_box_centre_inside_box)
{
    rbc::Shape sA = rbc::Sphere(0.3), bB = rbc::Box(m3d::vec3(2, 2, 2));
    auto tfA = test::tf_at(0.1, 0.1, 0.1);
    auto tfB = test::tf_at(0, 0, 0);

    rbc::ContactManifold c;
    ASSERT_TRUE((test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, c)));
    ASSERT_NORMAL_UNIT(c.normal);
    ASSERT_TRUE(test::depth(c) > 0.0);
}

TEST(box_sphere_symmetry)
{
    // CollisionAlgorithm<Box, Sphere> must give the same depth as <Sphere, Box>
    // but with flipped normal.
    rbc::Shape sA = rbc::Sphere(1.0), bB = rbc::Box(m3d::vec3(1, 1, 1));
    auto tfA = test::tf_at(0, 0, 0);
    auto tfB = test::tf_at(0, 1.7, 0);

    rbc::ContactManifold c_sb, c_bs;
    test::collide<rbc::Sphere, rbc::Box>(sA, tfA, bB, tfB, c_sb);
    test::collide<rbc::Box, rbc::Sphere>(bB, tfB, sA, tfA, c_bs);

    ASSERT_NEAR(test::depth(c_sb), test::depth(c_bs), 0.001);
    ASSERT_NEAR(c_sb.normal.x, -c_bs.normal.x, 0.001);
    ASSERT_NEAR(c_sb.normal.y, -c_bs.normal.y, 0.001);
    ASSERT_NEAR(c_sb.normal.z, -c_bs.normal.z, 0.001);
}

TEST_SUITE(
    RUN_TEST(sphere_box_separated),
    RUN_TEST(sphere_box_face_overlap_x),
    RUN_TEST(sphere_box_face_overlap_y),
    RUN_TEST(sphere_box_corner_region_no_hit),
    RUN_TEST(sphere_box_corner_region_hit),
    RUN_TEST(sphere_box_centre_inside_box),
    RUN_TEST(box_sphere_symmetry),
)
