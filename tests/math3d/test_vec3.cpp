#include "math3d/vec3.hpp"
#include "math3d/test_helper.hpp"

TEST(vec3_basics) {
    m3d::vec3 v1(1, 2, 3);
    m3d::vec3 v2(4, 5, 6);
    
    m3d::vec3 add = v1 + v2;
    ASSERT_NEAR(add.x, 5); ASSERT_NEAR(add.y, 7); ASSERT_NEAR(add.z, 9);
    
    m3d::vec3 sub = v2 - v1;
    ASSERT_NEAR(sub.x, 3); ASSERT_NEAR(sub.y, 3); ASSERT_NEAR(sub.z, 3);
    
    m3d::vec3 scalar_mult = v1 * 2.0;
    ASSERT_NEAR(scalar_mult.x, 2); ASSERT_NEAR(scalar_mult.z, 6);
}

TEST(vec3_dot_cross) {
    m3d::vec3 x(1, 0, 0);
    m3d::vec3 y(0, 1, 0);
    
    ASSERT_NEAR(m3d::dot(x, y), 0.0);
    ASSERT_NEAR(m3d::dot(x, x), 1.0);
    
    m3d::vec3 z = m3d::cross(x, y);
    ASSERT_NEAR(z.x, 0); ASSERT_NEAR(z.y, 0); ASSERT_NEAR(z.z, 1);
    
    // Cross product anti-commutativity
    m3d::vec3 neg_z = m3d::cross(y, x);
    ASSERT_NEAR(neg_z.z, -1.0);
}

TEST(vec3_length_normalize) {
    m3d::vec3 v(3, 4, 0);
    ASSERT_NEAR(m3d::length_sq(v), 25.0);
    ASSERT_NEAR(m3d::length(v), 5.0);
    
    m3d::vec3 n = m3d::normalize(v);
    ASSERT_NEAR(m3d::length(n), 1.0);
    ASSERT_NEAR(n.x, 0.6);
    ASSERT_NEAR(n.y, 0.8);
    
    // Safety check for zero vector
    m3d::vec3 zero(0, 0, 0);
    m3d::vec3 n_zero = m3d::normalize(zero);
    ASSERT_NEAR(n_zero.x, 0); // Should not crash
}

TEST_SUITE(
    RUN_TEST(vec3_basics),
    RUN_TEST(vec3_dot_cross),
    RUN_TEST(vec3_length_normalize)
)