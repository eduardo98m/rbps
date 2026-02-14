#include "math3d/vec3.hpp"
#include "tests/test_helper.hpp"

using namespace m3d;

TEST(vec3_constructors) {
    vec3 v1; // Default
    ASSERT_EQ(v1.x, 0.0);
    ASSERT_EQ(v1.y, 0.0);
    ASSERT_EQ(v1.z, 0.0);

    vec3 v2(1.0, 2.0, 3.0); // Parameterized
    ASSERT_EQ(v2.x, 1.0);
    ASSERT_EQ(v2.y, 2.0);
    ASSERT_EQ(v2.z, 3.0);
}

TEST(vec3_basic_operators) {
    vec3 a(1, 2, 3);
    vec3 b(4, 5, 6);

    // Addition & Subtraction
    ASSERT_EQ(a + b, vec3(5, 7, 9));
    ASSERT_EQ(b - a, vec3(3, 3, 3));

    // Multiplication & Division (Right side)
    ASSERT_EQ(a * 2.0, vec3(2, 4, 6));
    ASSERT_EQ(vec3(10, 20, 30) / 10.0, vec3(1, 2, 3));

    // Scalar Multiplication (Left side)
    ASSERT_EQ(3.0 * a, vec3(3, 6, 9));

    // Unary Negation
    ASSERT_EQ(-a, vec3(-1, -2, -3));
}

TEST(vec3_inplace_operators) {
    vec3 v(1, 1, 1);
    
    v += vec3(1, 2, 3);
    ASSERT_EQ(v, vec3(2, 3, 4));

    v -= vec3(1, 1, 1);
    ASSERT_EQ(v, vec3(1, 2, 3));

    v *= 2.0;
    ASSERT_EQ(v, vec3(2, 4, 6));
}

TEST(vec3_equality_and_approx) {
    vec3 a(1.0, 1.0, 1.0);
    vec3 b(1.0, 1.0, 1.0);
    vec3 c(1.000000000001, 1.0, 1.0);

    ASSERT_TRUE(a == b);
    ASSERT_TRUE(a != c);
    
    // Physics-safe check
    ASSERT_APPROX(a, c); 
    ASSERT_FALSE(a.is_approx(vec3(2, 2, 2)));
}

TEST(vec3_indexing) {
    vec3 v(10, 20, 30);
    
    // Mutable access
    v[0] = 1.1;
    v[1] = 2.2;
    v[2] = 3.3;
    ASSERT_EQ(v.x, 1.1);
    ASSERT_EQ(v.y, 2.2);
    ASSERT_EQ(v.z, 3.3);

    // Const access
    const vec3 cv(7, 8, 9);
    ASSERT_EQ(cv[0], 7.0);
    ASSERT_EQ(cv[1], 8.0);
    ASSERT_EQ(cv[2], 9.0);
}

TEST(vec3_math_functions) {
    vec3 a(1, 0, 0);
    vec3 b(0, 1, 0);

    // Dot product
    ASSERT_NEAR(dot(a, b), 0.0);
    ASSERT_NEAR(dot(a, a), 1.0);

    // Cross product
    ASSERT_EQ(cross(a, b), vec3(0, 0, 1));
    ASSERT_EQ(cross(b, a), vec3(0, 0, -1));

    // Magnitude
    vec3 v(3, 4, 0);
    ASSERT_NEAR(length_sq(v), 25.0);
    ASSERT_NEAR(length(v), 5.0);
}

TEST(vec3_normalization) {
    // Normal case
    vec3 v(10, 0, 0);
    ASSERT_EQ(normalize(v), vec3(1, 0, 0));

    // Edge case: Zero vector
    vec3 zero(0, 0, 0);
    vec3 n_zero = normalize(zero);
    ASSERT_EQ(n_zero, vec3(0, 0, 0)); // Safety check line
}

TEST_SUITE(
    RUN_TEST(vec3_constructors),
    RUN_TEST(vec3_basic_operators),
    RUN_TEST(vec3_inplace_operators),
    RUN_TEST(vec3_equality_and_approx),
    RUN_TEST(vec3_indexing),
    RUN_TEST(vec3_math_functions),
    RUN_TEST(vec3_normalization)
)