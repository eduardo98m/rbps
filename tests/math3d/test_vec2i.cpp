#include "math3d/vec2i.hpp"
#include "tests/test_helper.hpp"
#include <sstream>

using namespace m3d;

// ------------------------------------------------------------
// Constructors
// ------------------------------------------------------------

TEST(vec2i_constructors) {
    vec2i v1;
    ASSERT_EQ(v1.x, 0);
    ASSERT_EQ(v1.y, 0);

    vec2i v2(3, 4);
    ASSERT_EQ(v2.x, 3);
    ASSERT_EQ(v2.y, 4);

    vec2i v3(5);
    ASSERT_EQ(v3.x, 5);
    ASSERT_EQ(v3.y, 5);
}

// ------------------------------------------------------------
// Basic Operators
// ------------------------------------------------------------

TEST(vec2i_basic_operators) {
    vec2i a(2, 3);
    vec2i b(5, 7);

    // Addition
    ASSERT_EQ(a + b, vec2i(7, 10));

    // Subtraction
    ASSERT_EQ(b - a, vec2i(3, 4));

    // Scalar multiplication (right)
    ASSERT_EQ(a * 3, vec2i(6, 9));

    // Scalar multiplication (left)
    ASSERT_EQ(4 * a, vec2i(8, 12));

    // Division
    ASSERT_EQ(vec2i(8, 6) / 2, vec2i(4, 3));

    // Unary negation
    ASSERT_EQ(-a, vec2i(-2, -3));
}

// ------------------------------------------------------------
// In-place Operators
// ------------------------------------------------------------

TEST(vec2i_inplace_operators) {
    vec2i v(1, 2);

    v += vec2i(3, 4);
    ASSERT_EQ(v, vec2i(4, 6));

    v -= vec2i(1, 1);
    ASSERT_EQ(v, vec2i(3, 5));

    v *= 2;
    ASSERT_EQ(v, vec2i(6, 10));
}

// ------------------------------------------------------------
// Equality / Inequality
// ------------------------------------------------------------

TEST(vec2i_equality) {
    vec2i a(1, 2);
    vec2i b(1, 2);
    vec2i c(2, 1);

    ASSERT_TRUE(a == b);
    ASSERT_FALSE(a != b);

    ASSERT_TRUE(a != c);
    ASSERT_FALSE(a == c);
}

// ------------------------------------------------------------
// Indexing
// ------------------------------------------------------------

TEST(vec2i_indexing) {
    vec2i v(10, 20);

    // Mutable access
    v[0] = 100;
    v[1] = 200;
    ASSERT_EQ(v.x, 100);
    ASSERT_EQ(v.y, 200);

    // Const access
    const vec2i cv(7, 9);
    ASSERT_EQ(cv[0], 7);
    ASSERT_EQ(cv[1], 9);
}

// ------------------------------------------------------------
// Math Functions
// ------------------------------------------------------------

TEST(vec2i_math_functions) {
    vec2i a(1, 0);
    vec2i b(0, 1);

    // Dot product
    ASSERT_EQ(dot(a, b), 0);
    ASSERT_EQ(dot(a, a), 1);

    vec2i c(3, 4);
    ASSERT_EQ(dot(c, c), 25);

    // Cross product (2D scalar result)
    ASSERT_EQ(cross(a, b), 1);   // CCW
    ASSERT_EQ(cross(b, a), -1);  // CW

    vec2i d(2, 3);
    vec2i e(4, 5);
    ASSERT_EQ(cross(d, e), (2 * 5 - 3 * 4));
}

// ------------------------------------------------------------
// Integer Division Behavior
// ------------------------------------------------------------

TEST(vec2i_integer_division_behavior) {
    vec2i v(7, 5);

    // Integer truncation check
    ASSERT_EQ(v / 2, vec2i(3, 2));
}

// ------------------------------------------------------------
// Stream Operator
// ------------------------------------------------------------

TEST(vec2i_stream_operator) {
    vec2i v(3, 4);
    std::ostringstream oss;
    oss << v;

    ASSERT_TRUE(oss.str() == "vec2i(3, 4)");
}

// ------------------------------------------------------------

TEST_SUITE(
    RUN_TEST(vec2i_constructors),
    RUN_TEST(vec2i_basic_operators),
    RUN_TEST(vec2i_inplace_operators),
    RUN_TEST(vec2i_equality),
    RUN_TEST(vec2i_indexing),
    RUN_TEST(vec2i_math_functions),
    RUN_TEST(vec2i_integer_division_behavior),
    RUN_TEST(vec2i_stream_operator)
)