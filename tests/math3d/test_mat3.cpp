#include "math3d/mat3.hpp"
#include "math3d/test_helper.hpp"

TEST(identity_and_indexing)
{
    m3d::mat3 I;
    ASSERT_NEAR(I[0].x, 1.0);
    ASSERT_NEAR(I[1].y, 1.0);
    ASSERT_NEAR(I[0].y, 0.0);
}

TEST(matrix_vector_mult)
{
    m3d::mat3 m(1, 2, 3,
                4, 5, 6,
                7, 8, 9);
    m3d::vec3 v(1, 1, 1);
    m3d::vec3 res = m * v;
    // Row 0: 1*1 + 2*1 + 3*1 = 6
    ASSERT_NEAR(res.x, 6.0);
    ASSERT_NEAR(res.y, 15.0);
    ASSERT_NEAR(res.z, 24.0);
}

TEST(determinant_and_inverse)
{
    // A non-singular matrix
    m3d::mat3 m(2, 0, 0,
                0, 2, 0,
                0, 0, 2);
    ASSERT_NEAR(m.determinant(), 8.0);

    m3d::mat3 inv = m3d::mat3::inverse(m);
    ASSERT_NEAR(inv[0].x, 0.5);
    ASSERT_NEAR(inv[1].y, 0.5);
}

TEST(symmetric_matrix)
{
    m3d::SymMat3 s(2, 2, 2, 0, 0, 0); // Diagonal
    m3d::vec3 v(1, 2, 3);
    m3d::vec3 res = s * v;
    ASSERT_NEAR(res.x, 2.0);
    ASSERT_NEAR(res.y, 4.0);
    ASSERT_NEAR(res.z, 6.0);
}

TEST(mat3_multiplication)
{
    // Identity check
    m3d::mat3 A(1, 2, 3, 4, 5, 6, 7, 8, 9);
    m3d::mat3 I;
    m3d::mat3 res = A * I;
    ASSERT_NEAR(res[0].x, 1);
    ASSERT_NEAR(res[2].z, 9);

    // Matrix-Matrix
    m3d::mat3 B = A * A;
    // Calculation: Row0 * Col0 = 1*1 + 2*4 + 3*7 = 1 + 8 + 21 = 30
    ASSERT_NEAR(B[0].x, 30.0);
}

TEST(mat3_transpose)
{
    m3d::mat3 m(1, 2, 3,
                4, 5, 6,
                7, 8, 9);
    m3d::mat3 t = m3d::mat3::transpose(m);
    ASSERT_NEAR(t[1].x, 4); // Original m[0].y
    ASSERT_NEAR(t[0].y, 2); // Original m[1].x
}

TEST(mat3_inverse_complex)
{
    // Rotation matrix (90 deg around Z)
    m3d::quat q = m3d::quat::from_rpy(0, 0, 1.57079632679);
    m3d::mat3 R = m3d::mat3_cast(q);

    m3d::mat3 invR = m3d::mat3::inverse(R);
    m3d::mat3 identity = R * invR;

    ASSERT_NEAR(identity[0].x, 1.0);
    ASSERT_NEAR(identity[1].y, 1.0);
    ASSERT_NEAR(identity[2].z, 1.0);
    ASSERT_NEAR(identity[0].y, 0.0);
}

TEST_SUITE(
    RUN_TEST(identity_and_indexing),
    RUN_TEST(matrix_vector_mult),
    RUN_TEST(determinant_and_inverse),
    RUN_TEST(symmetric_matrix),
    RUN_TEST(mat3_multiplication),
    RUN_TEST(mat3_transpose),
    RUN_TEST(mat3_inverse_complex))