#include "math3d/mat3.hpp"
#include "tests/test_helper.hpp"

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

    m3d::mat3 inv = m.inverse();
    ASSERT_NEAR(inv[0].x, 0.5);
    ASSERT_NEAR(inv[1].y, 0.5);
}

TEST(symmetric_matrix)
{
    m3d::smat3 s(2, 2, 2, 0, 0, 0); // Diagonal
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
    m3d::mat3 t = m.transpose();
    ASSERT_NEAR(t[1].x, 4); // Original m[0].y
    ASSERT_NEAR(t[0].y, 2); // Original m[1].x
}

TEST(mat3_inverse_complex)
{
    // Rotation matrix (90 deg around Z)
    m3d::quat q = m3d::quat::from_rpy(0, 0, 1.57079632679);
    m3d::mat3 R = m3d::mat3_cast(q);

    m3d::mat3 invR = R.inverse();
    m3d::mat3 identity = R * invR;

    ASSERT_NEAR(identity[0].x, 1.0);
    ASSERT_NEAR(identity[1].y, 1.0);
    ASSERT_NEAR(identity[2].z, 1.0);
    ASSERT_NEAR(identity[0].y, 0.0);
}

TEST(mat3_equality_operator)
{
    m3d::mat3 A(1, 2, 3, 4, 5, 6, 7, 8, 9);
    m3d::mat3 B(1, 2, 3, 4, 5, 6, 7, 8, 9);
    m3d::mat3 C(1, 2, 3, 4, 5, 6, 7, 8, 10);
    
    ASSERT_TRUE(A == B);
    ASSERT_FALSE(A == C);
}

TEST(mat3_is_approx)
{
    m3d::mat3 A(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    m3d::mat3 B(1.0001, 2.0001, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    m3d::mat3 C(1.1, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    
    ASSERT_TRUE(A.is_approx(B, 0.001));
    ASSERT_FALSE(A.is_approx(C, 0.001));
}

TEST(mat3_inverse_singular)
{
    // Singular matrix (determinant = 0)
    m3d::mat3 singular(1, 2, 3,
                       2, 4, 6,
                       3, 6, 9);
    
    ASSERT_NEAR(singular.determinant(), 0.0);
    m3d::mat3 inv = singular.inverse();
    // Should return identity on failure
    ASSERT_NEAR(inv[0].x, 1.0);
    ASSERT_NEAR(inv[1].y, 1.0);
    ASSERT_NEAR(inv[2].z, 1.0);
}

TEST(smat3_with_off_diagonal)
{
    // Symmetric matrix with off-diagonal elements
    m3d::smat3 s(1, 2, 3,  // diagonal: xx, yy, zz
                 0.5, 0.7, 0.9);  // off-diagonal: xy, xz, yz
    
    m3d::vec3 v(1, 0, 0);
    m3d::vec3 res = s * v;
    ASSERT_NEAR(res.x, 1.0);   // xx * 1
    ASSERT_NEAR(res.y, 0.5);   // xy * 1
    ASSERT_NEAR(res.z, 0.7);   // xz * 1
}

TEST(smat3_scalar_multiplication)
{
    m3d::smat3 s(2, 3, 4, 1, 1.5, 2);
    m3d::smat3 scaled = s * 2.0;
    
    ASSERT_NEAR(scaled.xx, 4.0);
    ASSERT_NEAR(scaled.yy, 6.0);
    ASSERT_NEAR(scaled.zz, 8.0);
    ASSERT_NEAR(scaled.xy, 2.0);
    ASSERT_NEAR(scaled.xz, 3.0);
    ASSERT_NEAR(scaled.yz, 4.0);
    
    // Test commutative scalar multiplication
    m3d::smat3 scaled2 = 2.0 * s;
    ASSERT_NEAR(scaled2.xx, 4.0);
    ASSERT_NEAR(scaled2.yy, 6.0);
}

TEST(smat3_is_approx)
{
    m3d::smat3 s1(1, 2, 3, 0.5, 0.7, 0.9);
    m3d::smat3 s2(1.0001, 2.0001, 3.0, 0.5, 0.7, 0.9);
    m3d::smat3 s3(1.1, 2, 3, 0.5, 0.7, 0.9);
    
    ASSERT_TRUE(s1.is_approx(s2, 0.001));
    ASSERT_FALSE(s1.is_approx(s3, 0.001));
}

TEST(mat3_times_smat3)
{
    m3d::mat3 R(1, 0, 0,
                0, 1, 0,
                0, 0, 1);  // Identity
    m3d::smat3 s(2, 3, 4, 0.5, 0.7, 0.9);
    
    m3d::mat3 result = R * s;
    
    // Identity * s should give s back as mat3
    ASSERT_NEAR(result[0].x, 2.0);   // xx
    ASSERT_NEAR(result[1].y, 3.0);   // yy
    ASSERT_NEAR(result[2].z, 4.0);   // zz
    ASSERT_NEAR(result[1].x, 0.5);   // xy
}

TEST(smat3_from_mat3_conversion)
{
    m3d::mat3 m(1, 2, 3,
                4, 5, 6,
                7, 8, 9);
    
    m3d::smat3 s(m);
    
    // Diagonal elements
    ASSERT_NEAR(s.xx, 1.0);
    ASSERT_NEAR(s.yy, 5.0);
    ASSERT_NEAR(s.zz, 9.0);
    
    // Off-diagonal (averaged)
    ASSERT_NEAR(s.xy, (4.0 + 2.0) / 2.0);  // avg of m[1].x and m[0].y
    ASSERT_NEAR(s.xz, (7.0 + 3.0) / 2.0);  // avg of m[2].x and m[0].z
    ASSERT_NEAR(s.yz, (8.0 + 6.0) / 2.0);  // avg of m[2].y and m[1].z
}

TEST(smat3_from_symmetric_mat3)
{
    // Create a truly symmetric mat3
    m3d::mat3 m(2, 1, 3,
                1, 4, 5,
                3, 5, 6);
    
    m3d::smat3 s(m);
    
    ASSERT_NEAR(s.xx, 2.0);
    ASSERT_NEAR(s.yy, 4.0);
    ASSERT_NEAR(s.zz, 6.0);
    ASSERT_NEAR(s.xy, 1.0);
    ASSERT_NEAR(s.xz, 3.0);
    ASSERT_NEAR(s.yz, 5.0);
}

TEST(mat3_quat_cast_and_inverse)
{
    // Test that rotation matrices from quaternions are orthogonal
    m3d::quat q = m3d::quat::from_rpy(0.5, 0.3, 0.7);
    m3d::mat3 R = m3d::mat3_cast(q);
    
    // For rotation matrices: R^T = R^-1
    m3d::mat3 Rt = R.transpose();
    m3d::mat3 Rinv = R.inverse();
    
    ASSERT_TRUE(Rt.is_approx(Rinv, 0.0001));
}

TEST(smat3_default_constructor)
{
    m3d::smat3 s;
    ASSERT_NEAR(s.xx, 0.0);
    ASSERT_NEAR(s.yy, 0.0);
    ASSERT_NEAR(s.zz, 0.0);
    ASSERT_NEAR(s.xy, 0.0);
    ASSERT_NEAR(s.xz, 0.0);
    ASSERT_NEAR(s.yz, 0.0);
}

TEST(mat3_const_indexing)
{
    const m3d::mat3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    ASSERT_NEAR(m[0].x, 1.0);
    ASSERT_NEAR(m[1].y, 5.0);
    ASSERT_NEAR(m[2].z, 9.0);
}

TEST_SUITE(
    RUN_TEST(identity_and_indexing),
    RUN_TEST(matrix_vector_mult),
    RUN_TEST(determinant_and_inverse),
    RUN_TEST(symmetric_matrix),
    RUN_TEST(mat3_multiplication),
    RUN_TEST(mat3_transpose),
    RUN_TEST(mat3_inverse_complex),
    RUN_TEST(mat3_equality_operator),
    RUN_TEST(mat3_is_approx),
    RUN_TEST(mat3_inverse_singular),
    RUN_TEST(smat3_with_off_diagonal),
    RUN_TEST(smat3_scalar_multiplication),
    RUN_TEST(smat3_is_approx),
    RUN_TEST(mat3_times_smat3),
    RUN_TEST(smat3_from_mat3_conversion),
    RUN_TEST(smat3_from_symmetric_mat3),
    RUN_TEST(mat3_quat_cast_and_inverse),
    RUN_TEST(smat3_default_constructor),
    RUN_TEST(mat3_const_indexing)
)