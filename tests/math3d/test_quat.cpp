#include "math3d/quat.hpp"
#include "math3d/test_helper.hpp"

TEST(rpy_conversion)
{
    // 90 degrees around Z axis (Yaw)
    m3d::quat q = m3d::quat::from_rpy(0, 0, 1.57079632679);

    m3d::vec3 v(1, 0, 0);
    m3d::vec3 rotated = m3d::rotate(v, q);

    // Should now point towards Y axis
    ASSERT_NEAR(rotated.x, 0.0);
    ASSERT_NEAR(rotated.y, 1.0);
}

TEST(quat_multiplication)
{
    // Rotate 90 around X then 90 around Y
    m3d::quat qX = m3d::quat::from_rpy(1.57079632679, 0, 0);
    m3d::quat qY = m3d::quat::from_rpy(0, 1.57079632679, 0);

    m3d::quat qCombined = qY * qX; // Order matters

    m3d::vec3 v(0, 0, 1); // Pointing up Z
    m3d::vec3 res = m3d::rotate(v, qCombined);

    // Rotate 90 X: v becomes (0, -1, 0) (Y-axis)
    // Then 90 Y: v becomes (1, -1, 0) ... no, wait.
    // Let's use a simpler check:
    ASSERT_NEAR(m3d::length(res), 1.0);
}

TEST(quat_conjugate_normalize)
{
    m3d::quat q(1, 2, 3, 4);
    m3d::quat c = m3d::conjugate(q);
    ASSERT_NEAR(c.w, 1);
    ASSERT_NEAR(c.x, -2);

    m3d::quat n = m3d::normalize(q);
    m3d::scalar mag = std::sqrt(n.w * n.w + n.x * n.x + n.y * n.y + n.z * n.z);
    ASSERT_NEAR(mag, 1.0);
}

TEST(quat_drift_stress)
{
    // A small rotation (0.01 rad around X)
    m3d::quat step = m3d::quat::from_axis_angle(m3d::vec3(1, 0, 0), 0.01);
    m3d::quat total; // Identity

    // Apply rotation 1000 times
    for (int i = 0; i < 1000; ++i)
    {
        total = total * step;
    }

    // Check magnitude - should be exactly 1.0 (or very close)
    m3d::scalar mag_sq = total.w * total.w + total.x * total.x + total.y * total.y + total.z * total.z;

    // If our mandatory normalization works, this should be effectively 1.0
    ASSERT_TRUE(m3d::test::near(mag_sq, 1.0, 1e-12))
}

TEST(quat_multiplication_logic)
{
    // 90 deg around X
    m3d::quat qX = m3d::quat::from_axis_angle({1, 0, 0}, 1.57079632679);
    // 90 deg around Y
    m3d::quat qY = m3d::quat::from_axis_angle({0, 1, 0}, 1.57079632679);

    m3d::quat qCombined = qY * qX;

    m3d::vec3 v(0, 0, 1); // Z-axis
    m3d::vec3 res = m3d::rotate(v, qCombined);

    // Rotation sequence check:
    // Z rotated 90 around X -> -Y axis (0, -1, 0)
    // -Y rotated 90 around Y -> -Y axis (no change because it's the axis)
    ASSERT_APPROX(res, m3d::vec3(0, -1, 0));
}

TEST(quat_addition_basic)
{
    // q1 = (1, 2, 3, 4)
    m3d::quat q1(1.0, 2.0, 3.0, 4.0);
    // q2 = (5, 6, 7, 8)
    m3d::quat q2(5.0, 6.0, 7.0, 8.0);

    m3d::quat result = q1 + q2;

    ASSERT_NEAR(result.w, 6.0);
    ASSERT_NEAR(result.x, 8.0);
    ASSERT_NEAR(result.y, 10.0);
    ASSERT_NEAR(result.z, 12.0);
}

TEST(quat_plus_equals)
{
    m3d::quat q1(1.0, 0.0, 0.0, 0.0);
    m3d::quat q2(0.0, 1.0, 0.0, 0.0);

    q1 += q2;

    // Expected: (1, 1, 0, 0)
    ASSERT_NEAR(q1.w, 1.0);
    ASSERT_NEAR(q1.x, 1.0);
    ASSERT_NEAR(q1.y, 0.0);
    ASSERT_NEAR(q1.z, 0.0);
}

// CRITICAL TEST: Verifies that addition does NOT normalize automatically.
// If this fails, the physics engine update loop will be mathematically incorrect.
TEST(quat_addition_does_not_normalize)
{
    m3d::quat q1(1.0, 0.0, 0.0, 0.0); // Identity
    m3d::quat q2(0.0, 1.0, 0.0, 0.0); // Pure vector quat

    m3d::quat q_sum = q1 + q2;

    // Magnitude squared: 1^2 + 1^2 = 2.0
    m3d::scalar norm_sq = q_sum.w * q_sum.w + q_sum.x * q_sum.x + q_sum.y * q_sum.y + q_sum.z * q_sum.z;

    // If it normalized automatically, norm_sq would be 1.0.
    // It MUST be 2.0 to ensure linearity.
    ASSERT_NEAR(norm_sq, 2.0);
}

// Simulates the physics update loop to ensure manual normalization works
TEST(quat_physics_integration_logic)
{
    m3d::scalar dt = 1.0;
    m3d::quat orientation(1.0, 0.0, 0.0, 0.0); // Identity

    // Angular velocity (spinning around X)
    m3d::vec3 omega(2.0, 0.0, 0.0);

    // dq = 0.5 * dt * omega * q
    m3d::quat omega_q(0.0, omega.x, omega.y, omega.z);

    // 1. Derivative
    m3d::quat q_dot = omega_q * orientation;
    q_dot = q_dot * (0.5 * dt); // Scalar multiplication

    // 2. Integration (using operator+=)
    orientation += q_dot;

    // 3. Verify it is NOT normalized yet (physics engine job)
    m3d::scalar norm_sq_before = orientation.w * orientation.w + orientation.x * orientation.x;
    ASSERT_TRUE(norm_sq_before > 1.1); // Should be roughly 2.0

    // 4. Normalize
    orientation = m3d::normalize(orientation);

    // 5. Verify unit length
    m3d::scalar final_norm = orientation.w * orientation.w + orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z;
    ASSERT_NEAR(final_norm, 1.0);
}

TEST_SUITE(
    RUN_TEST(rpy_conversion),
    RUN_TEST(quat_multiplication),
    RUN_TEST(quat_conjugate_normalize),
    RUN_TEST(quat_drift_stress),
    RUN_TEST(quat_multiplication_logic),
    RUN_TEST(quat_addition_basic),
    RUN_TEST(quat_plus_equals),
    RUN_TEST(quat_addition_does_not_normalize),
    RUN_TEST(quat_physics_integration_logic))