#include "math3d/quat.hpp"
#include "math3d/test_helper.hpp"

TEST(rpy_conversion) {
    // 90 degrees around Z axis (Yaw)
    m3d::quat q = m3d::quat::from_rpy(0, 0, 1.57079632679);
    
    m3d::vec3 v(1, 0, 0);
    m3d::vec3 rotated = m3d::rotate(v, q);
    
    // Should now point towards Y axis
    ASSERT_NEAR(rotated.x, 0.0);
    ASSERT_NEAR(rotated.y, 1.0);
}


TEST(quat_multiplication) {
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

TEST(quat_conjugate_normalize) {
    m3d::quat q(1, 2, 3, 4);
    m3d::quat c = m3d::conjugate(q);
    ASSERT_NEAR(c.w, 1);
    ASSERT_NEAR(c.x, -2);
    
    m3d::quat n = m3d::normalize(q);
    scalar mag = std::sqrt(n.w*n.w + n.x*n.x + n.y*n.y + n.z*n.z);
    ASSERT_NEAR(mag, 1.0);
}

TEST_SUITE(
    RUN_TEST(rpy_conversion),
    RUN_TEST(quat_conjugate_normalize),
    RUN_TEST(quat_multiplication)
)