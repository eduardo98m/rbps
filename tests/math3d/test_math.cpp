#include "math3d/vec3.hpp"
#include "math3d/quat.hpp"
#include <cassert>
#include <iostream>

void test_vec3() {
    m3d::vec3 a(1, 0, 0);
    m3d::vec3 b(0, 1, 0);
    assert(m3d::dot(a, b) == 0);
    std::cout << "Vec3 tests passed! " << a << std::endl;
}

void test_quat_rpy() {
    // 90 degrees yaw (PI/2)
    auto q = m3d::quat::from_rpy(0, 0, 1.57079632679);
    // Should be roughly w: 0.707, z: 0.707
    assert(std::abs(q.w - 0.70710678118) < 0.001);
    std::cout << "Quat RPY tests passed! " << q << std::endl;
}

int main() {
    test_vec3();
    test_quat_rpy();
    std::cout << "All tests passed successfully!" << std::endl;
    return 0;
}