#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

namespace test
{
    struct TestCase
    {
        std::string name;
        void (*func)();
    };

    template<typename T1, typename T2, typename T3 = double>
    inline bool near(T1 a, T2 b, T3 precision = 1e-6)
    {
        return std::abs(static_cast<double>(a) - static_cast<double>(b)) < static_cast<double>(precision);
    }

    // Cross-domain math constants — used by quaternion/rotation tests that
    // hard-coded 1.57079632679 in 3+ files.
    constexpr double kPi        = 3.14159265358979323846;
    constexpr double kHalfPi    = kPi / 2.0;
    constexpr double kQuarterPi = kPi / 4.0;
}

#define TEST(name) static void test_##name()
#define RUN_TEST(name) {#name, test_##name}

#define ASSERT_EQ(a, b)                                                                                         \
    if (!(a == b))                                                                                              \
    {                                                                                                           \
        std::cerr << "\n  [FAIL] Expected Equality: " << #a << " == " << #b << " at line " << __LINE__ << "\n"; \
        throw std::runtime_error("test failed");                                                                \
    }

#define ASSERT_APPROX(a, b, ...)                                                              \
    if (!(a.is_approx(b, ##__VA_ARGS__)))                                                     \
    {                                                                                         \
        std::cerr << "\n  [FAIL] Expected Approx Equality: " << #a << " approx " << #b        \
                  << " at line " << __LINE__ << "\n";                                         \
        throw std::runtime_error("test failed");                                              \
    }

#define ASSERT_TRUE(cond)                                                    \
    if (!(cond))                                                             \
    {                                                                        \
        std::cerr << "  FAIL: " << #cond << " at line " << __LINE__ << "\n"; \
        throw std::runtime_error("test failed");                             \
    }

#define ASSERT_FALSE(cond)                                                                   \
    if ((cond))                                                                              \
    {                                                                                        \
        std::cerr << "  FAIL: " << #cond << " at line " << __LINE__ << " should be false\n"; \
        throw std::runtime_error("test failed");                                             \
    }


#define ASSERT_NEAR(a, b, ...)                                                                \
    if (!test::near(a, b, ##__VA_ARGS__))                                               \
    {                                                                                         \
        std::cerr << "  FAIL: " << a << " not near " << b << " at line " << __LINE__ << "\n"; \
        throw std::runtime_error("test failed");                                              \
    }

// Component-wise vector-near. Captures __LINE__ at call site so failures
// point at the test, not the macro body.
#define ASSERT_VEC_NEAR(va, vb, ...)                                                                  \
    do {                                                                                              \
        const auto &_a = (va); const auto &_b = (vb);                                                 \
        if (!test::near(_a.x, _b.x, ##__VA_ARGS__) ||                                                 \
            !test::near(_a.y, _b.y, ##__VA_ARGS__) ||                                                 \
            !test::near(_a.z, _b.z, ##__VA_ARGS__))                                                   \
        {                                                                                             \
            std::cerr << "  FAIL: vec " << #va << " not near " << #vb                                 \
                      << " at line " << __LINE__ << "\n"                                              \
                      << "        got    (" << _a.x << ", " << _a.y << ", " << _a.z << ")\n"          \
                      << "        wanted (" << _b.x << ", " << _b.y << ", " << _b.z << ")\n";         \
            throw std::runtime_error("test failed");                                                  \
        }                                                                                             \
    } while (0)

// Component-wise quaternion-near (w, x, y, z).
#define ASSERT_QUAT_NEAR(qa, qb, ...)                                                                 \
    do {                                                                                              \
        const auto &_a = (qa); const auto &_b = (qb);                                                 \
        if (!test::near(_a.w, _b.w, ##__VA_ARGS__) ||                                                 \
            !test::near(_a.x, _b.x, ##__VA_ARGS__) ||                                                 \
            !test::near(_a.y, _b.y, ##__VA_ARGS__) ||                                                 \
            !test::near(_a.z, _b.z, ##__VA_ARGS__))                                                   \
        {                                                                                             \
            std::cerr << "  FAIL: quat " << #qa << " not near " << #qb                                \
                      << " at line " << __LINE__ << "\n";                                             \
            throw std::runtime_error("test failed");                                                  \
        }                                                                                             \
    } while (0)

// Asserts a vec3-shaped value is unit length within 0.001.
#define ASSERT_NORMAL_UNIT(v)                                                                         \
    do {                                                                                              \
        const auto _len = m3d::length((v));                                                           \
        if (!test::near(_len, 1.0, 0.001)) {                                                          \
            std::cerr << "  FAIL: " << #v << " is not unit length (got " << _len                      \
                      << ") at line " << __LINE__ << "\n";                                            \
            throw std::runtime_error("test failed");                                                  \
        }                                                                                             \
    } while (0)

#define TEST_SUITE(...)                                                                                                    \
    int main()                                                                                                             \
    {                                                                                                                      \
        test::TestCase tests[] = {__VA_ARGS__};                                                                       \
        int failed = 0;                                                                                                    \
        for (auto &t : tests)                                                                                              \
        {                                                                                                                  \
            std::cout << "[RUN] " << t.name << "..." << std::flush;                                                        \
            try                                                                                                            \
            {                                                                                                              \
                t.func();                                                                                                  \
                std::cout << " PASS\n";                                                                                    \
            }                                                                                                              \
            catch (...)                                                                                                    \
            {                                                                                                              \
                std::cout << " FAILED\n";                                                                                  \
                failed++;                                                                                                  \
            }                                                                                                              \
        }                                                                                                                  \
        std::cout << "\nResult: " << (sizeof(tests) / sizeof(tests[0]) - failed) << " passed, " << failed << " failed.\n"; \
        return failed > 0;                                                                                                 \
    }
