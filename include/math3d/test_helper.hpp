#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

namespace m3d::test
{
    struct TestCase
    {
        std::string name;
        void (*func)();
    };

    inline bool near(double a, double b, double eps = 1e-6)
    {
        return std::abs(a - b) < eps;
    }
}

#define TEST(name) static void test_##name()
#define RUN_TEST(name) {#name, test_##name}

#define ASSERT_EQ(a, b)                                                                                         \
    if (!(a == b))                                                                                              \
    {                                                                                                           \
        std::cerr << "\n  [FAIL] Expected Equality: " << #a << " == " << #b << " at line " << __LINE__ << "\n"; \
        throw std::runtime_error("test failed");                                                                \
    }

#define ASSERT_APPROX(a, b)                                                                                                \
    if (!(a.is_approx(b)))                                                                                                 \
    {                                                                                                                      \
        std::cerr << "\n  [FAIL] Expected Approx Equality: " << #a << " approx " << #b << " at line " << __LINE__ << "\n"; \
        throw std::runtime_error("test failed");                                                                           \
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

#define ASSERT_NEAR(a, b)                                                                     \
    if (!m3d::test::near(a, b))                                                               \
    {                                                                                         \
        std::cerr << "  FAIL: " << a << " not near " << b << " at line " << __LINE__ << "\n"; \
        throw std::runtime_error("test failed");                                              \
    }

#define TEST_SUITE(...)                                                                                                    \
    int main()                                                                                                             \
    {                                                                                                                      \
        m3d::test::TestCase tests[] = {__VA_ARGS__};                                                                       \
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
