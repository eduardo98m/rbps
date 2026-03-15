#pragma once
// ============================================================================
//  tests/visr/visr_test_helper.hpp
//
//  Minimal assert framework + world-building helpers shared across visr tests.
//
//  ADAPT: If BodyParams / ColliderParams field names differ in your codebase,
//         update make_dynamic_body() and make_box_collider() below.
//         All visr logic being tested is independent of these exact names.
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>

// ── Assertion macros ──────────────────────────────────────────────────────────

#define VISR_ASSERT(expr) \
    do { if (!(expr)) { \
        std::fprintf(stderr, "FAIL  %s:%d  %s\n", __FILE__, __LINE__, #expr); \
        std::exit(1); \
    } } while(0)

#define VISR_ASSERT_MSG(expr, msg) \
    do { if (!(expr)) { \
        std::fprintf(stderr, "FAIL  %s:%d  %s  (%s)\n", __FILE__, __LINE__, #expr, msg); \
        std::exit(1); \
    } } while(0)

#define VISR_APPROX_EQ(a, b, eps) \
    VISR_ASSERT(std::fabs((double)(a) - (double)(b)) < (eps))

#define VISR_TEST(name) \
    std::printf("  %-50s", name); std::fflush(stdout);

#define VISR_PASS() \
    std::printf("PASS\n");

// ── Physics includes (must come after raylib guard if also including visr) ────
#include "rbps/API/World.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/JointAPI.hpp"

// ── World-building helpers ────────────────────────────────────────────────────

// Creates a simple dynamic box body at `pos` with mass 1.
// ADAPT: adjust BodyParams field names to match your BodyAPI.hpp
inline uint32_t make_dynamic_body(rbps::World &w,
                                   m3d::vec3 pos = {0, 0, 0},
                                   m3d::scalar mass = 1.0)
{
    rbps::BodyParams bp{};
    bp.type     = rbps::BodyType::DYNAMIC;
    bp.position = pos;
    bp.mass     = mass;
    return w.create_body(bp);
}

// Creates a static body (infinite mass, never moved).
inline uint32_t make_static_body(rbps::World &w,
                                  m3d::vec3 pos = {0, -5, 0})
{
    rbps::BodyParams bp{};
    bp.type     = rbps::BodyType::STATIC;
    bp.position = pos;
    bp.mass     = 0.0;
    return w.create_body(bp);
}

// Attaches a box collider to `body_id`.
// ADAPT: adjust ColliderParams field names to match your ColliderAPI.hpp
inline uint32_t make_box_collider(rbps::World &w,
                                   uint32_t body_id,
                                   m3d::vec3 half_extents = {0.5, 0.5, 0.5})
{
    rbps::ColliderParams cp{};
    cp.body_id          = body_id;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{0, 0, 0, 1};
    cp.shape            = rbc::Shape(rbc::Box{half_extents});
    cp.restitution      = 0.3f;
    cp.static_friction  = 0.5f;
    cp.dynamic_friction = 0.3f;
    return w.create_collider(cp);
}

inline uint32_t make_sphere_collider(rbps::World &w,
                                      uint32_t body_id,
                                      m3d::scalar radius = 0.5)
{
    rbps::ColliderParams cp{};
    cp.body_id          = body_id;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{0, 0, 0, 1};
    cp.shape            = rbc::Shape(rbc::Sphere{radius});
    cp.restitution      = 0.3f;
    cp.static_friction  = 0.5f;
    cp.dynamic_friction = 0.3f;
    return w.create_collider(cp);
}