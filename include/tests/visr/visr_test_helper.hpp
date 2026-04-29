#pragma once
// ============================================================================
//  tests/visr/visr_test_helper.hpp
//
//  World-building helpers shared across visr tests.
//  Assertions come from tests/test_helper.hpp — include that separately.
// ============================================================================

#include "rbps/API/World.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/JointAPI.hpp"

// Stable ID of the first body in the world's body collection. Encapsulates
// the `_ids[0]` pattern used at many sites in test_debug_channel.cpp.
inline uint32_t get_first_body_id(const rbps::World &w)
{
    return w.bodies._ids[0];
}

// Creates a dynamic body at `pos` with given mass.
inline uint32_t make_dynamic_body(rbps::World &w,
                                   m3d::vec3 pos  = {0, 0, 0},
                                   m3d::scalar mass = 1.0)
{
    rbps::BodyParams bp{};
    bp.type     = rbps::BodyType::DYNAMIC;
    bp.position = pos;
    bp.mass     = mass;
    return w.create_body(bp);
}

// Creates a static body at `pos`.
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
inline uint32_t make_box_collider(rbps::World &w,
                                   uint32_t body_id,
                                   m3d::vec3 half_extents = {0.5, 0.5, 0.5})
{
    rbps::ColliderParams cp{};
    cp.body_id          = body_id;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{1, 0, 0, 0};
    cp.shape            = rbc::Shape(rbc::Box{half_extents});
    cp.restitution      = 0.3;
    cp.static_friction  = 0.5;
    cp.dynamic_friction = 0.3;
    return w.create_collider(cp);
}

// Attaches a sphere collider to `body_id`.
inline uint32_t make_sphere_collider(rbps::World &w,
                                      uint32_t body_id,
                                      m3d::scalar radius = 0.5)
{
    rbps::ColliderParams cp{};
    cp.body_id          = body_id;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{1, 0, 0, 0};
    cp.shape            = rbc::Shape(rbc::Sphere{radius});
    cp.restitution      = 0.3;
    cp.static_friction  = 0.5;
    cp.dynamic_friction = 0.3;
    return w.create_collider(cp);
}