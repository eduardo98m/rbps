#pragma once
#include "rbps/CollisionPipeline.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/World.hpp"
#include "rbc/BroadPhase.hpp"
#include "tests/transform_helpers.hpp"   // test::tf_at

// ── Pipeline + World test helpers ───────────────────────────────────────────
// Two layers of factories:
//
//   1. Raw-collection layer (used by tests/rbps/test_collision_pipeline*).
//      add_dynamic_body / add_static_body return the slot index (not the ID)
//      because the existing tests use slot indices directly with the SoA.
//
//   2. World-API layer (used by tests/rbps/API/test_world). add_dynamic_sphere
//      / add_static_box bundle a body + collider creation through the public
//      rbps::World wrappers. Return the body ID.
//
// Mutation-style throughout: caller supplies the collection / world.
namespace test
{
    // ── Raw-collection layer ────────────────────────────────────────────────

    // Returns the SLOT INDEX of the newly-added body (not the stable ID).
    // Matches the existing helper signature in test_collision_pipeline.cpp.
    inline uint32_t add_dynamic_body(rbps::BodyCollection &bc,
                                     const m3d::vec3 &pos = m3d::vec3(0, 0, 0))
    {
        rbps::BodyParams p;
        p.type     = rbps::DYNAMIC;
        p.mass     = 1.0;
        p.position = pos;
        return bc.index_of(rbps::create_body(bc, p));
    }

    inline uint32_t add_static_body(rbps::BodyCollection &bc,
                                    const m3d::vec3 &pos = m3d::vec3(0, 0, 0))
    {
        rbps::BodyParams p;
        p.type     = rbps::STATIC;
        p.mass     = 1.0;
        p.position = pos;
        return bc.index_of(rbps::create_body(bc, p));
    }

    // Sphere collider bound to an existing body (`body_idx` is the body's slot
    // index). Local pos defaults to identity; the body's current world
    // position is reflected in the AABB the broad phase records.
    inline void add_sphere_collider(rbps::ColliderCollection &cc,
                                    rbc::BroadPhaseState     &bp,
                                    rbps::BodyCollection     &bc,
                                    uint32_t                  body_idx,
                                    m3d::scalar               radius,
                                    bool                      is_static = false)
    {
        rbps::ColliderParams p;
        p.shape     = rbc::Sphere(radius);
        p.body_id   = body_idx;
        p.is_static = is_static;
        m3d::tf init_tf;
        init_tf.pos = bc.position[body_idx];
        init_tf.rot = m3d::quat(1, 0, 0, 0);
        rbps::create_collider(cc, bp, p, init_tf);
    }

    // ── World-API layer ─────────────────────────────────────────────────────

    // Dynamic sphere: body + collider in one call. Returns the body ID.
    inline uint32_t add_dynamic_sphere(rbps::World &w,
                                       const m3d::vec3 &pos,
                                       m3d::scalar      radius = 0.5,
                                       m3d::scalar      mass   = 1.0,
                                       const m3d::vec3 &vel    = m3d::vec3(0, 0, 0))
    {
        rbps::BodyParams bp;
        bp.position        = pos;
        bp.linear_velocity = vel;
        bp.mass            = mass;
        bp.type            = rbps::BodyType::DYNAMIC;
        const uint32_t bid = w.create_body(bp);

        rbps::ColliderParams cp;
        cp.body_id = bid;
        cp.shape   = rbc::Sphere(radius);
        w.create_collider(cp);
        return bid;
    }

    // Static box: zero-mass body + collider. Returns the body ID. Default
    // half_extents (10, 2, 10) gives a wide ground plane.
    inline uint32_t add_static_box(rbps::World &w,
                                   const m3d::vec3 &pos,
                                   const m3d::vec3 &half_extents = m3d::vec3(10, 2, 10))
    {
        rbps::BodyParams bp;
        bp.position = pos;
        bp.mass     = 0.0;
        bp.type     = rbps::BodyType::STATIC;
        const uint32_t bid = w.create_body(bp);

        rbps::ColliderParams cp;
        cp.body_id = bid;
        cp.shape   = rbc::Box(half_extents);
        w.create_collider(cp);
        return bid;
    }
} // namespace test
