#pragma once
#include "rbps/Collider.hpp"

namespace rbps
{

    // -----------------------------------------------------------------------
    //  Params passed to collider_add()
    // -----------------------------------------------------------------------
    struct ColliderParams
    {
        rbc::Shape shape;
        uint32_t body_id = 0;
        bool is_static = false;
        m3d::vec3 local_pos = m3d::vec3(0, 0, 0);
        m3d::quat local_rot = m3d::quat(1, 0, 0, 0);
        m3d::scalar restitution = 0.5;
        m3d::scalar static_friction = 0.5;
        m3d::scalar dynamic_friction = 0.5;
    };

    // -----------------------------------------------------------------------
    //  ColliderAPI
    // -----------------------------------------------------------------------

    /// Register a collider and insert it into the broad phase.
    /// Returns a stable ivc::ID (never changes even after other removes).
    inline ivc::ID collider_add(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                const ColliderParams &p,
                                const m3d::tf &initial_world_tf)
    {
        ivc::ID id = ivc::add(cc._ivc);

        // Compute the world-space transform for the initial AABB
        m3d::vec3 world_pos = initial_world_tf.pos + m3d::rotate(initial_world_tf.rot, p.local_pos);
        m3d::quat world_rot = initial_world_tf.rot * p.local_rot;
        m3d::tf world_tf{world_pos, world_rot};

        rbc::AABB tight = compute_aabb(p.shape, world_tf);

        // Arent this the same?
        rbc::BPHandle bph = p.is_static
                           ? rbc::broad_phase_insert(bp, static_cast<uint32_t>(id), tight)
                           : rbc::broad_phase_insert(bp, static_cast<uint32_t>(id), tight);

        cc.shape.push_back(p.shape);
        cc.local_pos.push_back(p.local_pos);
        cc.local_rot.push_back(p.local_rot);
        cc.body_id.push_back(p.body_id);
        cc.bp_handle.push_back(bph);
        cc.is_static.push_back(p.is_static);
        cc.restitution.push_back(p.restitution);
        cc.static_friction.push_back(p.static_friction);
        cc.dynamic_friction.push_back(p.dynamic_friction);

        return id;
    }

    inline void swap_collider_arrays(ColliderCollection &cc, size_t a, size_t b)
    {
        using std::swap;
#define SWAP_FIELD(type, name) swap(cc.name[a], cc.name[b]);
        COLLIDER_FIELDS(SWAP_FIELD)
#undef SWAP_FIELD
    }

    inline void pop_back_collider(ColliderCollection &cc)
    {
#define POP_FIELD(type, name) cc.name.pop_back();
        COLLIDER_FIELDS(POP_FIELD)
#undef POP_FIELD
    }

    /// Remove a collider and unregister it from the broad phase.
    inline void collider_remove(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                ivc::ID id)
    {
        size_t idx = ivc::index(cc._ivc, id);
        rbc::broad_phase_remove(bp, cc.bp_handle[idx]);
        ivc::erase(cc._ivc, id,
                   [&](size_t a, size_t b)
                   { swap_collider_arrays(cc, a, b); });
        pop_back_collider(cc);
    }

} // namespace rbps