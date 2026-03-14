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
    /// Returns a stable uint32_t ID for referencing this collider later (e.g. to remove it).
    inline u_int32_t create_collider(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                const ColliderParams &p,
                                const m3d::tf &body_world_tf)
    {
        u_int32_t id = cc.add();
        uint32_t i = cc.index_of(id);

        // Compute the world-space transform for the initial AABB
        m3d::vec3 world_pos = body_world_tf.pos + m3d::rotate(body_world_tf.rot, p.local_pos);
        m3d::quat world_rot = body_world_tf.rot * p.local_rot;
        m3d::tf world_tf{world_pos, world_rot};

        rbc::AABB tight = compute_aabb(p.shape, world_tf);

        // Arent this the same?
        rbc::BPHandle bph = p.is_static
                           ? rbc::broad_phase_insert(bp, static_cast<uint32_t>(id), tight)
                           : rbc::broad_phase_insert(bp, static_cast<uint32_t>(id), tight);

        cc.shape[i] = p.shape;
        cc.local_pos[i] = p.local_pos;
        cc.local_rot[i] = p.local_rot;
        cc.body_id[i] = p.body_id;
        cc.bp_handle[i] = bph;
        cc.is_static[i] = p.is_static;
        cc.restitution[i] = p.restitution;
        cc.static_friction[i] = p.static_friction;
        cc.dynamic_friction[i] = p.dynamic_friction;

        return id;
    }


    /// Remove a collider and unregister it from the broad phase.
    inline void collider_remove(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                u_int32_t id)
    {
        size_t idx =  cc.index_of(id);
        rbc::broad_phase_remove(bp, cc.bp_handle[idx]);
        cc.remove(id);
    }

} // namespace rbps