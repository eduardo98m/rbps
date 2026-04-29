#pragma once
#include "rbps/Collider.hpp"

namespace rbps
{

    // -----------------------------------------------------------------------
    //  Params passed to create_collider()
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
    //  Shape types that cannot enter the SAP broad phase because their AABB
    //  is infinite (Plane) or must be handled outside the endpoint list.
    //  These colliders receive BP_INVALID_HANDLE and are tested analytically
    //  in run_narrow_phase() by iterating them against every dynamic collider.
    // -----------------------------------------------------------------------
    inline bool shape_bypasses_broadphase(const rbc::Shape &s)
    {
        return s.is<rbc::Plane>();
        // Heightmap will be added here in Phase 2.
    }

    // -----------------------------------------------------------------------
    //  Register a collider and (conditionally) insert it into the broad phase.
    //  Returns a stable uint32_t ID.
    //
    //  Planes and other infinite shapes skip broad-phase insertion entirely
    //  and receive BP_INVALID_HANDLE.  The collision pipeline handles them
    //  in a dedicated loop inside run_narrow_phase().
    // -----------------------------------------------------------------------
    inline uint32_t create_collider(ColliderCollection &cc,
                                    rbc::BroadPhaseState &bp,
                                    const ColliderParams &p,
                                    const m3d::tf &body_world_tf)
    {
        uint32_t id = cc.add();
        uint32_t i = cc.index_of(id);

        // Compute world transform and tight AABB for the initial broad-phase entry.
        const m3d::vec3 world_pos = body_world_tf.pos + m3d::rotate(body_world_tf.rot, p.local_pos);
        const m3d::quat world_rot = body_world_tf.rot * p.local_rot;
        const m3d::tf world_tf{world_pos, world_rot};

        // Decide whether this shape participates in the SAP.
        rbc::BPHandle bph = rbc::BP_INVALID_HANDLE;
        if (!shape_bypasses_broadphase(p.shape))
        {
            const rbc::AABB tight = rbc::compute_aabb(p.shape, world_tf);
            bph = rbc::broad_phase_insert(bp, static_cast<uint32_t>(id), tight);
        }

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

    // -----------------------------------------------------------------------
    //  Remove a collider and (if it was in the broad phase) unregister it.
    // -----------------------------------------------------------------------
    inline void collider_remove(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                uint32_t id)
    {
        const size_t idx = cc.index_of(id);
        if (cc.bp_handle[idx] != rbc::BP_INVALID_HANDLE) // Avoid out of index for planes and other non-BP shapes
            rbc::broad_phase_remove(bp, cc.bp_handle[idx]);
        cc.remove(id);
    }

} // namespace rbps