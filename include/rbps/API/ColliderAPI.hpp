#pragma once
#include "rbps/Collider.hpp"

/**
 * @file ColliderAPI.hpp
 * @brief Public helpers to add / remove colliders and keep the broad phase in sync.
 * @ingroup rbps
 */

namespace rbps
{

    /**
     * @brief Initial properties for a new collider.
     * @ingroup rbps
     */
    struct ColliderParams
    {
        rbc::Shape shape;                       ///< Geometric shape (any variant of `rbc::Shape`).
        uint32_t body_id = 0;                   ///< Stable ID of the body that owns this collider.
        bool is_static = false;                 ///< Tags the collider as non-moving for the static-pair filter.
        m3d::vec3 local_pos = m3d::vec3(0, 0, 0); ///< Body-local offset.
        m3d::quat local_rot = m3d::quat(1, 0, 0, 0); ///< Body-local rotation.
        m3d::scalar restitution = 0.5;          ///< Bounciness used by the velocity-level solver.
        m3d::scalar static_friction = 0.5;      ///< Coulomb static-friction coefficient.
        m3d::scalar dynamic_friction = 0.5;     ///< Coulomb dynamic-friction coefficient.
    };

    /**
     * @brief Predicate: shapes whose AABB cannot enter the SAP broad phase.
     *
     * `Plane` has an infinite AABB and is handled by a dedicated loop in
     * `run_narrow_phase` (planes are tested against every dynamic
     * collider). Heightmap follows the same pattern once its dispatch
     * is wired up.
     *
     * @ingroup rbps
     */
    inline bool shape_bypasses_broadphase(const rbc::Shape &s)
    {
        return s.is<rbc::Plane>();
        // Heightmap will be added here in Phase 2.
    }

    /**
     * @brief Register a collider and (conditionally) insert it into the broad phase.
     *
     * Planes and other infinite shapes skip broad-phase insertion entirely
     * and receive `BP_INVALID_HANDLE`; the collision pipeline handles them
     * in a dedicated loop inside `run_narrow_phase`.
     *
     * @param cc            The collider collection.
     * @param bp            Broad-phase state to update.
     * @param p             Collider properties.
     * @param body_world_tf World transform of the owning body, used to
     *                      compute the initial AABB.
     * @return Stable `uint32_t` ID of the new collider.
     *
     * @ingroup rbps
     */
    inline uint32_t create_collider(ColliderCollection &cc,
                                    rbc::BroadPhaseState &bp,
                                    const ColliderParams &p,
                                    const m3d::tf &body_world_tf)
    {
        uint32_t id = cc.add();
        uint32_t i = cc.index_of(id);

        const m3d::vec3 world_pos = body_world_tf.pos + m3d::rotate(body_world_tf.rot, p.local_pos);
        const m3d::quat world_rot = body_world_tf.rot * p.local_rot;
        const m3d::tf world_tf{world_pos, world_rot};

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

    /**
     * @brief Remove a collider and (if it was in the broad phase) unregister it.
     * @ingroup rbps
     */
    inline void collider_remove(ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                uint32_t id)
    {
        const size_t idx = cc.index_of(id);
        if (cc.bp_handle[idx] != rbc::BP_INVALID_HANDLE)
            rbc::broad_phase_remove(bp, cc.bp_handle[idx]);
        cc.remove(id);
    }

} // namespace rbps
