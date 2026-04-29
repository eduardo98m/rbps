#pragma once
#include <vector>
#include <storage/Soa.hpp>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/BroadPhase.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

/**
 * @file Collider.hpp
 * @brief `ColliderCollection` — pure collision layer that owns shapes + materials.
 * @ingroup rbps
 *
 * A collider holds a `rbc::Shape`, an offset relative to its owning body,
 * and material properties (restitution, friction). Layout is SoA (same
 * pattern as `BodyCollection`); identity is a stable `uint32_t` ID.
 *
 * @par Where this lives in the architecture
 * - `rbc`  — geometry, shapes, AABBs, broad phase, narrow-phase dispatch.
 * - `rbps` — bodies, integration, constraints, pipeline (uses this).
 *
 * @par Adding a new shape
 * Add a `compute_aabb()` overload in the new shape's header (see
 * [rbc/shapes/](../rbc/shapes/)). No changes are needed here.
 *
 * @par Future: planes / heightmaps
 * Planes can't enter the SAP endpoint list (their AABB is infinite). They
 * receive `BP_INVALID_HANDLE` and are tested directly inside
 * `run_narrow_phase`. Heightmap support follows the same pattern with
 * per-cell registration.
 */

namespace rbps
{
/**
 * @brief X-macro field list for `ColliderCollection`.
 *
 * Add new per-collider data here; `DEFINE_DYN_SOA` derives all bookkeeping
 * automatically.
 *
 * @ingroup rbps
 */
#define COLLIDER_FIELDS(X)          \
    X(rbc::Shape, shape)            \
    X(m3d::vec3, local_pos)         \
    X(m3d::quat, local_rot)         \
    X(uint32_t, body_id)            \
    X(rbc::BPHandle, bp_handle)     \
    X(bool, is_static)              \
    X(m3d::scalar, restitution)     \
    X(m3d::scalar, static_friction) \
    X(m3d::scalar, dynamic_friction)

    /**
     * @ingroup rbps
     * @brief SoA collection of colliders. Use `ColliderAPI` to add/remove safely.
     *
     * Each row pairs a `rbc::Shape` with a body-local pose offset, a
     * broad-phase handle, and material parameters used by the contact
     * solver. Shapes that bypass the broad phase (e.g. `Plane`) carry
     * `BP_INVALID_HANDLE` and are dispatched separately.
     */
    DEFINE_DYN_SOA(ColliderCollection, uint32_t, /*GenerationBits=*/8, COLLIDER_FIELDS)

    /**
     * @brief Compute the world transform for the collider at slot `idx`.
     *
     * Composes the body's world pose with the collider's body-local offset:
     * `world_tf = (body_pos + body_rot * local_pos, body_rot * local_rot)`.
     *
     * @ingroup rbps
     */
    inline m3d::tf collider_world_tf(const ColliderCollection &cc,
                                     size_t idx,
                                     const m3d::vec3 &body_pos,
                                     const m3d::quat &body_rot)
    {
        return {
            body_pos + m3d::rotate(body_rot, cc.local_pos[idx]),
            body_rot * cc.local_rot[idx]};
    }

} // namespace rbps
