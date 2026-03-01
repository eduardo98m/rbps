#pragma once
#include <vector>
#include <ivc/ivc.hpp>
#include "rbc/AABB.hpp"
#include "rbc/BroadPhase.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

// ============================================================================
//  rpbs/ColliderCollection.hpp
//
//  Pure collision layer — no knowledge of rbps bodies or physics.
//  A collider is: shape + local offset from its owning body + material props.
//
//  Layout: SoA (struct of arrays), same pattern as rbps::BodyCollection.
//  Identity: ivc::ID (stable across add/remove), same pattern as BodyAPI.
//
//  WHERE THIS LIVES IN THE ARCHITECTURE:
//    rbc  — geometry, shapes, AABBs, broad phase, narrow phase dispatch
//    rbps — bodies, integration, constraints, pipeline (uses this)
//
//  ADDING A NEW SHAPE:
//    Add compute_aabb() overload in the shape header.
//    No other changes needed here.
//
//  FUTURE: PLANES / INFINITE SHAPES
//    Planes cannot be in the SAP endpoint list (infinite AABB breaks the sort).
//    Add a  std::vector<PlaneCollider> static_planes  field below and test every
//    dynamic object against them *after* bp_sweep(), outside the endpoint list.
//    See CollisionPipeline.cpp for the hook comment.
//
//  FUTURE: HEIGHTMAPS
//    Same idea: a HeightmapCollider stores a grid of AABBs per cell.  Register
//    only the cells that are within range of dynamic objects each frame.
// ============================================================================

namespace rbps
{
// ---------------------------------------------------------------------------
//  Field list — add new per-collider data here, everything else is automatic.
// ---------------------------------------------------------------------------
#define COLLIDER_FIELDS(X)          \
    X(rbc::Shape, shape)                 \
    X(m3d::vec3, local_pos)         \
    X(m3d::quat, local_rot)         \
    X(uint32_t, body_id)            \
    X(rbc::BPHandle, bp_handle)          \
    X(bool, is_static)              \
    X(m3d::scalar, restitution)     \
    X(m3d::scalar, static_friction) \
    X(m3d::scalar, dynamic_friction)

    // -----------------------------------------------------------------------
    //  ColliderCollection — owns all collider data.
    //  Use ColliderAPI functions to add/remove colliders safely.
    // -----------------------------------------------------------------------
    struct ColliderCollection
    {
        IVC_CORE;
        size_t &n_colliders = _ivc.n_items;

#define DECLARE_VEC(type, name) std::vector<type> name;
        COLLIDER_FIELDS(DECLARE_VEC)
#undef DECLARE_VEC
    };



    /// Compute the world transform for collider at slot `idx`.
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