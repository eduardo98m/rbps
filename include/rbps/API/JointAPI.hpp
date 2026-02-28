#pragma once
// JointsAPI.hpp

#include "rbps/constraints/Joint.hpp"
#include "rbps/API/ConstraintAPI.hpp"

namespace rbps
{

// ─── Parameter structs ────────────────────────────────────────────────────────

struct PrismaticJointParams
{
    size_t             body_1         = 0;
    size_t             body_2         = 0;
    vec3               moving_axis    = {1.0, 0.0, 0.0};
    vec3               r_1            = {0.0, 0.0, 0.0};
    vec3               r_2            = {0.0, 0.0, 0.0};
    JointActuationType actuation_type = JointActuationType::FREE;
    scalar             compliance     = 0.0;
    scalar             damping        = 0.0;
    bool               limited        = false;
    scalar             lower_limit    = 0.0;
    scalar             upper_limit    = 0.0;
};

struct RevoluteJointParams
{
    size_t             body_1         = 0;
    size_t             body_2         = 0;
    vec3               aligned_axis   = {1.0, 0.0, 0.0};
    vec3               limit_axis     = {0.0, 1.0, 0.0};
    vec3               r_1            = {0.0, 0.0, 0.0};
    vec3               r_2            = {0.0, 0.0, 0.0};
    JointActuationType actuation_type = JointActuationType::FREE;
    scalar             compliance     = 0.0;
    scalar             damping        = 0.0;
    bool               limited        = false;
    scalar             lower_limit    = 0.0;
    scalar             upper_limit    = 0.0;
};

struct FixedJointParams
{
    size_t body_1 = 0;
    size_t body_2 = 0;
    vec3   r_1    = {0.0, 0.0, 0.0};
    vec3   r_2    = {0.0, 0.0, 0.0};
};

// ─── Swap / pop (auto-generated from JOINT_FIELDS) ───────────────────────────
//
// constraint_start and constraint_count are included in JOINT_FIELDS and
// therefore ARE swapped here. This is intentional: when swap-and-pop moves
// joint B into joint A's old slot, the constraint range indices must travel
// with it so the solver still reads the right rows in ConstraintCollection.

inline void swap_joint_arrays(JointCollection& jc, size_t a, size_t b)
{
    using std::swap;
#define SWAP_FIELD(type, name) swap(jc.name[a], jc.name[b]);
    JOINT_FIELDS(SWAP_FIELD)
#undef SWAP_FIELD
}

inline void pop_back_joint(JointCollection& jc)
{
#define POP_FIELD(type, name) jc.name.pop_back();
    JOINT_FIELDS(POP_FIELD)
#undef POP_FIELD
}

// ─── remove_joint ─────────────────────────────────────────────────────────────

/**
 * @brief Remove a joint and all of its child constraints.
 *
 * Child constraints are removed in reverse order so that each swap-and-pop
 * inside remove_constraint does not disturb the IVC IDs of the siblings
 * that are still pending removal.
 *
 * Order of operations:
 *   1. Read constraint_start / constraint_count from the joint (while still valid).
 *   2. Remove child constraints last-to-first.
 *   3. Remove the joint itself.
 *
 * @param jc  The JointCollection.
 * @param cc  The ConstraintCollection that owns the child constraints.
 * @param id  Stable ivc::ID of the joint to remove.
 */
inline void remove_joint(JointCollection&      jc,
                         ConstraintCollection& cc,
                         ivc::ID               id)
{
    // 1. Capture child-constraint info before the joint is gone.
    const size_t  idx     = ivc::index(jc._ivc, id);
    const size_t  c_start = jc.constraint_start[idx];
    const u_short c_count = jc.constraint_count[idx];

    // 2. Remove child constraints last-to-first.
    //    IVC IDs for this joint's constraints are c_start, c_start+1, ...,
    //    c_start+c_count-1  (they were allocated sequentially in create_*_joint).
    //    Removing from the end first keeps the earlier siblings' IDs unaffected.
    for (int k = static_cast<int>(c_count) - 1; k >= 0; --k)
        remove_constraint(cc, static_cast<ivc::ID>(c_start + k));

    // 3. Remove the joint.
    ivc::erase(jc._ivc, id, [&](size_t a, size_t b) {
        swap_joint_arrays(jc, a, b);
    });
    pop_back_joint(jc);
}

// ─── Shared push helper ───────────────────────────────────────────────────────
//
// All three joint creators share the same 17 push_back calls.
// Factored here to avoid repetition — no namespace wrapping.

inline ivc::ID push_joint(JointCollection& jc,
                           size_t c_start, u_short c_count,
                           size_t body_1, size_t body_2,
                           vec3 r_1, vec3 r_2,
                           JointType type,
                           JointActuationType actuation,
                           bool limited,
                           scalar lower, scalar upper,
                           vec3 main_axis, vec3 limit_axis,
                           scalar damping)
{
    // ivc::add() MUST be called — not n_joints++.
    // It fills _ivc.indexes + _ivc.metadata so remove_joint() can find this
    // entry later. Skipping it causes a segfault on the first remove call.
    ivc::ID id = ivc::add(jc._ivc);

    jc.type.push_back(type);
    jc.actuation_type.push_back(actuation);
    jc.body_1.push_back(body_1);
    jc.body_2.push_back(body_2);
    jc.r_1.push_back(r_1);
    jc.r_2.push_back(r_2);
    jc.main_axis.push_back(main_axis);
    jc.limited.push_back(limited);
    jc.lower_limit.push_back(lower);
    jc.upper_limit.push_back(upper);
    jc.target_position.push_back(0.0);
    jc.target_speed.push_back(0.0);
    jc.current_position.push_back(0.0);
    jc.damping.push_back(damping);
    jc.limit_axis.push_back(limit_axis);
    jc.constraint_start.push_back(c_start);
    jc.constraint_count.push_back(c_count);

    return id;
}

// ─── create_prismatic_joint ───────────────────────────────────────────────────

/**
 * @brief Create a prismatic (sliding) joint between two bodies.
 *
 * Child constraints in cc:
 *   [start+0] ROTATIONAL — full orientation lock
 *   [start+1] POSITIONAL — attachment-point lock (perpendicular to axis)
 *   [start+2] POSITIONAL — drive / limit along the axis (carries compliance)
 *
 * @return Stable ivc::ID of the joint.
 */
inline ivc::ID create_prismatic_joint(JointCollection&            jc,
                                      ConstraintCollection&       cc,
                                      const PrismaticJointParams& params = {})
{
    const size_t c_start = cc.n_constraints;

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::ROTATIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::POSITIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::POSITIONAL,
                            .r_1=params.r_1, .r_2=params.r_2,
                            .compliance=params.compliance});

    return push_joint(jc,
        c_start, 3,
        params.body_1, params.body_2, params.r_1, params.r_2,
        JointType::PRISMATIC, params.actuation_type,
        params.limited, params.lower_limit, params.upper_limit,
        params.moving_axis, {0.0, 0.0, 0.0},
        params.damping);
}

// ─── create_revolute_joint ────────────────────────────────────────────────────

/**
 * @brief Create a revolute (hinge) joint between two bodies.
 *
 * Child constraints in cc:
 *   [start+0] ROTATIONAL — axis alignment
 *   [start+1] POSITIONAL — attachment-point lock
 *   [start+2] ROTATIONAL — angle limit
 *   [start+3] ROTATIONAL — drive (carries compliance)
 *
 * @return Stable ivc::ID of the joint.
 */
inline ivc::ID create_revolute_joint(JointCollection&           jc,
                                     ConstraintCollection&      cc,
                                     const RevoluteJointParams& params = {})
{
    const size_t c_start = cc.n_constraints;

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::ROTATIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::POSITIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::ROTATIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::ROTATIONAL,
                            .r_1=params.r_1, .r_2=params.r_2,
                            .compliance=params.compliance});

    return push_joint(jc,
        c_start, 4,
        params.body_1, params.body_2, params.r_1, params.r_2,
        JointType::REVOLUTE, params.actuation_type,
        params.limited, params.lower_limit, params.upper_limit,
        params.aligned_axis, params.limit_axis,
        params.damping);
}

// ─── create_fixed_joint ───────────────────────────────────────────────────────

/**
 * @brief Create a fixed joint between two bodies (no relative motion).
 *
 * Child constraints in cc:
 *   [start+0] ROTATIONAL — full orientation lock
 *   [start+1] POSITIONAL — attachment-point lock
 *
 * @return Stable ivc::ID of the joint.
 */
inline ivc::ID create_fixed_joint(JointCollection&        jc,
                                  ConstraintCollection&   cc,
                                  const FixedJointParams& params = {})
{
    const size_t c_start = cc.n_constraints;

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::ROTATIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    create_constraint(cc, {.body_1=params.body_1, .body_2=params.body_2,
                            .type=ConstraintType::POSITIONAL,
                            .r_1=params.r_1, .r_2=params.r_2, .compliance=0.0});

    return push_joint(jc,
        c_start, 2,
        params.body_1, params.body_2, params.r_1, params.r_2,
        JointType::FIXED, JointActuationType::FREE,
        false, 0.0, 0.0,
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        0.0);
}

} // namespace rbps