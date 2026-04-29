#pragma once

/**
 * @file JointAPI.hpp
 * @brief Public helpers to create / remove joints (prismatic, revolute, fixed).
 * @ingroup rbps
 *
 * Each `create_*_joint` helper allocates the joint row plus its low-level
 * constraint rows in `ConstraintCollection`, and stores the constraint
 * IDs in the joint's `ConstraintBlock`. `remove_joint` reverses the
 * process safely.
 */

#include "rbps/constraints/Joint.hpp"
#include "rbps/API/ConstraintAPI.hpp"

namespace rbps
{

    /**
     * @brief Parameters for `create_prismatic_joint`.
     * @ingroup rbps
     */
    struct PrismaticJointParams
    {
        size_t body_1 = 0;
        size_t body_2 = 0;
        vec3 moving_axis = {1.0, 0.0, 0.0};
        vec3 r_1 = {0.0, 0.0, 0.0};
        vec3 r_2 = {0.0, 0.0, 0.0};
        JointActuationType actuation_type = JointActuationType::FREE;
        scalar compliance = 0.0;
        scalar damping = 0.0;
        bool limited = false;
        scalar lower_limit = 0.0;
        scalar upper_limit = 0.0;
    };

    /**
     * @brief Parameters for `create_revolute_joint`.
     * @ingroup rbps
     */
    struct RevoluteJointParams
    {
        size_t body_1 = 0;
        size_t body_2 = 0;
        vec3 aligned_axis = {1.0, 0.0, 0.0};
        vec3 limit_axis = {0.0, 1.0, 0.0};
        vec3 r_1 = {0.0, 0.0, 0.0};
        vec3 r_2 = {0.0, 0.0, 0.0};
        JointActuationType actuation_type = JointActuationType::FREE;
        scalar compliance = 0.0;
        scalar damping = 0.0;
        bool limited = false;
        scalar lower_limit = 0.0;
        scalar upper_limit = 0.0;
    };

    /**
     * @brief Parameters for `create_fixed_joint`.
     * @ingroup rbps
     */
    struct FixedJointParams
    {
        size_t body_1 = 0;
        size_t body_2 = 0;
        vec3 r_1 = {0.0, 0.0, 0.0};
        vec3 r_2 = {0.0, 0.0, 0.0};
    };

    // // ─── Swap / pop (auto-generated from JOINT_FIELDS) ───────────────────────────
    // //
    // // constraint_start and constraint_count are included in JOINT_FIELDS and
    // // therefore ARE swapped here. This is intentional: when swap-and-pop moves
    // // joint B into joint A's old slot, the constraint range indices must travel
    // // with it so the solver still reads the right rows in ConstraintCollection.

    // inline void swap_joint_arrays(JointCollection& jc, size_t a, size_t b)
    // {
    //     using std::swap;
    // #define SWAP_FIELD(type, name) swap(jc.name[a], jc.name[b]);
    //     JOINT_FIELDS(SWAP_FIELD)
    // #undef SWAP_FIELD
    // }

    // inline void pop_back_joint(JointCollection& jc)
    // {
    // #define POP_FIELD(type, name) jc.name.pop_back();
    //     JOINT_FIELDS(POP_FIELD)
    // #undef POP_FIELD
    // }

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
     *
     * @ingroup rbps
     */
    inline void remove_joint(JointCollection& jc, ConstraintCollection& cc, uint32_t id)
    {
        // 1. Read child info before the joint slot is freed.
        const uint32_t    idx     = jc.index_of(id);
        const u_short     c_count = jc.constraint_count[idx];
        const ConstraintBlock& cb = jc.constraints[idx];
 
        // 2. Remove each child constraint via its stable ID.
        for (u_short k = 0; k < c_count; ++k)
            cc.remove(cb.ids[k]);
 
        // 3. Remove the joint itself.
        jc.remove(id);
    }

    /**
     * @brief Internal helper: write all joint fields for a freshly-allocated row.
     *
     * Factored out so the three `create_*_joint` helpers don't duplicate
     * the 17 field assignments.
     *
     * @ingroup rbps
     * @ingroup internals
     */
    inline uint32_t push_joint(JointCollection &jc,
                               ConstraintBlock cb,
                               u_short c_count,
                               uint32_t body_1,
                               uint32_t body_2,
                               vec3 r_1,
                               vec3 r_2,
                               JointType type,
                               JointActuationType actuation,
                               bool limited,
                               scalar lower,
                               scalar upper,
                               vec3 main_axis,
                               vec3 limit_axis,
                               scalar damping)
    {
        uint32_t id = jc.add();
        uint32_t i = jc.index_of(id);

        jc.type[i] = type;
        jc.actuation_type[i] = actuation;
        jc.body_1[i] = body_1;
        jc.body_2[i] = body_2;
        jc.r_1[i] = r_1;
        jc.r_2[i] = r_2;
        jc.main_axis[i] = main_axis;
        jc.limited[i] = limited;
        jc.lower_limit[i] = lower;
        jc.upper_limit[i] = upper;
        jc.target_position[i] = 0.0;
        jc.target_speed[i] = 0.0;
        jc.current_position[i] = 0.0;
        jc.damping[i] = damping;
        jc.limit_axis[i] = limit_axis;
        jc.constraint_count[i] = c_count;
        jc.constraints[i] = cb;

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
     *
     * @ingroup rbps
     */
    inline uint32_t create_prismatic_joint(JointCollection &jc,
                                           ConstraintCollection &cc,
                                           const PrismaticJointParams &params = {})
    {
        ConstraintBlock cb{};
        cb.ids[0] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::ROTATIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});
        cb.ids[1] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::POSITIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});
        cb.ids[2] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::POSITIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = params.compliance});

        return push_joint(jc, cb, 3,
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
     *
     * @ingroup rbps
     */
    inline uint32_t create_revolute_joint(JointCollection &jc,
                                          ConstraintCollection &cc,
                                          const RevoluteJointParams &params = {})
    {
        ConstraintBlock cb{};
        cb.ids[0] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::ROTATIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});
        cb.ids[1] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::POSITIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});
        cb.ids[2] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::ROTATIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});
        cb.ids[3] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::ROTATIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = params.compliance});

        return push_joint(jc, cb, 4,
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
     * @return Stable uint32_t (id) of the joint.
     *
     * @ingroup rbps
     */
    inline uint32_t create_fixed_joint(JointCollection &jc,
                                       ConstraintCollection &cc,
                                       const FixedJointParams &params = {})
    {
        ConstraintBlock cb{};
        cb.ids[0] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::ROTATIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});
        cb.ids[1] = create_constraint(cc, {.body_1 = params.body_1, .body_2 = params.body_2, .type = ConstraintType::POSITIONAL, .r_1 = params.r_1, .r_2 = params.r_2, .compliance = 0.0});

        return push_joint(jc, cb, 2,
                          params.body_1, params.body_2, params.r_1, params.r_2,
                          JointType::FIXED, JointActuationType::FREE,
                          false, 0.0, 0.0,
                          {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                          0.0);
    }

} // namespace rbps