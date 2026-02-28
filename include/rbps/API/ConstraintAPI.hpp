#pragma once
// ConstraintsAPI.hpp

#include "rbps/constraints/Constraint.hpp"

namespace rbps
{

    /**
     * @brief Parameters for creating a constraint between two bodies.
     */
    struct ConstraintParams
    {
        size_t body_1 = 0;
        size_t body_2 = 0;
        ConstraintType type = ConstraintType::POSITIONAL;
        vec3 r_1 = {0.0, 0.0, 0.0};
        vec3 r_2 = {0.0, 0.0, 0.0};
        scalar compliance = 0.0;
    };

    /**
     * @brief Add a new constraint to the collection.
     *
     * @param cc     The ConstraintCollection.
     * @param params Constraint properties.
     * @return       Index (ivc::ID / uint64_t) of the newly created constraint.
     */
    inline ivc::ID create_constraint(ConstraintCollection &cc, const ConstraintParams &params = {})
    {
        ivc::ID id = ivc::add(cc._ivc);

        cc.body_1.push_back(params.body_1);
        cc.body_2.push_back(params.body_2);
        cc.r_1.push_back(params.r_1);
        cc.r_2.push_back(params.r_2);
        cc.direction.push_back({1.0, 0.0, 0.0}); // placeholder, set by solver
        cc.magnitude.push_back(0.0);
        cc.lambda.push_back(0.0);
        cc.force.push_back({0.0, 0.0, 0.0});
        cc.torque.push_back({0.0, 0.0, 0.0});
        cc.compliance.push_back(params.compliance);
        cc.type.push_back(params.type);
        cc.impulse.push_back({0.0, 0.0, 0.0});

        return id;
    }

    inline void swap_constraint_arrays(ConstraintCollection &cc, size_t a, size_t b)
    {
        using std::swap;
#define SWAP_FIELD(type, name) swap(cc.name[a], cc.name[b]);
        CONSTRAINT_FIELDS(SWAP_FIELD)
#undef SWAP_FIELD
    }

    inline void pop_back_constraint(ConstraintCollection &cc)
    {
#define POP_FIELD(type, name) cc.name.pop_back();
        CONSTRAINT_FIELDS(POP_FIELD)
#undef POP_FIELD
    }

    /**
     * @brief Remove a constraint from the collection.
     *
     * Uses swap-and-pop: the last constraint slides into the freed slot so all other
     * IDs remain valid.  Any Handle to this ID becomes false after this call.
     *
     * @param cc  The ConstraintCollection.
     * @param id  Stable ID returned by create_constraint().
     */
    inline void remove_constraint(ConstraintCollection &cc, ivc::ID id)
    {
        // swap_constraint_arrays and pop_back_constraint are auto-generated from CONSTRAINT_FIELDS,
        // so adding a new field to the constraint collection automatically updates them.
        ivc::erase(cc._ivc, id, [&](size_t a, size_t b)
                   { swap_constraint_arrays(cc, a, b); });
        pop_back_constraint(cc);
    }

} // namespace rbps