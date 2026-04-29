#pragma once

/**
 * @file ConstraintAPI.hpp
 * @brief Public helpers for adding / removing constraint rows.
 * @ingroup rbps
 */

#include "rbps/constraints/Constraint.hpp"

namespace rbps
{

    /**
     * @brief Parameters for creating a constraint row between two bodies.
     * @ingroup rbps
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
     * @return       Stable uint32_t ID of the new constraint, which can be used to reference it later.
     *
     * @ingroup rbps
     */
    inline uint32_t create_constraint(ConstraintCollection &cc,
                                  const ConstraintParams &params = {})
{
    uint32_t id = cc.add();          // allocate element
    uint32_t i  = cc.index_of(id);   // packed index

    // Fill the allocated slot instead of push_back
    cc.body_1[i] = params.body_1;
    cc.body_2[i] = params.body_2;

    cc.r_1[i] = params.r_1;
    cc.r_2[i] = params.r_2;

    cc.direction[i] = {1.0, 0.0, 0.0}; // placeholder axis

    cc.magnitude[i] = 0.0;
    cc.lambda[i]    = 0.0;

    cc.force[i]  = {0.0, 0.0, 0.0};
    cc.torque[i] = {0.0, 0.0, 0.0};

    cc.compliance[i] = params.compliance;
    cc.type[i]       = params.type;

    cc.impulse[i] = {0.0, 0.0, 0.0};

    return id;
}

    /**
     * @brief Remove a constraint from the collection.
     *
     * Uses swap-and-pop: the last constraint slides into the freed slot so all other
     * IDs remain valid.  Any Handle to this ID becomes false after this call.
     *
     * @param cc  The ConstraintCollection.
     * @param id  Stable ID returned by create_constraint().
     *
     * @ingroup rbps
     */
    inline void remove_constraint(ConstraintCollection &cc, uint32_t id)
    {
        cc.remove(id);
    }

} // namespace rbps