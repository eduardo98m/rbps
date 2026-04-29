#pragma once

/**
 * @file BodyAPI.hpp
 * @brief Public helpers to add / remove bodies in a `BodyCollection`.
 * @ingroup rbps
 *
 * Callers work with stable `uint32_t` IDs — these survive other bodies
 * being added or removed. All bookkeeping (slot allocation, generation
 * counter) is delegated to `BodyCollection` (`DynSoABase`).
 */

#include "rbps/Body.hpp"

namespace rbps
{

    /**
     * @brief Initial properties for a new body. All fields have sensible defaults.
     * @ingroup rbps
     */
    struct BodyParams
    {
        BodyType type = DYNAMIC;
        scalar mass = 1.0;
        smat3 inertia_tensor = smat3{1.0, 1.0, 1.0, 0, 0, 0};
        vec3 position = vec3(0.0);
        quat orientation = quat{1, 0, 0, 0};
        vec3 linear_velocity = vec3(0.0);
        vec3 angular_velocity = vec3(0.0);
    };

    /**
     * @brief Add a new body to the collection and return its stable ID.
     *
     * The SoA allocates the slot and zero-initialises all fields first.
     * This function then writes the caller-supplied parameter values.
     * Gravity is baked into the initial force vector.
     *
     * @param bc      The BodyCollection.
     * @param params  Initial body properties (all have defaults).
     * @return        Stable ID — safe to store long-term.
     *                Use bc.index_of(id) to get the current packed array index.
     *
     * @par Example
     * @code
     *   uint32_t ball = create_body(bc, { .mass=2.0, .position={0,5,0} });
     *   uint32_t i    = bc.index_of(ball);    // packed index for direct array access
     * @endcode
     *
     * @ingroup rbps
     */
    inline uint32_t create_body(BodyCollection &bc, const BodyParams &params = {})
    {
        uint32_t id = bc.add();       // allocates slot, zero-inits all field vectors
        uint32_t i = bc.index_of(id); // packed data index of the new item

        const scalar inv_mass = (params.type == STATIC) ? scalar(0) : scalar(1) / params.mass;
        const smat3 I_inv = (params.type == STATIC) ? smat3{} : params.inertia_tensor.inverse();

        bc.force[i] = vec3{0, -9.8f * params.mass, 0};
        bc.torque[i] = vec3(0);
        bc.mass[i] = params.mass;
        bc.inverse_mass[i] = inv_mass;
        bc.type[i] = params.type;
        bc.position[i] = params.position;
        bc.orientation[i] = params.orientation;
        bc.linear_velocity[i] = params.linear_velocity;
        bc.angular_velocity[i] = params.angular_velocity;
        bc.prev_position[i] = params.position;
        bc.prev_orientation[i] = params.orientation;
        bc.prev_linear_velocity[i] = params.linear_velocity;
        bc.prev_angular_velocity[i] = params.angular_velocity;
        bc.inertia_tensor[i] = params.inertia_tensor;
        bc.inverse_inertia_tensor[i] = I_inv;
        bc.inertia_tensor_world[i] = params.inertia_tensor;
        bc.inverse_inertia_tensor_world[i] = I_inv;

        return id;
    }

    /**
     * @brief Remove a body from the collection by stable ID.
     *
     * Uses swap-and-pop internally: the last live body slides into the freed
     * slot so all other IDs and packed indices are unaffected except for the
     * item that was last (its packed index changes).
     *
     * Passing a stale or never-issued ID is safe — it is silently ignored.
     *
     * @param bc  The BodyCollection.
     * @param id  Stable ID returned by create_body().
     *
     * @ingroup rbps
     */
    inline void remove_body(BodyCollection &bc, uint32_t id)
    {
        bc.remove(id);
    }

} // namespace rbps