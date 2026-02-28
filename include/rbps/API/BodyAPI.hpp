#pragma once
// BodyAPI.hpp
//
// Public API for adding and removing bodies from a BodyCollection.
// The caller works only with stable ivc::ID values — these never change
// even when other bodies are added or removed.

#include "rbps/Body.hpp"

namespace rbps
{
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
     * @brief Add a new body to the collection.
     *
     * @param bc     The BodyCollection.
     * @param params Initial body properties (all have defaults).
     * @return       Stable ID — safe to store long-term.
     *               Use  ivc::index(bc._ivc, id)  to get the current array index.
     *
     * Example:
     *   ivc::ID ball = create_body(bc, { .mass = 2.0, .position = {0,5,0} });
     *   ivc::Handle h = ivc::make_handle(bc._ivc, ball); // optional safe ref
     */
    inline ivc::ID create_body(BodyCollection &bc, const BodyParams &params = {})
    {
        // 1. Reserve a stable ID.  n_bodies (_ivc.n_items) is incremented here.
        ivc::ID id = ivc::add(bc._ivc);

        // 2. Push initial values to every data array.
        //    Order must match BODY_FIELDS — the compiler will catch size mismatches
        //    at runtime via assert inside ivc if arrays get out of sync.
        const scalar inv_mass = (params.type == STATIC) ? scalar(0) : scalar(1) / params.mass;
        const smat3 I_inv = (params.type == STATIC) ? smat3{} : params.inertia_tensor.inverse();

        bc.force.push_back(vec3{0, -9.8f * params.mass, 0});
        bc.torque.push_back(vec3(0));
        bc.mass.push_back(params.mass);
        bc.inverse_mass.push_back(inv_mass);
        bc.type.push_back(params.type);
        bc.position.push_back(params.position);
        bc.orientation.push_back(params.orientation);
        bc.linear_velocity.push_back(params.linear_velocity);
        bc.angular_velocity.push_back(params.angular_velocity);
        bc.prev_position.push_back(params.position);
        bc.prev_orientation.push_back(params.orientation);
        bc.prev_linear_velocity.push_back(params.linear_velocity);
        bc.prev_angular_velocity.push_back(params.angular_velocity);
        bc.inertia_tensor.push_back(params.inertia_tensor);
        bc.inverse_inertia_tensor.push_back(I_inv);
        bc.inertia_tensor_world.push_back(params.inertia_tensor);
        bc.inverse_inertia_tensor_world.push_back(I_inv);

        return id;
    }

    inline void swap_body_arrays(BodyCollection &bc, size_t a, size_t b)
    {
        using std::swap;
#define SWAP_FIELD(type, name) swap(bc.name[a], bc.name[b]);
        BODY_FIELDS(SWAP_FIELD)
#undef SWAP_FIELD
    }

    /// Pop the last element from every data array (called once after ivc::erase).
    inline void pop_back_body(BodyCollection &bc)
    {
#define POP_FIELD(type, name) bc.name.pop_back();
        BODY_FIELDS(POP_FIELD)
#undef POP_FIELD
    }

    /**
     * @brief Remove a body from the collection.
     *
     * Uses swap-and-pop: the last body slides into the freed slot so all other
     * IDs remain valid.  Any Handle to this ID becomes false after this call.
     *
     * @param bc  The BodyCollection.
     * @param id  Stable ID returned by create_body().
     */
    inline void remove_body(BodyCollection &bc, ivc::ID id)
    {
        // swap_body_arrays and pop_back_body are auto-generated from BODY_FIELDS,
        // so adding a new field to the collection automatically updates them.
        ivc::erase(bc._ivc, id, [&](size_t a, size_t b)
                   { swap_body_arrays(bc, a, b); });
        pop_back_body(bc);
    }

} // namespace rbps