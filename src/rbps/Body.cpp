// rbps/Body.cpp
// No changes to the physics math — only the loop bounds and index type
// were updated to match the new BodyCollection API:
//   bc.n_bodies  →  bc.count()
//   size_t i     →  uint32_t i

#include "rbps/Body.hpp"

namespace rbps
{
    void update_inertia_tensor_world(BodyCollection &bc, uint32_t i)
    {
        mat3 R = m3d::mat3_cast(bc.orientation[i]);
        bc.inertia_tensor_world[i] = smat3(R * bc.inertia_tensor[i] * R.transpose());
        bc.inverse_inertia_tensor_world[i] = smat3(R * bc.inverse_inertia_tensor[i] * R.transpose());
    }

    void update_position_and_orientation(BodyCollection &bc, scalar dt)
    {
        for (uint32_t i = 0; i < bc.count(); ++i)
        {
            if (bc.type[i] == STATIC)
                continue;

            update_inertia_tensor_world(bc, i);

            bc.prev_position[i] = bc.position[i];
            bc.linear_velocity[i] += bc.force[i] * bc.inverse_mass[i] * dt;
            bc.position[i] += bc.linear_velocity[i] * dt;

            bc.prev_orientation[i] = bc.orientation[i];
            bc.angular_velocity[i] += dt * bc.inverse_inertia_tensor_world[i] *
                                      (bc.torque[i] - m3d::cross(bc.angular_velocity[i],
                                                                 bc.inertia_tensor_world[i] * bc.angular_velocity[i]));
            vec3 omega_scaled = 0.5 * dt * bc.angular_velocity[i];
            quat rot = quat(0.0, omega_scaled.x, omega_scaled.y, omega_scaled.z);
            bc.orientation[i] = m3d::normalize(bc.orientation[i] + rot * bc.orientation[i]);
        }
    }

    void update_velocities(BodyCollection &bc, scalar inv_dt)
    {
        for (uint32_t i = 0; i < bc.count(); ++i)
        {
            if (bc.type[i] == STATIC)
                continue;

            bc.prev_linear_velocity[i] = bc.linear_velocity[i];
            bc.prev_angular_velocity[i] = bc.angular_velocity[i];

            bc.linear_velocity[i] = (bc.position[i] - bc.prev_position[i]) * inv_dt;

            quat dq = bc.orientation[i] * m3d::conjugate(bc.prev_orientation[i]);
            bc.angular_velocity[i] = 2 * inv_dt * vec3{dq.x, dq.y, dq.z};
            bc.angular_velocity[i] = dq.w >= 0 ? bc.angular_velocity[i] : -bc.angular_velocity[i];
        }
    }

    void apply_positional_constraint_impulse(BodyCollection &bc, uint32_t i, vec3 impulse, vec3 r)
    {
        if (bc.type[i] == STATIC)
            return;
        vec3 cross_ri = m3d::cross(r, impulse);
        bc.position[i] += impulse * bc.inverse_mass[i];
        vec3 rot = bc.inverse_inertia_tensor_world[i] * m3d::cross(r, impulse) * 0.5;
        quat rot_quat = quat(0.0, rot.x, rot.y, rot.z);
        bc.orientation[i] = m3d::normalize(bc.orientation[i] + rot_quat * bc.orientation[i]);
    }

    void apply_rotational_constraint_impulse(BodyCollection &bc, uint32_t i, vec3 impulse)
    {
        if (bc.type[i] == STATIC)
            return;
        vec3 rot = bc.inverse_inertia_tensor_world[i] * impulse * 0.5;
        quat rot_quat = quat(0.0, rot.x, rot.y, rot.z);
        bc.orientation[i] = m3d::normalize(bc.orientation[i] + rot_quat * bc.orientation[i]);
    }

    void apply_positional_velocity_constraint_impulse(BodyCollection &bc, uint32_t i, vec3 impulse, vec3 r)
    {
        if (bc.type[i] == STATIC)
            return;
        bc.linear_velocity[i] += impulse * bc.inverse_mass[i];
        bc.angular_velocity[i] += bc.inverse_inertia_tensor_world[i] * m3d::cross(r, impulse);
    }

    scalar get_positional_generalized_inverse_mass(BodyCollection &bc, uint32_t i, vec3 r, vec3 n)
    {
        if (bc.type[i] == STATIC)
            return 0.0;
        update_inertia_tensor_world(bc, i);
        vec3 cross_r_n = m3d::cross(r, n);
        return bc.inverse_mass[i] + m3d::dot(cross_r_n, bc.inverse_inertia_tensor_world[i] * cross_r_n);
    }

    scalar get_rotational_generalized_inverse_mass(BodyCollection &bc, uint32_t i, vec3 n)
    {
        if (bc.type[i] == STATIC)
            return 0.0;
        update_inertia_tensor_world(bc, i);
        return m3d::dot(n, bc.inverse_inertia_tensor_world[i] * n);
    }

} // namespace rbps