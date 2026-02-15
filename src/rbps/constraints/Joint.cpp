
#include "rbps/constraints/Joint.hpp"
namespace rbps
{
    void compute_prismatic_joint_errors(JointCollection &jc,
                                        size_t i,
                                        BodyCollection &bc,
                                        ConstraintCollection &cc,
                                        scalar time_step)
    {

        const size_t start = jc.constraint_start[i];
        const size_t b1 = jc.body_1[i];
        const size_t b2 = jc.body_2[i];
        // 1) Alignment error (rotational constraint)
        quat dq = bc.orientation[b1] * m3d::conjugate(bc.orientation[b2]);
        vec3 err_align = vec3{dq.x, dq.y, dq.z} * scalar(2.0);
        set_value(cc, start + 0, err_align);
        // 2) Attachment‐point error (positional constraint)
        vec3 r1_wc = m3d::rotate(bc.orientation[b1], jc.r_1[i]);
        vec3 r2_wc = m3d::rotate(bc.orientation[b2], jc.r_2[i]);
        vec3 p1 = bc.position[b1] + r1_wc;
        vec3 p2 = bc.position[b2] + r2_wc;
        vec3 delta_p = p1 - p2;
        vec3 axis = m3d::rotate(bc.orientation[b1], jc.main_axis[i]);
        scalar pos = m3d::dot(delta_p, axis);
        scalar overshoot = scalar(0);
        if (jc.limited[i])
        {
            scalar clamped = m3d::clamp(pos,
                                        jc.lower_limit[i],
                                        jc.upper_limit[i]);
            overshoot = pos - clamped;
        }
        vec3 correction = delta_p - axis * pos + axis * overshoot;
        set_value(cc, start + 1, correction);
        // 3) Drive error (positional constraint)
        vec3 err_drive = vec3{0.0, 0.0, 0.0};

        switch (jc.actuation_type[i])
        {
        case JointActuationType::FREE:
            break;
        case JointActuationType::SPEED:
            jc.target_position[i] += jc.target_speed[i] * time_step;
        case JointActuationType::POSITION:
        {
            vec3 drive_dp = p2 - p1;
            scalar curr = m3d::dot(drive_dp, axis);
            err_drive = (jc.target_position[i] - curr) * axis;
            break;
        }
        }

        set_value(cc, start + 2, err_drive);
        jc.current_position[i] = pos;
    }

    vec3 compute_angle_limit_correction(scalar &phi,
                                        vec3 n,
                                        vec3 n_1,
                                        vec3 n_2,
                                        scalar lower_limit,
                                        scalar upper_limit)
    {

        vec3 delta_q = {0.0, 0.0, 0.0};
        if (phi < lower_limit || phi > upper_limit)
        {
            phi = m3d::clamp(phi, lower_limit, upper_limit);
            quat rot = m3d::quat::from_axis_angle(n, phi);
            n_1 = m3d::rotate(rot, n_1);
            delta_q = m3d::cross(n_1, n_2);
        }
        return delta_q;
    }

    void compute_revolute_joint_errors(JointCollection &jc,
                                       size_t i,
                                       BodyCollection &bc,
                                       ConstraintCollection &cc,
                                       scalar time_step)
    {
        const size_t start = jc.constraint_start[i];
        const size_t b1 = jc.body_1[i];
        const size_t b2 = jc.body_2[i];
        // 1) alignment (rotational)
        // main_axis in world on each body
        vec3 a1 = m3d::rotate(bc.orientation[b1], jc.main_axis[i]);
        vec3 a2 = m3d::rotate(bc.orientation[b2], jc.main_axis[i]);
        vec3 err_align = m3d::cross(a1, a2); // According to the original paper err = a_1 x a_2
        set_value(cc, start + 0, err_align);
        // 2) attachment (positional)
        vec3 r1_wc = m3d::rotate(bc.orientation[b1], jc.r_1[i]);
        vec3 r2_wc = m3d::rotate(bc.orientation[b2], jc.r_2[i]);
        vec3 p1 = bc.position[b1] + r1_wc;
        vec3 p2 = bc.position[b2] + r2_wc;
        set_value(cc, start + 1, p1 - p2);

        // This peice of code was just to test the distance constraint
        // scalar current_distance = m3d::magnitude(p1 - p2);
        // scalar desired_length = m3d::magnitude(jc.r_1[i] - jc.r_2[i]);
        // vec3 direction = m3d::normalize(p1 - p2);
        // vec3 error_vector_for_distance_constraint = direction * (current_distance - desired_length);
        // set_value(cc, start + 1, error_vector_for_distance_constraint);

        // 3) angle limit (rotational)
        // compute current angle φ = atan2( (b1×b2)·a1, b1·b2 )
        vec3 b1_lim = m3d::rotate(bc.orientation[b1], jc.limit_axis[i]);
        vec3 b2_lim = m3d::rotate(bc.orientation[b2], jc.limit_axis[i]);
        scalar phi = m3d::atan2(
            m3d::dot(m3d::cross(b1_lim, b2_lim), a1),
            m3d::dot(b1_lim, b2_lim));
        vec3 err_limit{0.0, 0.0, 0.0};
        if (jc.limited[i])
        {
            err_limit = compute_angle_limit_correction(
                phi,
                a1,
                b1_lim,
                b2_lim,
                jc.lower_limit[i],
                jc.upper_limit[i]);
        }
        set_value(cc, start + 2, -err_limit);
        // 4) drive (rotational)
        vec3 err_drive{0.0, 0.0, 0.0};
        switch (jc.actuation_type[i])
        {
        case JointActuationType::FREE:
            break;
        case JointActuationType::SPEED:
            jc.target_position[i] += jc.target_speed[i] * time_step;
        case JointActuationType::POSITION:
        {
            scalar theta = jc.target_position[i] + PI;
            vec3 b_target = b1_lim * m3d::cos(theta) +
                            m3d::cross(a1, b1_lim) * m3d::sin(theta) +
                            a1 * (m3d::dot(a1, b1_lim)) * (1 - m3d::cos(theta));
            err_drive = m3d::cross(b_target, b2_lim);
            break;
        }
        }
        set_value(cc, start + 3, err_drive);
        jc.current_position[i] = phi;
    }

    void compute_fixed_joint_errors(
        JointCollection &jc,
        size_t i,
        BodyCollection &bc,
        ConstraintCollection &cc,
        scalar /*time_step—unused*/
    )
    {
        const size_t start = jc.constraint_start[i];
        const size_t b1 = jc.body_1[i];
        const size_t b2 = jc.body_2[i];
        // 1) Alignment error (rotational constraint)
        quat dq = bc.orientation[b1] * m3d::conjugate(bc.orientation[b2]);
        vec3 err_align = vec3{dq.x, dq.y, dq.z} * scalar(2.0);
        set_value(cc, start + 0, err_align);
        // 2) Attachment-point error (positional constraint)
        vec3 r1w = m3d::rotate(bc.orientation[b1], jc.r_1[i]);
        vec3 r2w = m3d::rotate(bc.orientation[b2], jc.r_2[i]);
        vec3 p1 = bc.position[b1] + r1w;
        vec3 p2 = bc.position[b2] + r2w;
        vec3 err_pos = p1 - p2;
        set_value(cc, start + 1, err_pos);
    }

    void apply_prismatic_joint_damping(JointCollection &jc,
                                       size_t i,
                                       BodyCollection &bc,
                                       scalar time_step)
    {
        const size_t b1 = jc.body_1[i];
        const size_t b2 = jc.body_2[i];
        vec3 delta_v = (bc.linear_velocity[b2] - bc.linear_velocity[b1]) * std::min(jc.damping[i] * time_step, 1.0);
        if (m3d::magnitude(delta_v) < EPSILON)
            return;
        vec3 r_1_wc = m3d::rotate(bc.orientation[b1], jc.r_1[i]);
        vec3 r_2_wc = m3d::rotate(bc.orientation[b2], jc.r_2[i]);
        vec3 n = m3d::normalize(delta_v);
        scalar w_1 = get_positional_generalized_inverse_mass(bc, b1, r_1_wc, n);
        scalar w_2 = get_positional_generalized_inverse_mass(bc, b2, r_2_wc, n);
        vec3 impulse = delta_v / (w_1 + w_2);
        apply_positional_velocity_constraint_impulse(bc, b1, impulse, r_1_wc);
        apply_positional_velocity_constraint_impulse(bc, b2, -impulse, r_2_wc);
    }

    void apply_revolute_joint_damping(JointCollection &jc,
                                      size_t i,
                                      BodyCollection &bc,
                                      scalar time_step)
    {
        const size_t b1 = jc.body_1[i];
        const size_t b2 = jc.body_2[i];
        vec3 delta_omega = (bc.angular_velocity[b2] - bc.angular_velocity[b1]) * std::min(jc.damping[i] * time_step, 1.0);
        bc.angular_velocity[b1] += delta_omega;
        bc.angular_velocity[b2] -= delta_omega;
    };

    void compute_joint_errors(JointCollection &jc,
                              BodyCollection &bc,
                              ConstraintCollection &cc,
                              scalar time_step)
    {
        for (size_t i = 0; i < jc.n_joints; ++i)
        {
            switch (jc.type[i])
            {
            case PRISMATIC:
                compute_prismatic_joint_errors(jc, i, bc, cc, time_step);
                break;
            case REVOLUTE:
                compute_revolute_joint_errors(jc, i, bc, cc, time_step);
                break;
            case FIXED:
                compute_fixed_joint_errors(jc, i, bc, cc, time_step);
                break;
            default:
                break;
            }
        }
    }

    /**
     * @brief Applies joint damping
     */
    void apply_joint_damping(JointCollection &jc,
                             BodyCollection &bc,
                             scalar time_step)
    {
        for (size_t i = 0; i < jc.n_joints; ++i)
        {
            switch (jc.type[i])
            {
            case PRISMATIC:
                apply_prismatic_joint_damping(jc, i, bc, time_step);
                break;
            case REVOLUTE:
                apply_revolute_joint_damping(jc, i, bc, time_step);
                break;
            default:
                break;
            }
        }
    }
}