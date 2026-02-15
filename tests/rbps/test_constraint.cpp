#include "rbps/constraints/Constraint.hpp"
#include "rbps/Body.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;
using namespace m3d;

BodyCollection create_bodies(size_t n)
{
    BodyCollection bc;
    bc.n_bodies = n;
    bc.force.resize(n, vec3{0, 0, 0});
    bc.torque.resize(n, vec3{0, 0, 0});
    bc.mass.resize(n, 1.0);
    bc.inverse_mass.resize(n, 1.0);
    bc.inertia_tensor.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.inverse_inertia_tensor.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.inertia_tensor_world.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.inverse_inertia_tensor_world.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.type.resize(n, BodyType::DYNAMIC);
    bc.position.resize(n, vec3{0, 0, 0});
    bc.orientation.resize(n, quat(1, 0, 0, 0));
    bc.linear_velocity.resize(n, vec3{0, 0, 0});
    bc.angular_velocity.resize(n, vec3{0, 0, 0});
    return bc;
}

ConstraintCollection create_constraints(size_t n)
{
    ConstraintCollection cc;
    cc.n_constraints = n;
    cc.body_1.resize(n, 0);
    cc.body_2.resize(n, 0);
    cc.r_1.resize(n, vec3{0, 0, 0});
    cc.r_2.resize(n, vec3{0, 0, 0});
    cc.direction.resize(n, vec3{1, 0, 0});
    cc.magnitude.resize(n, 0.0);
    cc.lambda.resize(n, 0.0);
    cc.force.resize(n, vec3{0, 0, 0});
    cc.torque.resize(n, vec3{0, 0, 0});
    cc.compliance.resize(n, 0.0); // Rigid by default
    cc.type.resize(n, ConstraintType::POSITIONAL);
    cc.impulse.resize(n, vec3{0, 0, 0});
    return cc;
}

// --- Tests ---

TEST(set_value_normal)
{
    ConstraintCollection cc = create_constraints(1);

    // A vector with magnitude 5 along X=3, Y=4
    vec3 val{3.0, 4.0, 0.0};

    set_value(cc, 0, val);

    ASSERT_NEAR(cc.magnitude[0], 5.0, 1e-5);
    // Direction should be normalized (0.6, 0.8, 0.0)
    ASSERT_NEAR(cc.direction[0].x, 0.6, 1e-5);
    ASSERT_NEAR(cc.direction[0].y, 0.8, 1e-5);
}

TEST(set_value_epsilon)
{
    ConstraintCollection cc = create_constraints(1);
    vec3 val{0.0, 0.0, 0.0};
    set_value(cc, 0, val);
    ASSERT_NEAR(cc.magnitude[0], 0.0, 1e-5);
    ASSERT_NEAR(cc.direction[0].x, 1.0, 1e-5);
}

TEST(compute_delta_lambda_pure_math)
{
    ConstraintCollection cc = create_constraints(1);

    cc.magnitude[0] = 0.5;  // Constraint violation C
    cc.compliance[0] = 0.0; // Alpha = 0
    cc.lambda[0] = 0.0;

    scalar w1 = 1.0;
    scalar w2 = 1.0;
    scalar inv_dt = 60.0;

    // Formula: (-C - alpha * lambda) / (w1 + w2 + alpha)
    // (-0.5 - 0) / (1 + 1 + 0) = -0.25
    scalar d_lambda = compute_delta_lambda(cc, 0, w1, w2, inv_dt);

    ASSERT_NEAR(d_lambda, -0.25, 1e-5);
}

TEST(compute_delta_lambda_with_compliance)
{
    ConstraintCollection cc = create_constraints(1);

    scalar dt = 0.1;
    scalar inv_dt = 10.0;

    cc.magnitude[0] = 1.0;
    cc.compliance[0] = 0.02;
    // alpha = compliance * inv_dt^2 = 0.02 * 100 = 2.0

    scalar w1 = 1.0;
    scalar w2 = 1.0;

    // d_lambda = (-1.0 - 2.0*0) / (1.0 + 1.0 + 2.0) = -1.0 / 4.0 = -0.25
    scalar d_lambda = compute_delta_lambda(cc, 0, w1, w2, inv_dt);

    ASSERT_NEAR(d_lambda, -0.25, 1e-5);
}

TEST(positional_constraint_updates_position)
{
    BodyCollection bc = create_bodies(2);
    ConstraintCollection cc = create_constraints(1);

    // Body 1 at (0,0,0), Body 2 at (1,0,0)
    bc.position[1] = vec3{1.0, 0.0, 0.0};
    bc.mass[0] = 1.0;
    bc.inverse_mass[0] = 1.0;
    bc.mass[1] = 1.0;
    bc.inverse_mass[1] = 1.0;

    cc.body_1[0] = 0;
    cc.body_2[0] = 1;
    cc.type[0] = ConstraintType::POSITIONAL;
    cc.r_1[0] = vec3{0, 0, 0};
    cc.r_2[0] = vec3{0, 0, 0};

    // Constraint error vector is (1,0,0)
    set_value(cc, 0, vec3{1.0, 0.0, 0.0});

    scalar inv_dt = 10.0;

    compute_positional_constraint_impulse(bc, cc, 0, inv_dt);

    // Lambda should update
    ASSERT_TRUE(cc.lambda[0] < 0.0);
    ASSERT_TRUE(bc.position[0].x < 0.0);
    ASSERT_TRUE(bc.position[1].x > 1.0);
}

TEST(rotational_constraint_updates_orientation)
{
    BodyCollection bc = create_bodies(2);
    ConstraintCollection cc = create_constraints(1);

    cc.body_1[0] = 0;
    cc.body_2[0] = 1;
    cc.type[0] = ConstraintType::ROTATIONAL;

    // Error rotation around Z
    set_value(cc, 0, vec3{0.0, 0.0, 0.1});

    scalar inv_dt = 10.0;

    // Store initial orientations (identity)
    quat q0_initial = bc.orientation[0];
    quat q1_initial = bc.orientation[1];

    compute_rotational_constraint_impulse(bc, cc, 0, inv_dt);

    // Body 1 should rotate
    ASSERT_FALSE(bc.orientation[0] == q0_initial);
    ASSERT_TRUE(std::abs(bc.orientation[0].z) > 1e-6);

    // Body 2 should rotate
    ASSERT_FALSE(bc.orientation[1] == q1_initial);
    ASSERT_TRUE(std::abs(bc.orientation[1].z) > 1e-6);

    bool signs_differ = (bc.orientation[0].z * bc.orientation[1].z) < 0;
    ASSERT_TRUE(signs_differ);
}

TEST(reset_lagrange)
{
    ConstraintCollection cc = create_constraints(1);
    cc.lambda[0] = 5.0;

    reset_lagrange_multiplier(cc, 0);
    std::cout << "Lambda after reset: " << cc.lambda[0] << std::endl;
    ASSERT_NEAR(cc.lambda[0], 0.0, 1e-6);
}

TEST(solve_constraints_loop)
{
    BodyCollection bc = create_bodies(2);
    ConstraintCollection cc = create_constraints(2);

    // Setup two constraints
    cc.type[0] = ConstraintType::POSITIONAL;
    cc.body_1[0] = 0;
    cc.body_2[0] = 1;
    cc.magnitude[0] = 1.0;
    cc.direction[0] = vec3{1, 0, 0};

    cc.type[1] = ConstraintType::ROTATIONAL;
    cc.body_1[1] = 0;
    cc.body_2[1] = 1;
    cc.magnitude[1] = 0.5;
    cc.direction[1] = vec3{0, 1, 0};

    solve_constraints(bc, cc, 60.0);

    // Both lambdas should have been touched (non-zero)
    ASSERT_TRUE(std::abs(cc.lambda[0]) > 1e-6);
    ASSERT_TRUE(std::abs(cc.lambda[1]) > 1e-6);
}

TEST_SUITE(
    RUN_TEST(set_value_normal),
    RUN_TEST(set_value_epsilon),
    RUN_TEST(compute_delta_lambda_pure_math),
    RUN_TEST(compute_delta_lambda_with_compliance),
    RUN_TEST(positional_constraint_updates_position),
    RUN_TEST(rotational_constraint_updates_orientation),
    RUN_TEST(reset_lagrange),
    RUN_TEST(solve_constraints_loop))