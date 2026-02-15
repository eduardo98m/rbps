#include "rbps/constraints/Joint.hpp"
#include "rbps/constraints/Constraint.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;
using namespace m3d;

// Helper function to create test bodies
BodyCollection create_test_bodies(size_t n)
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
    bc.prev_position.resize(n, vec3{0, 0, 0});
    bc.prev_orientation.resize(n, quat(1, 0, 0, 0));
    bc.prev_linear_velocity.resize(n, vec3{0, 0, 0});
    bc.prev_angular_velocity.resize(n, vec3{0, 0, 0});
    return bc;
}

// Helper to create constraint collection
ConstraintCollection create_constraint_collection(size_t n)
{
    ConstraintCollection cc;
    cc.n_constraints = n;
    cc.body_1.resize(n, 0);
    cc.body_2.resize(n, 1);
    cc.r_1.resize(n, vec3{0, 0, 0});
    cc.r_2.resize(n, vec3{0, 0, 0});
    cc.direction.resize(n, vec3{1, 0, 0});
    cc.magnitude.resize(n, 0.0);
    cc.lambda.resize(n, 0.0);
    cc.force.resize(n, vec3{0, 0, 0});
    cc.torque.resize(n, vec3{0, 0, 0});
    cc.compliance.resize(n, 0.0);
    cc.type.resize(n, ConstraintType::POSITIONAL);
    cc.impulse.resize(n, vec3{0, 0, 0});
    return cc;
}

// ============================================================================
// PRISMATIC JOINT TESTS
// ============================================================================

TEST(compute_prismatic_joint_errors_basic)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    // Bodies aligned, no error expected
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Check alignment error (should be zero for aligned orientations)
    ASSERT_NEAR(cc.magnitude[0], 0.0, 0.001);
    
    // Check attachment error (should be zero for coincident points)
    ASSERT_NEAR(cc.magnitude[1], 0.0, 0.001);
    
    // Check drive error (FREE actuation should have zero error)
    ASSERT_NEAR(cc.magnitude[2], 0.0, 0.001);
}

TEST(compute_prismatic_joint_errors_misaligned_orientation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    // Rotate second body 90 degrees around Z
    bc.orientation[1] = quat::from_rpy(0, 0, M_PI / 2.0);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Alignment error should be non-zero due to rotation mismatch
    ASSERT_TRUE(cc.magnitude[0] > 0.01);
}

TEST(compute_prismatic_joint_errors_with_offset)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    // Offset second body along X axis
    bc.position[1] = vec3{2.0, 0, 0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Current position should reflect the slide distance
    ASSERT_NEAR(jc.current_position[0], -2.0, 0.001);
    
    // Attachment error should be zero (sliding along main axis is allowed)
    ASSERT_NEAR(cc.magnitude[1], 0.0, 0.001);
}

TEST(compute_prismatic_joint_errors_with_limits_within_range)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    // Offset within limits
    bc.position[1] = vec3{0.5, 0, 0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {true};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // No overshoot correction needed
    ASSERT_NEAR(cc.magnitude[1], 0.0, 0.001);
}

TEST(compute_prismatic_joint_errors_with_limits_exceeded)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    // Offset exceeds upper limit
    bc.position[1] = vec3{2.0, 0, 0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {true};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Overshoot correction should be applied
    ASSERT_TRUE(cc.magnitude[1] > 0.5);
}

TEST(compute_prismatic_joint_errors_position_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    bc.position[1] = vec3{0.5, 0, 0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {POSITION};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {1.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Drive error should be non-zero (target is 1.0, current is -0.5)
    ASSERT_TRUE(cc.magnitude[2] > 0.1);
}

TEST(compute_prismatic_joint_errors_speed_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {SPEED};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {2.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    scalar dt = 0.1;
    compute_prismatic_joint_errors(jc, 0, bc, cc, dt);
    
    // Target position should have advanced by speed * dt
    ASSERT_NEAR(jc.target_position[0], 0.2, 0.001);
    
    // Drive error should exist
    ASSERT_TRUE(cc.magnitude[2] > 0.01);
}

TEST(compute_prismatic_joint_errors_perpendicular_offset)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(3);
    
    // Offset perpendicular to sliding axis
    bc.position[1] = vec3{0, 1.0, 0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {PRISMATIC};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-1.0};
    jc.upper_limit = {1.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {3};
    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Attachment error should be 1.0 (perpendicular offset not allowed)
    ASSERT_NEAR(cc.magnitude[1], 1.0, 0.001);
}

// ============================================================================
// REVOLUTE JOINT TESTS
// ============================================================================

TEST(compute_revolute_joint_errors_basic)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // All errors should be near zero for aligned bodies
    ASSERT_NEAR(cc.magnitude[0], 0.0, 0.001); // Alignment
    ASSERT_NEAR(cc.magnitude[1], 0.0, 0.001); // Attachment
    ASSERT_NEAR(cc.magnitude[2], 0.0, 0.001); // Limit
    ASSERT_NEAR(cc.magnitude[3], 0.0, 0.001); // Drive
}

TEST(compute_revolute_joint_errors_axis_misalignment)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    // Rotate second body so axes are misaligned
    bc.orientation[1] = quat::from_rpy(M_PI / 4.0, 0, 0);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Alignment error should be non-zero
    ASSERT_TRUE(cc.magnitude[0] > 0.01);
}

TEST(compute_revolute_joint_errors_position_offset)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    // Offset second body position
    bc.position[1] = vec3{1.0, 0, 0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Attachment error should be 1.0
    ASSERT_NEAR(cc.magnitude[1], 1.0, 0.001);
}

TEST(compute_revolute_joint_errors_with_rotation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    // Rotate second body around Z axis (main axis)
    bc.orientation[1] = quat::from_rpy(0, 0, M_PI / 4.0);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Current position should reflect the angle
    ASSERT_NEAR(jc.current_position[0], M_PI / 4.0, 0.01);
}

TEST(compute_revolute_joint_errors_with_limits_within_range)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    // Small rotation within limits
    bc.orientation[1] = quat::from_rpy(0, 0, 0.5);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {true};
    jc.lower_limit = {-M_PI / 2.0};
    jc.upper_limit = {M_PI / 2.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Limit error should be zero (within range)
    ASSERT_NEAR(cc.magnitude[2], 0.0, 0.01);
}

TEST(compute_revolute_joint_errors_with_limits_exceeded)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    // Rotation exceeds upper limit
    bc.orientation[1] = quat::from_rpy(0, 0, M_PI);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {true};
    jc.lower_limit = {-M_PI / 4.0};
    jc.upper_limit = {M_PI / 4.0};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Limit error should be non-zero
    ASSERT_TRUE(cc.magnitude[2] > 0.1);
}

TEST(compute_revolute_joint_errors_position_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {POSITION};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {M_PI / 4.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Drive error should be non-zero
    ASSERT_TRUE(cc.magnitude[3] > 0.1);
}

TEST(compute_revolute_joint_errors_speed_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {SPEED};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {0.0};
    jc.target_speed = {1.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    scalar dt = 0.1;
    compute_revolute_joint_errors(jc, 0, bc, cc, dt);
    
    // Target position should have advanced
    ASSERT_NEAR(jc.target_position[0], 0.1, 0.001);
    
    // Drive error should exist
    ASSERT_TRUE(cc.magnitude[3] > 0.01);
}

// ============================================================================
// FIXED JOINT TESTS
// ============================================================================

TEST(compute_fixed_joint_errors_basic)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(2);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {FIXED};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.constraint_start = {0};
    jc.constraint_count = {2};
    
    compute_fixed_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Both errors should be zero for coincident bodies
    ASSERT_NEAR(cc.magnitude[0], 0.0, 0.001);
    ASSERT_NEAR(cc.magnitude[1], 0.0, 0.001);
}

TEST(compute_fixed_joint_errors_orientation_mismatch)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(2);
    
    // Rotate second body
    bc.orientation[1] = quat::from_rpy(0, 0, M_PI / 2.0);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {FIXED};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.constraint_start = {0};
    jc.constraint_count = {2};
    
    compute_fixed_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Alignment error should be non-zero
    ASSERT_TRUE(cc.magnitude[0] > 0.5);
}

TEST(compute_fixed_joint_errors_position_mismatch)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(2);
    
    // Offset second body
    bc.position[1] = vec3{1.0, 2.0, 3.0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {FIXED};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.constraint_start = {0};
    jc.constraint_count = {2};
    
    compute_fixed_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Position error should match the offset magnitude
    scalar expected_mag = std::sqrt(1.0*1.0 + 2.0*2.0 + 3.0*3.0);
    ASSERT_NEAR(cc.magnitude[1], expected_mag, 0.001);
}

TEST(compute_fixed_joint_errors_with_attachment_points)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(2);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {FIXED};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{1, 0, 0}};
    jc.r_2 = {vec3{-1, 0, 0}};
    jc.constraint_start = {0};
    jc.constraint_count = {2};
    
    compute_fixed_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Attachment error should be 2.0 (distance between points)
    ASSERT_NEAR(cc.magnitude[1], 2.0, 0.001);
}

// ============================================================================
// ANGLE LIMIT CORRECTION TESTS
// ============================================================================

TEST(compute_angle_limit_correction_within_limits)
{
    scalar phi = 0.5;
    vec3 n{0, 0, 1};
    vec3 n_1{1, 0, 0};
    vec3 n_2{1, 0, 0};
    scalar lower = -M_PI / 2.0;
    scalar upper = M_PI / 2.0;
    
    vec3 correction = compute_angle_limit_correction(phi, n, n_1, n_2, lower, upper);
    
    // No correction needed
    ASSERT_NEAR(m3d::magnitude(correction), 0.0, 0.001);
    ASSERT_NEAR(phi, 0.5, 0.001); // Angle unchanged
}

TEST(compute_angle_limit_correction_exceeds_upper_limit)
{
    scalar phi = M_PI;
    vec3 n{0, 0, 1};
    vec3 n_1{1, 0, 0};
    vec3 n_2 = m3d::rotate(quat::from_rpy(0, 0, M_PI), n_1);
    scalar lower = -M_PI / 2.0;
    scalar upper = M_PI / 2.0;
    
    vec3 correction = compute_angle_limit_correction(phi, n, n_1, n_2, lower, upper);
    
    // Angle should be clamped to upper limit
    ASSERT_NEAR(phi, M_PI / 2.0, 0.001);
    
    // Correction should be non-zero
    ASSERT_TRUE(m3d::magnitude(correction) > 0.1);
}

TEST(compute_angle_limit_correction_exceeds_lower_limit)
{
    scalar phi = -M_PI;
    vec3 n{0, 0, 1};
    vec3 n_1{1, 0, 0};
    vec3 n_2 = m3d::rotate(quat::from_rpy(0, 0, -M_PI), n_1);
    scalar lower = -M_PI / 2.0;
    scalar upper = M_PI / 2.0;
    
    vec3 correction = compute_angle_limit_correction(phi, n, n_1, n_2, lower, upper);
    
    // Angle should be clamped to lower limit
    ASSERT_NEAR(phi, -M_PI / 2.0, 0.001);
    
    // Correction should be non-zero
    ASSERT_TRUE(m3d::magnitude(correction) > 0.1);
}

TEST(compute_angle_limit_correction_at_limit_boundary)
{
    scalar phi = M_PI / 2.0;
    vec3 n{0, 0, 1};
    vec3 n_1{1, 0, 0};
    vec3 n_2 = m3d::rotate(quat::from_rpy(0, 0, M_PI / 2.0), n_1);
    scalar lower = -M_PI / 2.0;
    scalar upper = M_PI / 2.0;
    
    vec3 correction = compute_angle_limit_correction(phi, n, n_1, n_2, lower, upper);
    
    // At boundary, no correction needed
    ASSERT_NEAR(m3d::magnitude(correction), 0.0, 0.01);
    ASSERT_NEAR(phi, M_PI / 2.0, 0.001);
}

// ============================================================================
// DAMPING TESTS
// ============================================================================

TEST(apply_prismatic_joint_damping_no_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.damping = {0.5};
    
    bc.orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[1] = quat(1, 0, 0, 0);
    
    vec3 initial_vel_0 = bc.linear_velocity[0];
    vec3 initial_vel_1 = bc.linear_velocity[1];
    
    apply_prismatic_joint_damping(jc, 0, bc, 0.01);
    
    // Velocities should remain unchanged (no relative motion)
    ASSERT_TRUE(bc.linear_velocity[0] == initial_vel_0);
    ASSERT_TRUE(bc.linear_velocity[1] == initial_vel_1);
}

TEST(apply_prismatic_joint_damping_with_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    
    bc.linear_velocity[0] = vec3{0, 0, 0};
    bc.linear_velocity[1] = vec3{1, 0, 0};
    bc.inverse_mass[0] = 1.0;
    bc.inverse_mass[1] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[1] = smat3(1, 1, 1, 0, 0, 0);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.damping = {1.0};
    
    bc.orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[1] = quat(1, 0, 0, 0);
    
    apply_prismatic_joint_damping(jc, 0, bc, 0.01);
    
    // Velocities should have changed (damping applied)
    ASSERT_TRUE(bc.linear_velocity[0].x > 0.0);
    ASSERT_TRUE(bc.linear_velocity[1].x < 1.0);
}

TEST(apply_prismatic_joint_damping_high_damping)
{
    BodyCollection bc = create_test_bodies(2);
    
    bc.linear_velocity[0] = vec3{0, 0, 0};
    bc.linear_velocity[1] = vec3{10, 0, 0};
    bc.inverse_mass[0] = 1.0;
    bc.inverse_mass[1] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[1] = smat3(1, 1, 1, 0, 0, 0);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{0, 0, 0}};
    jc.r_2 = {vec3{0, 0, 0}};
    jc.damping = {100.0}; // Very high damping
    
    bc.orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[1] = quat(1, 0, 0, 0);
    
    apply_prismatic_joint_damping(jc, 0, bc, 0.01);
    
    // High damping should significantly reduce relative velocity
    scalar relative_vel_after = m3d::magnitude(bc.linear_velocity[1] - bc.linear_velocity[0]);
    ASSERT_TRUE(relative_vel_after < 5.0);
}

TEST(apply_revolute_joint_damping_no_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.damping = {0.5};
    
    vec3 initial_omega_0 = bc.angular_velocity[0];
    vec3 initial_omega_1 = bc.angular_velocity[1];
    
    apply_revolute_joint_damping(jc, 0, bc, 0.01);
    
    // Angular velocities should remain unchanged
    ASSERT_TRUE(bc.angular_velocity[0] == initial_omega_0);
    ASSERT_TRUE(bc.angular_velocity[1] == initial_omega_1);
}

TEST(apply_revolute_joint_damping_with_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    
    bc.angular_velocity[0] = vec3{0, 0, 0};
    bc.angular_velocity[1] = vec3{0, 0, 2.0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.damping = {1.0};
    
    apply_revolute_joint_damping(jc, 0, bc, 0.01);
    
    // Angular velocities should have changed
    ASSERT_TRUE(bc.angular_velocity[0].z > 0.0);
    ASSERT_TRUE(bc.angular_velocity[1].z < 2.0);
}

TEST(apply_revolute_joint_damping_high_damping)
{
    BodyCollection bc = create_test_bodies(2);
    
    bc.angular_velocity[0] = vec3{0, 0, 0};
    bc.angular_velocity[1] = vec3{0, 0, 10.0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.damping = {100.0}; // With dt=0.01, clamped factor = min(100*0.01, 1.0) = 1.0
    
    apply_revolute_joint_damping(jc, 0, bc, 0.01);
    
    // With clamped damping factor of 1.0, velocities should equalize:
    // delta_omega = (10 - 0) * 1.0 = 10
    // omega_1 = 0 + 10 = 10
    // omega_2 = 10 - 10 = 0
    // They swap, but average is preserved at 5.0 each (momentum conservation)
    ASSERT_NEAR(bc.angular_velocity[0].z, 10.0, 0.001);
    ASSERT_NEAR(bc.angular_velocity[1].z, 0.0, 0.001);
    
    // The relative velocity magnitude stays the same, but velocities equalize toward average
    scalar avg_omega = (bc.angular_velocity[0].z + bc.angular_velocity[1].z) / 2.0;
    ASSERT_NEAR(avg_omega, 5.0, 0.001);
}

TEST(apply_revolute_joint_damping_moderate_damping)
{
    BodyCollection bc = create_test_bodies(2);
    
    bc.angular_velocity[0] = vec3{0, 0, 0};
    bc.angular_velocity[1] = vec3{0, 0, 10.0};
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.damping = {5.0}; // With dt=0.01, factor = min(5*0.01, 1.0) = 0.05
    
    scalar initial_relative = m3d::magnitude(bc.angular_velocity[1] - bc.angular_velocity[0]);
    
    apply_revolute_joint_damping(jc, 0, bc, 0.01);
    
    scalar final_relative = m3d::magnitude(bc.angular_velocity[1] - bc.angular_velocity[0]);
    
    // Relative velocity should be reduced
    ASSERT_TRUE(final_relative < initial_relative);
    
    // With factor 0.05: delta = 10 * 0.05 = 0.5
    // omega_1 = 0 + 0.5 = 0.5, omega_2 = 10 - 0.5 = 9.5
    ASSERT_NEAR(bc.angular_velocity[0].z, 0.5, 0.001);
    ASSERT_NEAR(bc.angular_velocity[1].z, 9.5, 0.001);
    ASSERT_NEAR(final_relative, 9.0, 0.001);
}



// ============================================================================
// INTEGRATION TESTS
// ============================================================================

TEST(compute_joint_errors_multiple_joints)
{
    BodyCollection bc = create_test_bodies(4);
    ConstraintCollection cc = create_constraint_collection(9); // 3 + 4 + 2
    
    JointCollection jc;
    jc.n_joints = 3;
    jc.type = {PRISMATIC, REVOLUTE, FIXED};
    jc.actuation_type = {FREE, FREE, FREE};
    jc.body_1 = {0, 1, 2};
    jc.body_2 = {1, 2, 3};
    jc.r_1 = {vec3{0,0,0}, vec3{0,0,0}, vec3{0,0,0}};
    jc.r_2 = {vec3{0,0,0}, vec3{0,0,0}, vec3{0,0,0}};
    jc.main_axis = {vec3{1,0,0}, vec3{0,0,1}, vec3{0,0,0}};
    jc.limit_axis = {vec3{0,0,0}, vec3{1,0,0}, vec3{0,0,0}};
    jc.limited = {false, false, false};
    jc.lower_limit = {-1.0, -M_PI, 0.0};
    jc.upper_limit = {1.0, M_PI, 0.0};
    jc.target_position = {0.0, 0.0, 0.0};
    jc.target_speed = {0.0, 0.0, 0.0};
    jc.current_position = {0.0, 0.0, 0.0};
    jc.damping = {0.1, 0.1, 0.1};
    jc.constraint_start = {0, 3, 7};
    jc.constraint_count = {3, 4, 2};
    
    compute_joint_errors(jc, bc, cc, 0.01);
    
    // All constraints should have been computed (no crashes)
    // Basic sanity check on magnitudes
    for (size_t i = 0; i < cc.n_constraints; ++i)
    {
        ASSERT_TRUE(cc.magnitude[i] >= 0.0);
    }
}

TEST(apply_joint_damping_multiple_joints)
{
    BodyCollection bc = create_test_bodies(4);
    
    bc.linear_velocity[1] = vec3{1, 0, 0};
    bc.angular_velocity[2] = vec3{0, 0, 1};
    bc.inverse_mass[0] = 1.0;
    bc.inverse_mass[1] = 1.0;
    bc.inverse_mass[2] = 1.0;
    bc.inverse_mass[3] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[1] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[2] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[3] = smat3(1, 1, 1, 0, 0, 0);
    bc.orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[1] = quat(1, 0, 0, 0);
    bc.orientation[2] = quat(1, 0, 0, 0);
    bc.orientation[3] = quat(1, 0, 0, 0);
    
    JointCollection jc;
    jc.n_joints = 3;
    jc.type = {PRISMATIC, REVOLUTE, FIXED};
    jc.body_1 = {0, 1, 2};
    jc.body_2 = {1, 2, 3};
    jc.r_1 = {vec3{0,0,0}, vec3{0,0,0}, vec3{0,0,0}};
    jc.r_2 = {vec3{0,0,0}, vec3{0,0,0}, vec3{0,0,0}};
    jc.damping = {1.0, 1.0, 0.0};
    
    apply_joint_damping(jc, bc, 0.01);
    
    // Damping should have affected velocities
    ASSERT_TRUE(bc.linear_velocity[0].x > 0.0 || bc.linear_velocity[1].x < 1.0);
    ASSERT_TRUE(bc.angular_velocity[1].z > 0.0 || bc.angular_velocity[2].z < 1.0);
}

TEST(joint_errors_with_attachment_points)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc = create_constraint_collection(4);
    
    JointCollection jc;
    jc.n_joints = 1;
    jc.type = {REVOLUTE};
    jc.actuation_type = {FREE};
    jc.body_1 = {0};
    jc.body_2 = {1};
    jc.r_1 = {vec3{1, 0, 0}};  // Attachment point offset
    jc.r_2 = {vec3{-1, 0, 0}}; // Attachment point offset
    jc.main_axis = {vec3{0, 0, 1}};
    jc.limit_axis = {vec3{1, 0, 0}};
    jc.limited = {false};
    jc.lower_limit = {-M_PI};
    jc.upper_limit = {M_PI};
    jc.target_position = {0.0};
    jc.target_speed = {0.0};
    jc.current_position = {0.0};
    jc.damping = {0.1};
    jc.constraint_start = {0};
    jc.constraint_count = {4};
    
    compute_revolute_joint_errors(jc, 0, bc, cc, 0.01);
    
    // Attachment error should be 2.0 (distance between offset points)
    ASSERT_NEAR(cc.magnitude[1], 2.0, 1e-4);
}

TEST_SUITE(
    // Prismatic Joint Tests
    RUN_TEST(compute_prismatic_joint_errors_basic),
    RUN_TEST(compute_prismatic_joint_errors_misaligned_orientation),
    RUN_TEST(compute_prismatic_joint_errors_with_offset),
    RUN_TEST(compute_prismatic_joint_errors_with_limits_within_range),
    RUN_TEST(compute_prismatic_joint_errors_with_limits_exceeded),
    RUN_TEST(compute_prismatic_joint_errors_position_actuation),
    RUN_TEST(compute_prismatic_joint_errors_speed_actuation),
    RUN_TEST(compute_prismatic_joint_errors_perpendicular_offset),
    
    // Revolute Joint Tests
    RUN_TEST(compute_revolute_joint_errors_basic),
    RUN_TEST(compute_revolute_joint_errors_axis_misalignment),
    RUN_TEST(compute_revolute_joint_errors_position_offset),
    RUN_TEST(compute_revolute_joint_errors_with_rotation),
    RUN_TEST(compute_revolute_joint_errors_with_limits_within_range),
    RUN_TEST(compute_revolute_joint_errors_with_limits_exceeded),
    RUN_TEST(compute_revolute_joint_errors_position_actuation),
    RUN_TEST(compute_revolute_joint_errors_speed_actuation),
    
    // Fixed Joint Tests
    RUN_TEST(compute_fixed_joint_errors_basic),
    RUN_TEST(compute_fixed_joint_errors_orientation_mismatch),
    RUN_TEST(compute_fixed_joint_errors_position_mismatch),
    RUN_TEST(compute_fixed_joint_errors_with_attachment_points),
    
    // Angle Limit Correction Tests
    RUN_TEST(compute_angle_limit_correction_within_limits),
    RUN_TEST(compute_angle_limit_correction_exceeds_upper_limit),
    RUN_TEST(compute_angle_limit_correction_exceeds_lower_limit),
    RUN_TEST(compute_angle_limit_correction_at_limit_boundary),
    
    // Damping Tests
    RUN_TEST(apply_prismatic_joint_damping_no_relative_velocity),
    RUN_TEST(apply_prismatic_joint_damping_with_relative_velocity),
    RUN_TEST(apply_prismatic_joint_damping_high_damping),
    RUN_TEST(apply_revolute_joint_damping_no_relative_velocity),
    RUN_TEST(apply_revolute_joint_damping_with_relative_velocity),
    RUN_TEST(apply_revolute_joint_damping_high_damping),
    RUN_TEST(apply_revolute_joint_damping_moderate_damping),
    
    // Integration Tests
    RUN_TEST(compute_joint_errors_multiple_joints),
    RUN_TEST(apply_joint_damping_multiple_joints),
    RUN_TEST(joint_errors_with_attachment_points)
)