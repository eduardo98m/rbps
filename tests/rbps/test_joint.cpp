#include "rbps/constraints/Joint.hpp"
#include "rbps/constraints/Constraint.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;
using namespace m3d;

// Helper function to create test bodies
static BodyCollection create_test_bodies(uint32_t n)
{
    BodyCollection bc;
    for (uint32_t k = 0; k < n; ++k)
    {
        uint32_t i = bc.index_of(bc.add());
        bc.mass[i] = 1.0;
        bc.inverse_mass[i] = 1.0;
        bc.type[i] = BodyType::DYNAMIC;
        bc.position[i] = vec3{0, 0, 0};
        bc.linear_velocity[i] = vec3{0, 0, 0};
        bc.orientation[i] = quat(1, 0, 0, 0);
        bc.angular_velocity[i] = vec3{0, 0, 0};
        bc.prev_orientation[i] = quat(1, 0, 0, 0);
        bc.inertia_tensor[i] = smat3(1, 1, 1, 0, 0, 0);
        bc.inverse_inertia_tensor[i] = smat3(1, 1, 1, 0, 0, 0);
        bc.inertia_tensor_world[i] = smat3(1, 1, 1, 0, 0, 0);
        bc.inverse_inertia_tensor_world[i] = smat3(1, 1, 1, 0, 0, 0);
    }
    return bc;
}



// Helper to create constraint collection
static ConstraintCollection create_constraint_collection(uint32_t n)
{
    ConstraintCollection cc;
    for (uint32_t k = 0; k < n; ++k)
    {
        int32_t i = cc.index_of(cc.add());
        cc.body_1[i] = 0;
        cc.body_2[i] = 0;
        cc.r_1[i] = vec3{0, 0, 0};
        cc.r_2[i] = vec3{0, 0, 0};
        cc.direction[i] = vec3{1, 0, 0};
        cc.magnitude[i] = 0.0;
        cc.lambda[i] = 0.0;
        cc.force[i] = vec3{0, 0, 0};
        cc.torque[i] = vec3{0, 0, 0};
        cc.compliance[i] = 0.0;
        cc.type[i] = ConstraintType::POSITIONAL;
        cc.impulse[i] = vec3{0, 0, 0};        
    }
    return cc;
}

static std::vector<uint32_t> add_test_constraints(ConstraintCollection& cc, uint32_t n)
{
    std::vector<uint32_t> ids;
    ids.reserve(n);
    
    for (uint32_t k = 0; k < n; ++k)
    {
        // 1. Allocate a new element in the SoA and get its stable ID
        uint32_t id = cc.add();
        ids.push_back(id);
        
        // 2. Resolve the ID to the current memory index
        uint32_t i = cc.index_of(id);
        
        // 3. Initialize all data fields to safe defaults
        cc.body_1[i]      = 0;
        cc.body_2[i]      = 0;
        cc.r_1[i]         = vec3{0, 0, 0};
        cc.r_2[i]         = vec3{0, 0, 0};
        cc.direction[i]   = vec3{1, 0, 0}; // Default axis
        cc.magnitude[i]   = 0.0;
        cc.lambda[i]      = 0.0;
        cc.force[i]       = vec3{0, 0, 0};
        cc.torque[i]      = vec3{0, 0, 0};
        cc.compliance[i]  = 0.0;
        cc.type[i]        = ConstraintType::POSITIONAL;
        cc.impulse[i]     = vec3{0, 0, 0};
    }
    return ids;
}

// -------------------------------------------------------------
// Test utilities for creating joints
// -------------------------------------------------------------

static uint32_t create_prismatic_joint(
    JointCollection& jc,
    uint32_t body1,
    uint32_t body2,
    const m3d::vec3& axis)
{
    uint32_t jid = jc.add();
    uint32_t i = jc.index_of(jid);

    jc.type[i] = PRISMATIC;
    jc.body_1[i] = body1;
    jc.body_2[i] = body2;
    jc.main_axis[i] = axis;

    jc.target_position[i] = 0.0;
    jc.target_speed[i] = 0.0;

    jc.lower_limit[i] = -std::numeric_limits<double>::infinity();
    jc.upper_limit[i] = std::numeric_limits<double>::infinity();

    jc.constraint_count[i] = 3; // 1 for rotation lock, 2 for positional constraints

    return jid;
}

static uint32_t create_revolute_joint(
    JointCollection& jc,
    uint32_t body1,
    uint32_t body2,
    const m3d::vec3& axis)
{
    uint32_t jid = jc.add();
    uint32_t i = jc.index_of(jid);

    jc.type[i] = REVOLUTE;
    jc.body_1[i] = body1;
    jc.body_2[i] = body2;
    jc.main_axis[i] = axis;

    jc.target_position[i] = 0.0;
    jc.target_speed[i] = 0.0;

    jc.lower_limit[i] = -std::numeric_limits<double>::infinity();
    jc.upper_limit[i] = std::numeric_limits<double>::infinity();

    jc.constraint_count[i] = 4; // 2 for rotation lock, 2 for attachment

    return jid;
}

static uint32_t create_fixed_joint(
    JointCollection& jc,
    uint32_t body1,
    uint32_t body2)
{
    uint32_t jid = jc.add();
    uint32_t i = jc.index_of(jid);

    jc.type[i] = FIXED;
    jc.body_1[i] = body1;
    jc.body_2[i] = body2;
    jc.constraint_count[i] = 2; // 1 rotational + 1 positional constraint for full lock

    return jid;
}
// ============================================================================
// PRISMATIC JOINT TESTS
// ============================================================================

TEST(compute_prismatic_joint_errors_basic)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    
    // 1. Create the 3 constraints needed and keep their IDs
    auto c_ids = add_test_constraints(cc, 3);

    // 2. Create the joint using the SoA API
    JointCollection jc;
    uint32_t j_id = jc.add();
    uint32_t i = jc.index_of(j_id);
    
    // 3. Populate joint data at index 'i'
    jc.type[i]            = PRISMATIC;
    jc.actuation_type[i]  = FREE;
    jc.body_1[i]          = 0; // Assuming body indices for now
    jc.body_2[i]          = 1;
    jc.main_axis[i]       = vec3{1, 0, 0};
    jc.damping[i]         = 0.1;
    jc.constraint_count[i]= 3;
    
    // 4. LINK: Store the stable IDs in the joint's constraint block
    for (int k = 0; k < 3; ++k) {
        jc.constraints[i].ids[k] = c_ids[k];
    }
    
    // 5. Run the calculation
    compute_prismatic_joint_errors(jc, i, bc, cc, 0.01);
    
    // 6. VERIFY: Lookup the result using the stable ID
    // Don't use cc.magnitude[0], use cc.index_of(id)
    ASSERT_NEAR(cc.magnitude[cc.index_of(c_ids[0])], 0.0, 0.001);
    ASSERT_NEAR(cc.magnitude[cc.index_of(c_ids[1])], 0.0, 0.001);
    ASSERT_NEAR(cc.magnitude[cc.index_of(c_ids[2])], 0.0, 0.001);
}

TEST(compute_prismatic_joint_errors_misaligned_orientation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);
    
    // Rotate second body 90 degrees around Z
    bc.orientation[1] = quat::from_rpy(0, 0, M_PI / 2.0);
    
    JointCollection jc;
    uint32_t i = jc.index_of(jc.add());
    
    jc.type[i]            = PRISMATIC;
    jc.actuation_type[i]  = FREE;
    jc.body_1[i]          = 0;
    jc.body_2[i]          = 1;
    jc.main_axis[i]       = vec3{1, 0, 0};
    for(int k=0; k<3; ++k) jc.constraints[i].ids[k] = c_ids[k];
    
    compute_prismatic_joint_errors(jc, i, bc, cc, 0.01);
    
    // Alignment error (slot 0) should be non-zero
    ASSERT_TRUE(cc.magnitude[cc.index_of(c_ids[0])] > 0.01);
}

TEST(compute_prismatic_joint_errors_with_offset)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);
    
    bc.position[1] = vec3{2.0, 0, 0}; // Offset 2m along X
    
    JointCollection jc;
    uint32_t i = jc.index_of(jc.add());
    jc.type[i]            = PRISMATIC;
    jc.body_1[i]          = 0;
    jc.body_2[i]          = 1;
    jc.main_axis[i]       = vec3{1, 0, 0};
    for(int k=0; k<3; ++k) jc.constraints[i].ids[k] = c_ids[k];
    
    compute_prismatic_joint_errors(jc, i, bc, cc, 0.01);
    
    // Current position should reflect slide distance
    ASSERT_NEAR(jc.current_position[i], -2.0, 0.001);
    // Attachment error (slot 1) should be zero (sliding is allowed)
    ASSERT_NEAR(cc.magnitude[cc.index_of(c_ids[1])], 0.0, 0.001);
}

TEST(compute_prismatic_joint_errors_with_limits_within_range)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);

    
    // Offset within limits
    bc.position[1] = vec3{0.5, 0, 0};
    
    JointCollection jc;
    uint32_t i = jc.index_of(jc.add());
    jc.type[i] = {PRISMATIC};
    jc.actuation_type[i] = {FREE};
    jc.body_1[i] = {0};
    jc.body_2[i] = {1};
    jc.r_1[i] = {vec3{0, 0, 0}};
    jc.r_2[i] = {vec3{0, 0, 0}};
    jc.main_axis[i] = {vec3{1, 0, 0}};
    jc.limited[i] = {true};
    jc.lower_limit[i] = {-1.0};
    jc.upper_limit[i] = {1.0};
    jc.target_position[i] = {0.0};
    jc.target_speed[i] = {0.0};
    jc.current_position[i] = {0.0};
    jc.damping[i] = {0.1};
    jc.constraint_count[i] = {3};
    for(int k=0; k<3; ++k) jc.constraints[i].ids[k] = c_ids[k];

    
    compute_prismatic_joint_errors(jc, 0, bc, cc, 0.01);
    
    // No overshoot correction needed
    ASSERT_NEAR(cc.magnitude[1], 0.0, 0.001);
}

TEST(compute_prismatic_joint_errors_with_limits_exceeded)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);

    // Offset exceeds upper limit
    bc.position[1] = vec3{2.0, 0, 0};

    JointCollection jc;
    uint32_t jid = create_prismatic_joint(jc, 0, 1, vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.actuation_type[i] = FREE;
    jc.limited[i] = true;
    jc.lower_limit[i] = -1.0;
    jc.upper_limit[i] = 1.0;

    for(int k=0;k<3;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_prismatic_joint_errors(jc, i, bc, cc, 0.01);

    // Overshoot correction should be applied
    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[1])] > 0.5
    );
}

TEST(compute_prismatic_joint_errors_position_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);

    bc.position[1] = vec3{0.5, 0, 0};

    JointCollection jc;
    uint32_t jid = create_prismatic_joint(jc,0,1,vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.actuation_type[i] = POSITION;
    jc.target_position[i] = 1.0;

    for(int k=0;k<3;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_prismatic_joint_errors(jc, i, bc, cc, 0.01);

    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[2])] > 0.1
    );
}

TEST(compute_prismatic_joint_errors_speed_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);

    JointCollection jc;
    uint32_t jid = create_prismatic_joint(jc,0,1,vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.actuation_type[i] = SPEED;
    jc.target_speed[i] = 2.0;

    for(int k=0;k<3;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    scalar dt = 0.1;

    compute_prismatic_joint_errors(jc, i, bc, cc, dt);

    ASSERT_NEAR(jc.target_position[i], 0.2, 0.001);

    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[2])] > 0.01
    );
}

TEST(compute_prismatic_joint_errors_perpendicular_offset)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 3);

    // Offset perpendicular to sliding axis
    bc.position[1] = vec3{0, 1.0, 0};

    JointCollection jc;
    uint32_t jid = create_prismatic_joint(jc, 0, 1, vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.actuation_type[i] = FREE;
    jc.limited[i] = false;

    // Attach constraints to the joint
    for(int k=0;k<3;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_prismatic_joint_errors(jc, i, bc, cc, 0.01);

    // Attachment error should be 1.0 (perpendicular offset not allowed)
    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[1])],
        1.0,
        0.001
    );
}

// ============================================================================
// REVOLUTE JOINT TESTS
// ============================================================================

TEST(compute_revolute_joint_errors_basic)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4); 
    
    JointCollection jc;
    uint32_t i = jc.index_of(jc.add());
    jc.type[i]            = REVOLUTE;
    jc.body_1[i]          = 0;
    jc.body_2[i]          = 1;
    jc.main_axis[i]       = vec3{0, 0, 1};
    jc.limit_axis[i]      = vec3{1, 0, 0};
    for(int k=0; k<4; ++k) jc.constraints[i].ids[k] = c_ids[k];
    
    compute_revolute_joint_errors(jc, i, bc, cc, 0.01);
    
    // All error magnitudes should be zero
    for(uint32_t id : c_ids) {
        ASSERT_NEAR(cc.magnitude[cc.index_of(id)], 0.0, 0.001);
    }
}

TEST(compute_revolute_joint_errors_axis_misalignment)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    bc.orientation[1] = quat::from_rpy(M_PI/4.0,0,0);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i   = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc,i,bc,cc,0.01);

    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[0])] > 0.01
    );
}

TEST(compute_revolute_joint_errors_position_offset)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    // Offset second body position
    bc.position[1] = vec3{1.0, 0, 0};

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc, 0, 1, vec3{0,0,1});
    uint32_t i = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc, i, bc, cc, 0.01);

    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[1])],
        1.0,
        0.001
    );
}

TEST(compute_revolute_joint_errors_with_rotation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    bc.orientation[1] = quat::from_rpy(0,0,M_PI/4.0);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc,i,bc,cc,0.01);

    ASSERT_NEAR(
        jc.current_position[i],
        M_PI/4.0,
        0.01
    );
}

TEST(compute_revolute_joint_errors_with_limits_within_range)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    bc.orientation[1] = quat::from_rpy(0,0,0.5);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};
    jc.limited[i] = true;
    jc.lower_limit[i] = -M_PI/2.0;
    jc.upper_limit[i] =  M_PI/2.0;

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc,i,bc,cc,0.01);

    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[2])],
        0.0,
        0.01
    );
}

TEST(compute_revolute_joint_errors_with_limits_exceeded)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    bc.orientation[1] = quat::from_rpy(0,0,M_PI);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};
    jc.limited[i] = true;
    jc.lower_limit[i] = -M_PI/4.0;
    jc.upper_limit[i] =  M_PI/4.0;

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc,i,bc,cc,0.01);

    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[2])] > 0.1
    );
}

TEST(compute_revolute_joint_errors_position_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};
    jc.actuation_type[i] = POSITION;
    jc.target_position[i] = M_PI/4.0;

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc,i,bc,cc,0.01);

    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[3])] > 0.1
    );
}

TEST(compute_revolute_joint_errors_speed_actuation)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i = jc.index_of(jid);

    jc.limit_axis[i] = vec3{1,0,0};
    jc.actuation_type[i] = SPEED;
    jc.target_speed[i] = 1.0;

    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    scalar dt = 0.1;

    compute_revolute_joint_errors(jc,i,bc,cc,dt);

    ASSERT_NEAR(
        jc.target_position[i],
        0.1,
        0.001
    );

    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[3])] > 0.01
    );
}

// ============================================================================
// FIXED JOINT TESTS
// ============================================================================

TEST(compute_fixed_joint_errors_basic)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc,2);

    JointCollection jc;
    uint32_t jid = create_fixed_joint(jc,0,1);
    uint32_t i   = jc.index_of(jid);

    for(int k=0;k<2;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_fixed_joint_errors(jc,i,bc,cc,0.01);

    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[0])],
        0.0,
        0.001
    );

    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[1])],
        0.0,
        0.001
    );
}

TEST(compute_fixed_joint_errors_orientation_mismatch)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 2);

    // Rotate second body
    bc.orientation[1] = quat::from_rpy(0, 0, M_PI / 2.0);

    JointCollection jc;
    uint32_t jid = create_fixed_joint(jc, 0, 1);
    uint32_t i   = jc.index_of(jid);

    jc.r_1[i] = vec3{0,0,0};
    jc.r_2[i] = vec3{0,0,0};

    // Attach constraints
    for(int k=0;k<2;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_fixed_joint_errors(jc, i, bc, cc, 0.01);

    // Alignment error should be non-zero
    ASSERT_TRUE(
        cc.magnitude[cc.index_of(c_ids[0])] > 0.5
    );
}


TEST(compute_fixed_joint_errors_position_mismatch)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 2);

    // Offset second body
    bc.position[1] = vec3{1.0, 2.0, 3.0};

    JointCollection jc;
    uint32_t jid = create_fixed_joint(jc, 0, 1);
    uint32_t i   = jc.index_of(jid);

    jc.r_1[i] = vec3{0,0,0};
    jc.r_2[i] = vec3{0,0,0};

    // Attach constraints
    for(int k=0;k<2;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_fixed_joint_errors(jc, i, bc, cc, 0.01);

    // Position error should match the offset magnitude
    scalar expected_mag = std::sqrt(1.0*1.0 + 2.0*2.0 + 3.0*3.0);

    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[1])],
        expected_mag,
        0.001
    );
}

TEST(compute_fixed_joint_errors_with_attachment_points)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 2);
    
    JointCollection jc;
    uint32_t i = jc.index_of(jc.add());
    jc.type[i]   = FIXED;
    jc.body_1[i] = 0;
    jc.body_2[i] = 1;
    jc.r_1[i]    = vec3{1, 0, 0};
    jc.r_2[i]    = vec3{-1, 0, 0};
    for(int k=0; k<2; ++k) jc.constraints[i].ids[k] = c_ids[k];
    
    compute_fixed_joint_errors(jc, i, bc, cc, 0.01);
    
    // Body positions are (0,0,0). Attachment points are (1,0,0) and (-1,0,0).
    // The gap is 2.0.
    ASSERT_NEAR(cc.magnitude[cc.index_of(c_ids[1])], 2.0, 0.001);
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
    uint32_t jid = create_prismatic_joint(jc, 0, 1, vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.r_1[i] = vec3{0,0,0};
    jc.r_2[i] = vec3{0,0,0};
    jc.damping[i] = 0.5;

    bc.orientation[0] = quat(1,0,0,0);
    bc.orientation[1] = quat(1,0,0,0);

    vec3 initial_vel_0 = bc.linear_velocity[0];
    vec3 initial_vel_1 = bc.linear_velocity[1];

    apply_prismatic_joint_damping(jc, i, bc, 0.01);

    ASSERT_TRUE(bc.linear_velocity[0] == initial_vel_0);
    ASSERT_TRUE(bc.linear_velocity[1] == initial_vel_1);
}


TEST(apply_prismatic_joint_damping_with_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);

    bc.linear_velocity[0] = vec3{0,0,0};
    bc.linear_velocity[1] = vec3{1,0,0};

    bc.inverse_mass[0] = 1.0;
    bc.inverse_mass[1] = 1.0;

    bc.inverse_inertia_tensor_world[0] = smat3(1,1,1,0,0,0);
    bc.inverse_inertia_tensor_world[1] = smat3(1,1,1,0,0,0);

    JointCollection jc;
    uint32_t jid = create_prismatic_joint(jc,0,1,vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.r_1[i] = vec3{0,0,0};
    jc.r_2[i] = vec3{0,0,0};
    jc.damping[i] = 1.0;

    bc.orientation[0] = quat(1,0,0,0);
    bc.orientation[1] = quat(1,0,0,0);

    apply_prismatic_joint_damping(jc,i,bc,0.01);

    ASSERT_TRUE(bc.linear_velocity[0].x > 0.0);
    ASSERT_TRUE(bc.linear_velocity[1].x < 1.0);
}


TEST(apply_prismatic_joint_damping_high_damping)
{
    BodyCollection bc = create_test_bodies(2);

    bc.linear_velocity[0] = vec3{0,0,0};
    bc.linear_velocity[1] = vec3{10,0,0};

    bc.inverse_mass[0] = 1.0;
    bc.inverse_mass[1] = 1.0;

    bc.inverse_inertia_tensor_world[0] = smat3(1,1,1,0,0,0);
    bc.inverse_inertia_tensor_world[1] = smat3(1,1,1,0,0,0);

    JointCollection jc;
    uint32_t jid = create_prismatic_joint(jc,0,1,vec3{1,0,0});
    uint32_t i   = jc.index_of(jid);

    jc.r_1[i] = vec3{0,0,0};
    jc.r_2[i] = vec3{0,0,0};
    jc.damping[i] = 100.0;

    bc.orientation[0] = quat(1,0,0,0);
    bc.orientation[1] = quat(1,0,0,0);

    apply_prismatic_joint_damping(jc,i,bc,0.01);

    scalar relative_vel_after =
        m3d::magnitude(bc.linear_velocity[1] - bc.linear_velocity[0]);

    ASSERT_TRUE(relative_vel_after < 5.0);
}


TEST(apply_revolute_joint_damping_no_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i   = jc.index_of(jid);

    jc.damping[i] = 0.5;

    vec3 initial_omega_0 = bc.angular_velocity[0];
    vec3 initial_omega_1 = bc.angular_velocity[1];

    apply_revolute_joint_damping(jc,i,bc,0.01);

    ASSERT_TRUE(bc.angular_velocity[0] == initial_omega_0);
    ASSERT_TRUE(bc.angular_velocity[1] == initial_omega_1);
}


TEST(apply_revolute_joint_damping_with_relative_velocity)
{
    BodyCollection bc = create_test_bodies(2);

    bc.angular_velocity[0] = vec3{0,0,0};
    bc.angular_velocity[1] = vec3{0,0,2.0};

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i   = jc.index_of(jid);

    jc.damping[i] = 1.0;

    apply_revolute_joint_damping(jc,i,bc,0.01);

    ASSERT_TRUE(bc.angular_velocity[0].z > 0.0);
    ASSERT_TRUE(bc.angular_velocity[1].z < 2.0);
}


TEST(apply_revolute_joint_damping_high_damping)
{
    BodyCollection bc = create_test_bodies(2);

    bc.angular_velocity[0] = vec3{0,0,0};
    bc.angular_velocity[1] = vec3{0,0,10.0};

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i   = jc.index_of(jid);

    jc.damping[i] = 100.0;

    apply_revolute_joint_damping(jc,i,bc,0.01);

    ASSERT_NEAR(bc.angular_velocity[0].z,10.0,0.001);
    ASSERT_NEAR(bc.angular_velocity[1].z,0.0,0.001);

    scalar avg_omega =
        (bc.angular_velocity[0].z + bc.angular_velocity[1].z) / 2.0;

    ASSERT_NEAR(avg_omega,5.0,0.001);
}


TEST(apply_revolute_joint_damping_moderate_damping)
{
    BodyCollection bc = create_test_bodies(2);

    bc.angular_velocity[0] = vec3{0,0,0};
    bc.angular_velocity[1] = vec3{0,0,10.0};

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc,0,1,vec3{0,0,1});
    uint32_t i   = jc.index_of(jid);

    jc.damping[i] = 5.0;

    scalar initial_relative =
        m3d::magnitude(bc.angular_velocity[1] - bc.angular_velocity[0]);

    apply_revolute_joint_damping(jc,i,bc,0.01);

    scalar final_relative =
        m3d::magnitude(bc.angular_velocity[1] - bc.angular_velocity[0]);

    ASSERT_TRUE(final_relative < initial_relative);

    ASSERT_NEAR(bc.angular_velocity[0].z,0.5,0.001);
    ASSERT_NEAR(bc.angular_velocity[1].z,9.5,0.001);
    ASSERT_NEAR(final_relative,9.0,0.001);
}



// ============================================================================
// INTEGRATION TESTS
// ============================================================================

TEST(compute_joint_errors_multiple_joints)
{
    BodyCollection bc = create_test_bodies(4);
    ConstraintCollection cc;

    auto c_prismatic = add_test_constraints(cc,3);
    auto c_revolute  = add_test_constraints(cc,4);
    auto c_fixed     = add_test_constraints(cc,2);

    JointCollection jc;

    uint32_t j0 = create_prismatic_joint(jc,0,1,vec3{1,0,0});
    uint32_t j1 = create_revolute_joint(jc,1,2,vec3{0,0,1});
    uint32_t j2 = create_fixed_joint(jc,2,3);

    uint32_t i0 = jc.index_of(j0);
    uint32_t i1 = jc.index_of(j1);
    uint32_t i2 = jc.index_of(j2);

    for(int k=0;k<3;k++) jc.constraints[i0].ids[k] = c_prismatic[k];
    for(int k=0;k<4;k++) jc.constraints[i1].ids[k] = c_revolute[k];
    for(int k=0;k<2;k++) jc.constraints[i2].ids[k] = c_fixed[k];

    compute_joint_errors(jc, bc, cc, 0.01);

    for(size_t i=0;i<cc.count();i++)
        ASSERT_TRUE(cc.magnitude[i] >= 0.0);
}

TEST(apply_joint_damping_multiple_joints)
{
    BodyCollection bc = create_test_bodies(4);
    
    // Setup some velocities
    bc.linear_velocity[1] = vec3{1, 0, 0};
    bc.angular_velocity[2] = vec3{0, 0, 1};
    
    JointCollection jc;
    // Joint 0: Prismatic between 0 and 1
    uint32_t j0 = jc.index_of(jc.add());
    jc.type[j0] = PRISMATIC;
    jc.body_1[j0] = 0;
    jc.body_2[j0] = 1;
    jc.damping[j0] = 1.0;

    // Joint 1: Revolute between 1 and 2
    uint32_t j1 = jc.index_of(jc.add());
    jc.type[j1] = REVOLUTE;
    jc.body_1[j1] = 1;
    jc.body_2[j1] = 2;
    jc.damping[j1] = 1.0;
    
    apply_joint_damping(jc, bc, 0.01);
    
    // Check that damping affected velocities
    ASSERT_TRUE(m3d::magnitude(bc.linear_velocity[1]) < 1.0);
    ASSERT_TRUE(m3d::magnitude(bc.angular_velocity[2]) < 1.0);
}

TEST(joint_errors_with_attachment_points)
{
    BodyCollection bc = create_test_bodies(2);
    ConstraintCollection cc;
    auto c_ids = add_test_constraints(cc, 4);

    JointCollection jc;
    uint32_t jid = create_revolute_joint(jc, 0, 1, vec3{0,0,1});
    uint32_t i   = jc.index_of(jid);

    jc.actuation_type[i] = FREE;

    jc.r_1[i] = vec3{1,0,0};   // Attachment point offset
    jc.r_2[i] = vec3{-1,0,0};  // Attachment point offset

    jc.main_axis[i]  = vec3{0,0,1};
    jc.limit_axis[i] = vec3{1,0,0};

    jc.limited[i] = false;

    jc.lower_limit[i] = -M_PI;
    jc.upper_limit[i] =  M_PI;

    jc.target_position[i] = 0.0;
    jc.target_speed[i]    = 0.0;
    jc.current_position[i]= 0.0;

    jc.damping[i] = 0.1;

    // Attach constraints
    for(int k=0;k<4;k++)
        jc.constraints[i].ids[k] = c_ids[k];

    compute_revolute_joint_errors(jc, i, bc, cc, 0.01);

    // Attachment error should be 2.0 (distance between offset points)
    ASSERT_NEAR(
        cc.magnitude[cc.index_of(c_ids[1])],
        2.0,
        1e-4
    );
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