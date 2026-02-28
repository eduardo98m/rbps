#include "rbps/API/JointAPI.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;

struct Fixture {
    JointCollection      jc;
    ConstraintCollection cc;
};

// ═════════════════════════════════════════════════════════════════════════════
// PRISMATIC — create
// ═════════════════════════════════════════════════════════════════════════════

TEST(prismatic_joint_count_increments)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.jc.n_joints, 1u);
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=2});
    ASSERT_EQ(f.jc.n_joints, 2u);
}

TEST(prismatic_joint_creates_3_constraints)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.cc.n_constraints, 3u);
    ASSERT_EQ(f.jc.constraint_count[0], 3);
}

TEST(prismatic_joint_constraint_start_is_zero_for_first)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.jc.constraint_start[0], 0u);
}

TEST(prismatic_joint_second_start_is_3)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    create_prismatic_joint(f.jc, f.cc, {.body_1=1, .body_2=2});
    ASSERT_EQ(f.jc.constraint_start[1], 3u);
    ASSERT_EQ(f.cc.n_constraints, 6u);
}

TEST(prismatic_joint_stores_bodies)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=4, .body_2=7});
    ASSERT_EQ(f.jc.body_1[0], 4u);
    ASSERT_EQ(f.jc.body_2[0], 7u);
}

TEST(prismatic_joint_stores_type)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.jc.type[0], JointType::PRISMATIC);
}

TEST(prismatic_joint_stores_axis)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1, .moving_axis={0,1,0}});
    ASSERT_NEAR(f.jc.main_axis[0].y, 1.0);
}

TEST(prismatic_joint_stores_limits)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc,
        {.body_1=0, .body_2=1, .limited=true, .lower_limit=-1.0, .upper_limit=2.0});
    ASSERT_TRUE(f.jc.limited[0]);
    ASSERT_NEAR(f.jc.lower_limit[0], -1.0);
    ASSERT_NEAR(f.jc.upper_limit[0],  2.0);
}

TEST(prismatic_joint_constraint_types)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.cc.type[0], ConstraintType::ROTATIONAL);
    ASSERT_EQ(f.cc.type[1], ConstraintType::POSITIONAL);
    ASSERT_EQ(f.cc.type[2], ConstraintType::POSITIONAL);
}

TEST(prismatic_drive_compliance_on_correct_constraint)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1, .compliance=0.05});
    ASSERT_NEAR(f.cc.compliance[2], 0.05);
    ASSERT_NEAR(f.cc.compliance[0], 0.0);
    ASSERT_NEAR(f.cc.compliance[1], 0.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// REVOLUTE — create
// ═════════════════════════════════════════════════════════════════════════════

TEST(revolute_joint_creates_4_constraints)
{
    Fixture f;
    create_revolute_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.cc.n_constraints, 4u);
    ASSERT_EQ(f.jc.constraint_count[0], 4);
}

TEST(revolute_joint_stores_type)
{
    Fixture f;
    create_revolute_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.jc.type[0], JointType::REVOLUTE);
}

TEST(revolute_joint_stores_limit_axis)
{
    Fixture f;
    create_revolute_joint(f.jc, f.cc, {.body_1=0, .body_2=1, .limit_axis={1,0,0}});
    ASSERT_NEAR(f.jc.limit_axis[0].x, 1.0);
}

TEST(revolute_joint_constraint_types)
{
    Fixture f;
    create_revolute_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.cc.type[0], ConstraintType::ROTATIONAL);
    ASSERT_EQ(f.cc.type[1], ConstraintType::POSITIONAL);
    ASSERT_EQ(f.cc.type[2], ConstraintType::ROTATIONAL);
    ASSERT_EQ(f.cc.type[3], ConstraintType::ROTATIONAL);
}

TEST(revolute_drive_compliance_on_correct_constraint)
{
    Fixture f;
    create_revolute_joint(f.jc, f.cc, {.body_1=0, .body_2=1, .compliance=0.1});
    ASSERT_NEAR(f.cc.compliance[3], 0.1);
    ASSERT_NEAR(f.cc.compliance[0], 0.0);
    ASSERT_NEAR(f.cc.compliance[1], 0.0);
    ASSERT_NEAR(f.cc.compliance[2], 0.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// FIXED — create
// ═════════════════════════════════════════════════════════════════════════════

TEST(fixed_joint_creates_2_constraints)
{
    Fixture f;
    create_fixed_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.cc.n_constraints, 2u);
    ASSERT_EQ(f.jc.constraint_count[0], 2);
}

TEST(fixed_joint_stores_type)
{
    Fixture f;
    create_fixed_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.jc.type[0], JointType::FIXED);
}

TEST(fixed_joint_is_not_limited_and_free)
{
    Fixture f;
    create_fixed_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_FALSE(f.jc.limited[0]);
    ASSERT_EQ(f.jc.actuation_type[0], JointActuationType::FREE);
}

TEST(fixed_joint_constraint_types)
{
    Fixture f;
    create_fixed_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ASSERT_EQ(f.cc.type[0], ConstraintType::ROTATIONAL);
    ASSERT_EQ(f.cc.type[1], ConstraintType::POSITIONAL);
}

// ═════════════════════════════════════════════════════════════════════════════
// remove_joint — same pattern as remove_body / remove_constraint
// ═════════════════════════════════════════════════════════════════════════════

TEST(remove_joint_decrements_count)
{
    Fixture f;
    ivc::ID id = create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    create_revolute_joint(f.jc, f.cc, {.body_1=1, .body_2=2});
    remove_joint(f.jc, f.cc, id);
    ASSERT_EQ(f.jc.n_joints, 1u);
}

TEST(remove_joint_shrinks_all_arrays)
{
    Fixture f;
    ivc::ID id = create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    create_revolute_joint(f.jc, f.cc, {.body_1=1, .body_2=2});
    remove_joint(f.jc, f.cc, id);
    ASSERT_EQ(f.jc.type.size(),              1u);
    ASSERT_EQ(f.jc.body_1.size(),            1u);
    ASSERT_EQ(f.jc.constraint_start.size(),  1u);
    ASSERT_EQ(f.jc.constraint_count.size(),  1u);
}

TEST(remove_joint_also_removes_child_constraints)
{
    Fixture f;
    // prismatic → 3 constraints, revolute → 4 constraints = 7 total
    ivc::ID id = create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    create_revolute_joint(f.jc, f.cc, {.body_1=1, .body_2=2});
    ASSERT_EQ(f.cc.n_constraints, 7u);

    remove_joint(f.jc, f.cc, id);   // removes prismatic + its 3 constraints

    ASSERT_EQ(f.cc.n_constraints, 4u);   // only the revolute's 4 remain
    ASSERT_EQ(f.cc.body_1.size(), 4u);
}

TEST(remove_joint_handle_becomes_invalid)
{
    Fixture f;
    ivc::ID id = create_fixed_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ivc::Handle h = ivc::make_handle(f.jc._ivc, id);
    remove_joint(f.jc, f.cc, id);
    ASSERT_FALSE(h.isValid());
}

TEST(remove_joint_surviving_id_stable)
{
    Fixture f;
    ivc::ID id_a = create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ivc::ID id_b = create_revolute_joint (f.jc, f.cc, {.body_1=1, .body_2=2});
    ivc::ID id_c = create_fixed_joint   (f.jc, f.cc, {.body_1=2, .body_2=3});

    remove_joint(f.jc, f.cc, id_b);

    ASSERT_EQ(f.jc.type[ivc::index(f.jc._ivc, id_a)], JointType::PRISMATIC);
    ASSERT_EQ(f.jc.type[ivc::index(f.jc._ivc, id_c)], JointType::FIXED);
}

TEST(remove_joint_data_intact_after_swap)
{
    Fixture f;
    ivc::ID id_a = create_revolute_joint(f.jc, f.cc,
        {.body_1=10, .body_2=11, .aligned_axis={0,0,1}});
    ivc::ID id_b = create_fixed_joint(f.jc, f.cc,
        {.body_1=20, .body_2=21});

    remove_joint(f.jc, f.cc, id_a);

    size_t i = ivc::index(f.jc._ivc, id_b);
    ASSERT_EQ(f.jc.body_1[i], 20u);
    ASSERT_EQ(f.jc.body_2[i], 21u);
    ASSERT_EQ(f.jc.type[i], JointType::FIXED);
}

TEST(remove_all_joints)
{
    Fixture f;
    ivc::ID a = create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    ivc::ID b = create_revolute_joint (f.jc, f.cc, {.body_1=1, .body_2=2});
    ivc::ID c = create_fixed_joint   (f.jc, f.cc, {.body_1=2, .body_2=3});
    remove_joint(f.jc, f.cc, b);
    remove_joint(f.jc, f.cc, a);
    remove_joint(f.jc, f.cc, c);
    ASSERT_EQ(f.jc.n_joints,      0u);
    ASSERT_EQ(f.cc.n_constraints, 0u);
    ASSERT_EQ(f.jc.type.size(),   0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// MIXED — constraint_start bookkeeping
// ═════════════════════════════════════════════════════════════════════════════

TEST(mixed_joints_constraint_starts_contiguous)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1}); // 3 constraints → start=0
    create_revolute_joint (f.jc, f.cc, {.body_1=1, .body_2=2}); // 4 constraints → start=3
    create_fixed_joint    (f.jc, f.cc, {.body_1=2, .body_2=3}); // 2 constraints → start=7

    ASSERT_EQ(f.jc.n_joints,      3u);
    ASSERT_EQ(f.cc.n_constraints,  9u);
    ASSERT_EQ(f.jc.constraint_start[0], 0u);
    ASSERT_EQ(f.jc.constraint_start[1], 3u);
    ASSERT_EQ(f.jc.constraint_start[2], 7u);
}

TEST(mixed_joints_all_arrays_consistent)
{
    Fixture f;
    create_prismatic_joint(f.jc, f.cc, {.body_1=0, .body_2=1});
    create_revolute_joint (f.jc, f.cc, {.body_1=1, .body_2=2});
    create_fixed_joint    (f.jc, f.cc, {.body_1=2, .body_2=3});

    ASSERT_EQ(f.jc.type.size(),             3u);
    ASSERT_EQ(f.jc.body_1.size(),           3u);
    ASSERT_EQ(f.jc.constraint_start.size(), 3u);
    ASSERT_EQ(f.jc.constraint_count.size(), 3u);
    ASSERT_EQ(f.jc.limit_axis.size(),       3u);
    ASSERT_EQ(f.jc.damping.size(),          3u);
}

// ─── Suite ───────────────────────────────────────────────────────────────────

TEST_SUITE(
    RUN_TEST(prismatic_joint_count_increments),
    RUN_TEST(prismatic_joint_creates_3_constraints),
    RUN_TEST(prismatic_joint_constraint_start_is_zero_for_first),
    RUN_TEST(prismatic_joint_second_start_is_3),
    RUN_TEST(prismatic_joint_stores_bodies),
    RUN_TEST(prismatic_joint_stores_type),
    RUN_TEST(prismatic_joint_stores_axis),
    RUN_TEST(prismatic_joint_stores_limits),
    RUN_TEST(prismatic_joint_constraint_types),
    RUN_TEST(prismatic_drive_compliance_on_correct_constraint),
    RUN_TEST(revolute_joint_creates_4_constraints),
    RUN_TEST(revolute_joint_stores_type),
    RUN_TEST(revolute_joint_stores_limit_axis),
    RUN_TEST(revolute_joint_constraint_types),
    RUN_TEST(revolute_drive_compliance_on_correct_constraint),
    RUN_TEST(fixed_joint_creates_2_constraints),
    RUN_TEST(fixed_joint_stores_type),
    RUN_TEST(fixed_joint_is_not_limited_and_free),
    RUN_TEST(fixed_joint_constraint_types),
    RUN_TEST(remove_joint_decrements_count),
    RUN_TEST(remove_joint_shrinks_all_arrays),
    RUN_TEST(remove_joint_handle_becomes_invalid),
    RUN_TEST(remove_joint_surviving_id_stable),
    RUN_TEST(remove_joint_data_intact_after_swap),
    RUN_TEST(remove_all_joints),
    RUN_TEST(mixed_joints_constraint_starts_contiguous),
    RUN_TEST(mixed_joints_all_arrays_consistent)
)