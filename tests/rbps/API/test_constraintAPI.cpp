#include "rbps/API/ConstraintAPI.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;

// ─── Counter / size tests ─────────────────────────────────────────────────────

TEST(constraint_count_starts_at_zero)
{
    ConstraintCollection cc;
    ASSERT_EQ(cc.count(), 0u);
}

TEST(constraint_count_increments)
{
    ConstraintCollection cc;
    create_constraint(cc, {.body_1 = 0, .body_2 = 1, .type = ConstraintType::POSITIONAL});
    ASSERT_EQ(cc.count(), 1u);
    create_constraint(cc, {.body_1 = 1, .body_2 = 2, .type = ConstraintType::ROTATIONAL});
    ASSERT_EQ(cc.count(), 2u);
}

TEST(all_arrays_same_size_after_creates)
{
    ConstraintCollection cc;
    create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    create_constraint(cc, {.body_1=1, .body_2=2, .type=ConstraintType::ROTATIONAL});

    ASSERT_EQ(cc.body_1.size(),     2u);
    ASSERT_EQ(cc.body_2.size(),     2u);
    ASSERT_EQ(cc.r_1.size(),        2u);
    ASSERT_EQ(cc.r_2.size(),        2u);
    ASSERT_EQ(cc.direction.size(),  2u);
    ASSERT_EQ(cc.magnitude.size(),  2u);
    ASSERT_EQ(cc.lambda.size(),     2u);
    ASSERT_EQ(cc.compliance.size(), 2u);
    ASSERT_EQ(cc.type.size(),       2u);
    ASSERT_EQ(cc.impulse.size(),    2u);
}

// ─── Return value ─────────────────────────────────────────────────────────────

TEST(create_returns_sequential_ids)
{
    ConstraintCollection cc;
    size_t id0 = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    size_t id1 = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    size_t id2 = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    ASSERT_EQ(id0, 0u);
    ASSERT_EQ(id1, 1u);
    ASSERT_EQ(id2, 2u);
}

// ─── Field initialisation ─────────────────────────────────────────────────────

TEST(positional_constraint_stores_bodies)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=3, .body_2=7, .type=ConstraintType::POSITIONAL});
    ASSERT_EQ(cc.body_1[id], 3u);
    ASSERT_EQ(cc.body_2[id], 7u);
}

TEST(constraint_stores_type_positional)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    ASSERT_EQ(cc.type[id], ConstraintType::POSITIONAL);
}

TEST(constraint_stores_type_rotational)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::ROTATIONAL});
    ASSERT_EQ(cc.type[id], ConstraintType::ROTATIONAL);
}

TEST(constraint_stores_compliance)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1,
                                       .type=ConstraintType::POSITIONAL, .compliance=0.01});
    ASSERT_NEAR(cc.compliance[id], 0.01);
}

TEST(constraint_stores_attachment_points)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1,
                                       .type=ConstraintType::POSITIONAL,
                                       .r_1={1,2,3}, .r_2={4,5,6}});
    ASSERT_NEAR(cc.r_1[id].x, 1.0);
    ASSERT_NEAR(cc.r_1[id].y, 2.0);
    ASSERT_NEAR(cc.r_1[id].z, 3.0);
    ASSERT_NEAR(cc.r_2[id].x, 4.0);
    ASSERT_NEAR(cc.r_2[id].y, 5.0);
    ASSERT_NEAR(cc.r_2[id].z, 6.0);
}

TEST(constraint_lambda_initialised_to_zero)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    ASSERT_NEAR(cc.lambda[id], 0.0);
}

TEST(constraint_magnitude_initialised_to_zero)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    ASSERT_NEAR(cc.magnitude[id], 0.0);
}

TEST(zero_compliance_by_default)
{
    ConstraintCollection cc;
    size_t id = create_constraint(cc, {.body_1=0, .body_2=1, .type=ConstraintType::POSITIONAL});
    ASSERT_NEAR(cc.compliance[id], 0.0);
}


// ─── remove_constraint ─────────────────────────────────────────────────────────────

TEST(remove_decrements_count)
{
    ConstraintCollection cc;
    u_int32_t id = create_constraint(cc);
    create_constraint(cc);
    remove_constraint(cc, id);
    ASSERT_EQ(cc.count(), 1u);
}

TEST(remove_shrinks_all_arrays)
{
    ConstraintCollection cc;
    u_int32_t id = create_constraint(cc);
    create_constraint(cc);
    remove_constraint(cc, id);
    ASSERT_EQ(cc.body_1.size(),        1u);
    ASSERT_EQ(cc.body_2.size(),        1u);
    ASSERT_EQ(cc.r_1.size(),        1u);
    ASSERT_EQ(cc.r_2.size(),        1u);
    ASSERT_EQ(cc.direction.size(),  1u);
    ASSERT_EQ(cc.magnitude.size(),  1u);
    ASSERT_EQ(cc.lambda.size(),     1u);
    ASSERT_EQ(cc.compliance.size(), 1u);
    ASSERT_EQ(cc.type.size(),       1u);
    ASSERT_EQ(cc.impulse.size(),    1u);
}

TEST(remove_first_constraint_data_intact)
{
    // After removing the first constraint the second must slide to index 0
    // and still have its original values.
    ConstraintCollection cc;
    u_int32_t id_a = create_constraint(cc, {.body_1=1, .body_2=2});
    u_int32_t id_b = create_constraint(cc, {.body_1=3, .body_2=4});
    remove_constraint(cc, id_a);

    size_t i = cc.index_of(id_b); //ivc::index(cc._ivc, id_b);
    ASSERT_EQ(cc.body_1[i], 3u);
    ASSERT_EQ(cc.body_2[i], 4u);
}

TEST(remove_all_constraints)
{
    ConstraintCollection cc;
    u_int32_t a = create_constraint(cc);
    u_int32_t b = create_constraint(cc);
    u_int32_t c = create_constraint(cc);
    remove_constraint(cc, b);
    remove_constraint(cc, a);
    remove_constraint(cc, c);
    ASSERT_EQ(cc.count(), 0u);
    ASSERT_EQ(cc.body_1.size(), 0u);
}


// ─── Suite ───────────────────────────────────────────────────────────────────

TEST_SUITE(
    RUN_TEST(constraint_count_starts_at_zero),
    RUN_TEST(constraint_count_increments),
    RUN_TEST(all_arrays_same_size_after_creates),
    RUN_TEST(create_returns_sequential_ids),
    RUN_TEST(positional_constraint_stores_bodies),
    RUN_TEST(constraint_stores_type_positional),
    RUN_TEST(constraint_stores_type_rotational),
    RUN_TEST(constraint_stores_compliance),
    RUN_TEST(constraint_stores_attachment_points),
    RUN_TEST(constraint_lambda_initialised_to_zero),
    RUN_TEST(constraint_magnitude_initialised_to_zero),
    RUN_TEST(zero_compliance_by_default),
    RUN_TEST(remove_decrements_count),
    RUN_TEST(remove_shrinks_all_arrays),
    RUN_TEST(remove_first_constraint_data_intact),
    RUN_TEST(remove_all_constraints)
)