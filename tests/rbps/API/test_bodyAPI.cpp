#include "rbps/API/BodyAPI.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static BodyCollection make_two_body_collection()
{
    BodyCollection bc;
    create_body(bc, {.type = DYNAMIC, .mass = 2.0, .position = {1, 2, 3}});
    create_body(bc, {.type = STATIC,  .mass = 0.0, .position = {4, 5, 6}});
    return bc;
}

// ─── Counter / size tests ─────────────────────────────────────────────────────

TEST(body_count_starts_at_zero)
{
    BodyCollection bc;
    ASSERT_EQ(bc.count(), 0u);
    ASSERT_EQ(bc.position.size(), 0u);
}

TEST(body_count_increments_on_create)
{
    BodyCollection bc;
    create_body(bc);
    ASSERT_EQ(bc.count(), 1u);
    create_body(bc);
    ASSERT_EQ(bc.count(), 2u);
}

TEST(all_arrays_same_size_after_creates)
{
    BodyCollection bc;
    create_body(bc);
    create_body(bc);
    create_body(bc);
    ASSERT_EQ(bc.position.size(),    3u);
    ASSERT_EQ(bc.mass.size(),        3u);
    ASSERT_EQ(bc.orientation.size(), 3u);
    ASSERT_EQ(bc.type.size(),        3u);
    ASSERT_EQ(bc.force.size(),       3u);
}

// ─── Field initialisation ─────────────────────────────────────────────────────

TEST(dynamic_body_stores_position)
{
    BodyCollection bc;
    uint32_t id = create_body(bc, {.position = {1, 2, 3}});
    uint32_t i  = bc.index_of(id);
    ASSERT_NEAR(bc.position[i].x, 1.0);
    ASSERT_NEAR(bc.position[i].y, 2.0);
    ASSERT_NEAR(bc.position[i].z, 3.0);
}

TEST(dynamic_body_stores_mass)
{
    BodyCollection bc;
    uint32_t id = create_body(bc, {.mass = 5.0});
    uint32_t i  = bc.index_of(id);
    ASSERT_NEAR(bc.mass[i], 5.0);
    ASSERT_NEAR(bc.inverse_mass[i], 0.2);
}

TEST(static_body_has_zero_inverse_mass)
{
    BodyCollection bc;
    uint32_t id = create_body(bc, {.type = STATIC, .mass = 10.0});
    uint32_t i  = bc.index_of(id);
    ASSERT_EQ(bc.type[i], STATIC);
    ASSERT_NEAR(bc.inverse_mass[i], 0.0);
}

TEST(dynamic_body_gravity_force)
{
    BodyCollection bc;
    uint32_t id = create_body(bc, {.mass = 2.0});
    uint32_t i  = bc.index_of(id);
    ASSERT_NEAR(bc.force[i].x, 0.0);
    ASSERT_NEAR(bc.force[i].y, -9.8 * 2.0, 1e-4);
    ASSERT_NEAR(bc.force[i].z, 0.0);
}

TEST(prev_position_matches_initial_position)
{
    BodyCollection bc;
    uint32_t id = create_body(bc, {.position = {7, 8, 9}});
    uint32_t i  = bc.index_of(id);
    ASSERT_NEAR(bc.prev_position[i].x, 7.0);
    ASSERT_NEAR(bc.prev_position[i].y, 8.0);
    ASSERT_NEAR(bc.prev_position[i].z, 9.0);
}

TEST(linear_velocity_initialised_correctly)
{
    BodyCollection bc;
    uint32_t id = create_body(bc, {.linear_velocity = {1, 0, -1}});
    uint32_t i  = bc.index_of(id);
    ASSERT_NEAR(bc.linear_velocity[i].x,  1.0);
    ASSERT_NEAR(bc.linear_velocity[i].y,  0.0);
    ASSERT_NEAR(bc.linear_velocity[i].z, -1.0);
}

// ─── Stable-ID tests ──────────────────────────────────────────────────────────

TEST(returned_id_is_stable)
{
    BodyCollection bc;
    uint32_t id_a = create_body(bc, {.position = {1, 0, 0}});
    uint32_t id_b = create_body(bc, {.position = {2, 0, 0}});
    uint32_t id_c = create_body(bc, {.position = {3, 0, 0}});

    remove_body(bc, id_b);

    ASSERT_NEAR(bc.position[bc.index_of(id_a)].x, 1.0);
    ASSERT_NEAR(bc.position[bc.index_of(id_c)].x, 3.0);
}


TEST(handle_valid_after_create)
{
    BodyCollection bc;
    uint32_t id = create_body(bc);
    ASSERT_TRUE(bc.contains(id));
}

TEST(handle_invalid_after_remove)
{
    BodyCollection bc;
    uint32_t id = create_body(bc);
    remove_body(bc, id);
    ASSERT_FALSE(bc.contains(id));
}

// ─── remove_body ─────────────────────────────────────────────────────────────

TEST(remove_decrements_count)
{
    BodyCollection bc;
    uint32_t id = create_body(bc);
    create_body(bc);
    remove_body(bc, id);
    ASSERT_EQ(bc.count(), 1u);
}

TEST(remove_shrinks_all_arrays)
{
    BodyCollection bc;
    uint32_t id = create_body(bc);
    create_body(bc);
    remove_body(bc, id);
    ASSERT_EQ(bc.position.size(),    1u);
    ASSERT_EQ(bc.mass.size(),        1u);
    ASSERT_EQ(bc.orientation.size(), 1u);
}

TEST(remove_first_body_data_intact)
{
    BodyCollection bc;
    uint32_t id_a = create_body(bc, {.mass = 1.0, .position = {10, 0, 0}});
    uint32_t id_b = create_body(bc, {.mass = 9.0, .position = {99, 0, 0}});
    remove_body(bc, id_a);

    uint32_t i = bc.index_of(id_b);
    ASSERT_NEAR(bc.mass[i],       9.0);
    ASSERT_NEAR(bc.position[i].x, 99.0);
}

TEST(remove_all_bodies)
{
    BodyCollection bc;
    uint32_t a = create_body(bc);
    uint32_t b = create_body(bc);
    uint32_t c = create_body(bc);
    remove_body(bc, b);
    remove_body(bc, a);
    remove_body(bc, c);
    ASSERT_EQ(bc.count(), 0u);
    ASSERT_EQ(bc.position.size(), 0u);
}

// ─── Suite ───────────────────────────────────────────────────────────────────

TEST_SUITE(
    RUN_TEST(body_count_starts_at_zero),
    RUN_TEST(body_count_increments_on_create),
    RUN_TEST(all_arrays_same_size_after_creates),
    RUN_TEST(dynamic_body_stores_position),
    RUN_TEST(dynamic_body_stores_mass),
    RUN_TEST(static_body_has_zero_inverse_mass),
    RUN_TEST(dynamic_body_gravity_force),
    RUN_TEST(prev_position_matches_initial_position),
    RUN_TEST(linear_velocity_initialised_correctly),
    RUN_TEST(returned_id_is_stable),
    RUN_TEST(handle_valid_after_create),
    RUN_TEST(handle_invalid_after_remove),
    RUN_TEST(remove_decrements_count),
    RUN_TEST(remove_shrinks_all_arrays),
    RUN_TEST(remove_first_body_data_intact),
    RUN_TEST(remove_all_bodies)
)