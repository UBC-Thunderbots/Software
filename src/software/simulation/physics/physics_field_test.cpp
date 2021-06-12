#include "software/simulation/physics/physics_field.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "software/simulation/physics/physics_ball.h"
#include "software/time/duration.h"
#include "software/world/field.h"

class PhysicsFieldTest : public testing::Test
{
   public:
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    static constexpr int BOX2D_STEP_VELOCITY_ITERATIONS = 5;
    static constexpr int BOX2D_STEP_POSITION_ITERATIONS = 8;

   protected:
    virtual void SetUp()
    {
        b2Vec2 gravity(0, 0);
        world          = std::make_shared<b2World>(gravity);
        physics_config = std::make_shared<const PhysicsConfig>();
    }

    void simulateForDuration(const Duration& duration)
    {
        double step_size_seconds = 1.0 / 60.0;
        unsigned int num_steps =
            static_cast<unsigned int>(duration.toSeconds() / step_size_seconds);

        // We have to take lots of small steps because a significant amount of accuracy
        // is lost if we take a single step of 1 second
        for (unsigned int i = 0; i < num_steps; i++)
        {
            world->Step(static_cast<float>(step_size_seconds),
                        BOX2D_STEP_VELOCITY_ITERATIONS, BOX2D_STEP_POSITION_ITERATIONS);
        }
    }

    std::shared_ptr<b2World> world;
    std::shared_ptr<const PhysicsConfig> physics_config;
};

TEST_F(PhysicsFieldTest, test_get_field)
{
    Field field_parameter(Field::createSSLDivisionBField());
    auto physics_field = PhysicsField(world, field_parameter);
    auto field         = physics_field.getField();

    EXPECT_EQ(field_parameter, field);
}

TEST_F(PhysicsFieldTest, test_field_added_to_physics_world_on_creation)
{
    Field field_parameter(Field::createSSLDivisionBField());

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_field = PhysicsField(world, field_parameter);

    // There is only 1 body that represents the entire field
    EXPECT_EQ(1, world->GetBodyCount());
}

TEST_F(PhysicsFieldTest, test_physics_field_is_removed_from_world_when_destroyed)
{
    {
        Field field_parameter(Field::createSSLDivisionBField());

        EXPECT_EQ(0, world->GetBodyCount());

        auto physics_field = PhysicsField(world, field_parameter);

        EXPECT_EQ(1, world->GetBodyCount());
    }

    // Once we leave the above scope the field is destroyed, so it should have been
    // removed from the world
    EXPECT_EQ(0, world->GetBodyCount());
}

TEST_F(PhysicsFieldTest, test_field_dimensions_do_not_change_during_simulation_step)
{
    Field field_parameter(Field::createSSLDivisionBField());
    auto physics_field = PhysicsField(world, field_parameter);

    simulateForDuration(Duration::fromSeconds(1));

    auto field = physics_field.getField();
    EXPECT_EQ(field, field_parameter);
}

TEST_F(PhysicsFieldTest, test_ball_bounces_off_field_boundary)
{
    Field field_parameter(Field::createSSLDivisionBField());
    auto physics_field = PhysicsField(world, field_parameter);

    BallState initial_ball_state(field_parameter.friendlyHalf().posXPosYCorner(),
                                 Vector(0, 2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 0.1, physics_config);

    simulateForDuration(Duration::fromSeconds(1));

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(0, -2) - ball.velocity()).length(), 1e-5);
}

TEST_F(PhysicsFieldTest, test_ball_bounces_off_enemy_goal)
{
    Field field_parameter(Field::createSSLDivisionBField());
    auto physics_field = PhysicsField(world, field_parameter);

    BallState initial_ball_state(field_parameter.enemyGoalCenter() + Vector(-1, 0),
                                 Vector(3.0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 0.1, physics_config);

    simulateForDuration(Duration::fromSeconds(1));

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(-3.0, 0.0) - ball.velocity()).length(), 1e-3);
}

TEST_F(PhysicsFieldTest, test_ball_bounces_off_friendly_goal)
{
    Field field_parameter(Field::createSSLDivisionBField());
    auto physics_field = PhysicsField(world, field_parameter);

    BallState initial_ball_state(field_parameter.friendlyGoalCenter() + Vector(1, 0),
                                 Vector(-3.0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 0.1, physics_config);

    simulateForDuration(Duration::fromSeconds(1));

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(3.0, 0.0) - ball.velocity()).length(), 1e-3);
}
