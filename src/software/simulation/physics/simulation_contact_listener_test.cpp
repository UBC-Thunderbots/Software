#include "software/simulation/physics/simulation_contact_listener.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "shared/2015_robot_constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_object_user_data.h"
#include "software/simulation/physics/physics_robot.h"
#include "software/test_util/test_util.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

class SimulationContactListenerTest : public testing::Test
{
   protected:
    std::tuple<std::shared_ptr<b2World>, std::shared_ptr<PhysicsRobot>,
               std::shared_ptr<PhysicsBall>>
    createWorld(const Robot& robot, const Ball& ball)
    {
        b2Vec2 gravity(0, 0);
        simulator_config = std::make_shared<const SimulatorConfig>();
        physics_world    = std::make_shared<b2World>(gravity);
        physics_ball  = std::make_shared<PhysicsBall>(physics_world, ball.currentState(),
                                                     1.0, simulator_config);
        physics_robot = std::make_shared<PhysicsRobot>(
            robot.id(), physics_world, robot.currentState(), create2015RobotConstants(),
            create2015WheelConstants());

        return std::make_tuple(physics_world, physics_robot, physics_ball);
    }

    void TearDown() override
    {
        // All PhysicsObject that were added to the physics_world must be destroyed first,
        // otherwise the PhysicsObjects will attempt to access the world when they are
        // destroyed and cause a segfault
        physics_ball.reset();
        physics_robot.reset();
        physics_world.reset();
    }

   private:
    std::shared_ptr<b2World> physics_world;
    std::shared_ptr<PhysicsBall> physics_ball;
    std::shared_ptr<const SimulatorConfig> simulator_config;
    std::shared_ptr<PhysicsRobot> physics_robot;
};

TEST_F(SimulationContactListenerTest, test_is_ball_contact)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_robot{PhysicsObjectType ::ROBOT_BODY,
                                          physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallContact(&user_data_ball, &user_data_robot);

    ASSERT_TRUE(result);
    EXPECT_EQ(result, physics_ball.get());
    UNUSED(physics_world);
}

TEST_F(SimulationContactListenerTest, test_is_ball_contact_with_reversed_args)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_chicker{PhysicsObjectType ::ROBOT_CHICKER,
                                            physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallContact(&user_data_chicker, &user_data_ball);

    ASSERT_TRUE(result);
    EXPECT_EQ(result, physics_ball.get());
    UNUSED(physics_world);
}

TEST_F(SimulationContactListenerTest, test_is_ball_chicker_contact)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_chicker{PhysicsObjectType ::ROBOT_CHICKER,
                                            physics_robot.get()};

    SimulationContactListener listener;
    auto result =
        listener.isDribblerDamperBallContact(&user_data_chicker, &user_data_ball);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
    UNUSED(physics_world);
}

TEST_F(SimulationContactListenerTest, test_is_ball_chicker_contact_with_reversed_args)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_chicker{PhysicsObjectType ::ROBOT_CHICKER,
                                            physics_robot.get()};

    SimulationContactListener listener;
    auto result =
        listener.isDribblerDamperBallContact(&user_data_ball, &user_data_chicker);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
    UNUSED(physics_world);
}

TEST_F(SimulationContactListenerTest, test_is_ball_dribbler_contact)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_dribbler{PhysicsObjectType ::ROBOT_DRIBBLER,
                                             physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isDribblerBallContact(&user_data_ball, &user_data_dribbler);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
    UNUSED(physics_world);
}

TEST_F(SimulationContactListenerTest, test_is_ball_dribbler_contact_with_reversed_args)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_dribbler{PhysicsObjectType ::ROBOT_DRIBBLER,
                                             physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isDribblerBallContact(&user_data_dribbler, &user_data_ball);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
    UNUSED(physics_world);
}
