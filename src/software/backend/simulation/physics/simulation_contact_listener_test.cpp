#include "software/backend/simulation/physics/simulation_contact_listener.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/world/robot.h"
#include "software/world/ball.h"

class SimulationContactListenerTest : public testing::Test
{
protected:
    std::tuple<std::shared_ptr<b2World>, std::shared_ptr<PhysicsRobot>,
            std::shared_ptr<PhysicsBall>>
    createWorld(const Robot& robot, const Ball& ball)
    {
        b2Vec2 gravity(0, 0);
        auto physics_world = std::make_shared<b2World>(gravity);

        auto physics_ball = std::make_shared<PhysicsBall>(physics_world, ball, 1.0, 9.8);
        auto physics_robot = std::make_shared<PhysicsRobot>(physics_world, robot, 1.0);

        return std::make_tuple(physics_world, physics_robot, physics_ball);
    }

    Point getDribblingPoint(const Point& robot_position, const Angle& robot_orientation)
    {
        double dribbler_depth = PhysicsRobot::dribbler_depth;
        Point dribbling_point =
                robot_position + Vector::createFromAngle(robot_orientation)
                        .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                   BALL_MAX_RADIUS_METERS - dribbler_depth);
        return dribbling_point;
    }
};

TEST_F(SimulationContactListenerTest, test_is_ball_contact)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_robot{PhysicsObjectType ::ROBOT_BODY, physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallContact(&user_data_ball, &user_data_robot);

    ASSERT_TRUE(result);
    EXPECT_EQ(result, physics_ball.get());
}

TEST_F(SimulationContactListenerTest, test_is_ball_contact_with_reversed_args)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_chicker{PhysicsObjectType ::ROBOT_CHICKER, physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallContact(&user_data_chicker, &user_data_ball);

    ASSERT_TRUE(result);
    EXPECT_EQ(result, physics_ball.get());
}

TEST_F(SimulationContactListenerTest, test_is_ball_chicker_contact)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_chicker{PhysicsObjectType ::ROBOT_CHICKER, physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallChickerContact(&user_data_chicker, &user_data_ball);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
}

TEST_F(SimulationContactListenerTest, test_is_ball_chicker_contact_with_reversed_args)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_chicker{PhysicsObjectType ::ROBOT_CHICKER, physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallChickerContact(&user_data_ball, &user_data_chicker);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
}

TEST_F(SimulationContactListenerTest, test_is_ball_dribbler_contact)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_dribbler{PhysicsObjectType ::ROBOT_DRIBBLER, physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallDribblerContact(&user_data_ball, &user_data_dribbler);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
}

TEST_F(SimulationContactListenerTest, test_is_ball_dribbler_contact_with_reversed_args)
{
    // The positions of the objects don't matter for this test, only the user data
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [physics_world, physics_robot, physics_ball] = createWorld(robot, ball);

    PhysicsObjectUserData user_data_ball{PhysicsObjectType ::BALL, physics_ball.get()};
    PhysicsObjectUserData user_data_dribbler{PhysicsObjectType ::ROBOT_DRIBBLER, physics_robot.get()};

    SimulationContactListener listener;
    auto result = listener.isBallDribblerContact(&user_data_dribbler, &user_data_ball);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result->first);
    EXPECT_EQ(result->first, physics_ball.get());
    ASSERT_TRUE(result->second);
    EXPECT_EQ(result->second, physics_robot.get());
}
