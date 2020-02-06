#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/physics/physics_world.h"
#include "software/world/robot.h"
#include "software/world/world.h"
#include "software/test_util/test_util.h"
#include "shared/constants.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

class SimulatorRobotTest : public testing::Test
{
protected:
    /**
     * TODO
     * @param robot
     * @param ball
     * @param enemy_robot_positions
     * @return
     */
    std::tuple<std::shared_ptr<PhysicsWorld>, std::shared_ptr<SimulatorRobot>, std::shared_ptr<SimulatorBall>> createWorldWithEnemyRobots(Robot robot, Ball ball, std::vector<Point> enemy_robot_positions) {
        World world = ::Test::TestUtil::createBlankTestingWorld();
        world = ::Test::TestUtil::setEnemyRobotPositions(world, enemy_robot_positions, Timestamp::fromSeconds(0));

        world.mutableFriendlyTeam().updateRobots({robot});
        world.mutableBall() = ball;
        auto physics_world = std::make_shared<PhysicsWorld>(world);

        std::shared_ptr<SimulatorRobot> simulator_robot;
        auto physics_robot = physics_world->getFriendlyPhysicsRobots().at(0);
        if(physics_robot.lock()) {
            simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);
        }else {
            ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
        }

        std::shared_ptr<SimulatorBall> simulator_ball;
        auto physics_ball = physics_world->getPhysicsBall();
        if(physics_ball.lock()) {
            simulator_ball = std::make_shared<SimulatorBall>(physics_ball);
        }else {
            ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
        }

        return std::make_tuple(physics_world, simulator_robot, simulator_ball);
    }

    std::tuple<std::shared_ptr<PhysicsWorld>, std::shared_ptr<SimulatorRobot>, std::shared_ptr<SimulatorBall>> createWorld(Robot robot, Ball ball) {
        return createWorldWithEnemyRobots(robot, ball, {});
    }

    const Robot robot_non_zero_state = Robot(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12), AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
    const Ball ball_zero_state = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
};

TEST_F(SimulatorRobotTest, test_robot_id) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getRobotId(), 7);
}

TEST_F(SimulatorRobotTest, test_get_position) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getPositionX(), 1.04);
    EXPECT_FLOAT_EQ(simulator_robot->getPositionY(), -0.8);
}

TEST_F(SimulatorRobotTest, test_get_orientation) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getOrientation(), 2.12);
}

TEST_F(SimulatorRobotTest, test_get_linear_velocity) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_NEAR(simulator_robot->getVelocityX(), -1.5, 0.01);
    EXPECT_NEAR(simulator_robot->getVelocityY(), 0.0, 0.01);
}

TEST_F(SimulatorRobotTest, test_get_angular_velocity) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getVelocityAngular(), -1.0);
}

TEST_F(SimulatorRobotTest, test_get_battery_voltage) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getBatteryVoltage(), 16.0);
}

TEST_F(SimulatorRobotTest, test_enabling_and_disabling_autokick) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FALSE(simulator_robot->getAutokickSpeed());
    simulator_robot->enableAutokick(1.5);
    ASSERT_TRUE(simulator_robot->getAutokickSpeed());
    EXPECT_NEAR(simulator_robot->getAutokickSpeed().value(), 1.5, 1e-6);
    simulator_robot->disableAutokick();
    EXPECT_FALSE(simulator_robot->getAutokickSpeed());
}

TEST_F(SimulatorRobotTest, test_enabling_and_disabling_autochip) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FALSE(simulator_robot->getAutochipDistance());
    simulator_robot->enableAutochip(2.1);
    ASSERT_TRUE(simulator_robot->getAutochipDistance());
    EXPECT_NEAR(simulator_robot->getAutochipDistance().value(), 2.1, 1e-6);
    simulator_robot->disableAutochip();
    EXPECT_FALSE(simulator_robot->getAutochipDistance());
}

TEST_F(SimulatorRobotTest, test_changing_dribbler_speed) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 0);
    simulator_robot->setDribblerSpeed(50);
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 50);
    simulator_robot->setDribblerSpeed(10499);
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 10499);
    simulator_robot->setDribblerSpeed(0);
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 0);
}

TEST_F(SimulatorRobotTest, test_dribbler_coast) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 0);
    simulator_robot->setDribblerSpeed(50);
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 50);
    simulator_robot->dribblerCoast();
    EXPECT_EQ(simulator_robot->getDribblerSpeed(), 0);
}

TEST_F(SimulatorRobotTest, test_get_dribbler_temperature) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getDribblerTemperatureDegC(), 25);
}

TEST_F(SimulatorRobotTest, test_robot_does_not_kick_ball_when_autokick_disabled) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);

    // Simulate for 1/2 second
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT(simulator_ball->velocity().length(), 0.25);
}

TEST_F(SimulatorRobotTest, test_robot_kicks_ball_with_autokick_enabled) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-1, 0)).length(), 0.001);

    simulator_robot->enableAutokick(5);

    // Simulate for 1/2 second
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT((simulator_ball->velocity() - Vector(4, 0)).length(), 0.001);
}

TEST_F(SimulatorRobotTest, test_robot_chips_ball_with_autochip_enabled_and_ball_does_not_land_in_obstacle) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    std::vector<Point> enemy_robot_positions = {Point(0.4, 0), Point(1.9, 0)};
    auto [world, simulator_robot, simulator_ball] = createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);

    simulator_robot->enableAutochip(2);

    // Simulate for 1.5 seconds
    for(unsigned int i = 0; i < 90; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_GT(simulator_ball->velocity().x(), 1);
    EXPECT_GT(simulator_ball->position().x(), 2.0);
}

TEST_F(SimulatorRobotTest, test_robot_chips_ball_with_autochip_enabled_and_ball_lands_on_obstacle) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    // Add an enemy robot right where the ball will land
    std::vector<Point> enemy_robot_positions = {Point(2.0, 0)};
    auto [world, simulator_robot, simulator_ball] = createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);

    simulator_robot->enableAutochip(2);

    // Simulate for 1.5 seconds
    for(unsigned int i = 0; i < 90; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball's velocity is very roughly within a reasonable range. We don't care
    // exactly what it is, just that it hasn't changed an impossible amount by
    // getting caught inside the robot it landed in and then squeezed out
    EXPECT_GT(simulator_ball->velocity().x(), 1);
    EXPECT_LT(simulator_ball->velocity().x(), 3);
    EXPECT_GT(simulator_ball->position().x(), 2.0);
}

TEST_F(SimulatorRobotTest, test_ball_bounces_off_front_of_robot_when_dribbler_not_on) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.5, 0), Vector(-3, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-3, 0)).length(), 0.001);

    // Simulate for 1 second
    for(unsigned int i = 0; i < 60; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball has bounced of the robot and is moving away from it
    EXPECT_GT(simulator_ball->velocity().x(), 1.0);
}

TEST_F(SimulatorRobotTest, test_ball_does_not_bounce_off_front_of_robot_when_dribbler_on) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-0.5, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

//    EXPECT_LT((simulator_ball->velocity() - Vector(-3, 0)).length(), 0.001);

    // We use an arbitrarily large number here for speed
    simulator_robot->setDribblerSpeed(10000);

    // Simulate for 1 second
    for(unsigned int i = 0; i < 150; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
        std::cout << simulator_ball->position() << "  ,   " << simulator_ball->velocity() << std::endl;
    }

    // Check the ball has stuck to the dribbler
    EXPECT_NEAR(simulator_ball->velocity().x(), 0.0, 0.01);
//    EXPECT_LT((simulator_ball->position() - Point(DIST_TO_FRONT_OF_ROBOT_METERS, 0)).length(), 0.03);
}
