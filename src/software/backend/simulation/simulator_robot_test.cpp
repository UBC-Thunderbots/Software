#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/physics/physics_world.h"
#include "software/world/robot.h"
#include "software/world/world.h"
#include "software/test_util/test_util.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

class SimulatorRobotTest : public testing::Test
{
protected:
//    virtual void SetUp()
//    {
//        World world = ::Test::TestUtil::createBlankTestingWorld();
//
//        Robot robot_non_zero_state(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12),
//                              AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
//        Robot robot_zero_state(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
//
//        world.mutableFriendlyTeam().updateRobots({
//            robot_zero_state,
//            robot_non_zero_state
//        });
//        physics_world = std::make_shared<PhysicsWorld>(world);
//        auto physics_robots = physics_world->getFriendlyPhysicsRobots();
//        for(const auto& physics_robot : physics_world->getFriendlyPhysicsRobots()) {
//            if(auto robot = physics_robot.lock()) {
//                if(robot->getRobotId() == 0) {
//                    simulator_robot_zero_state = SimulatorRobot(physics_robot);
//                }else if(robot->getRobotId() == 7) {
//                    simulator_robot_non_zero_state = SimulatorRobot(physics_robot);
//                }
//            }
//            else {
//                ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
//            }
//        }
//
//        if(physics_world->getPhysicsBall().lock()) {
//            simulator_ball = SimulatorBall(physics_world->getPhysicsBall());
//        }
//    }
//
//    SimulatorRobot createSimulatorRobot(const Robot& robot) {
//
//
//    }

//    std::shared_ptr<PhysicsWorld> physics_world;
//    SimulatorRobot simulator_robot_non_zero_state;
//    SimulatorRobot simulator_robot_zero_state;
//    SimulatorBall simulator_ball;


    const Robot robot_non_zero_state = Robot(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12), AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
    const Ball ball_zero_state = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));

    std::tuple<std::shared_ptr<PhysicsWorld>, SimulatorRobot, SimulatorBall> createWorld(Robot robot, Ball ball) {
        World world = ::Test::TestUtil::createBlankTestingWorld();

        world.mutableFriendlyTeam().updateRobots({robot});
        world.mutableBall() = ball;
        auto physics_world = std::make_shared<PhysicsWorld>(world);

        SimulatorRobot simulator_robot;
        auto physics_robot = physics_world->getFriendlyPhysicsRobots().at(0);
        if(physics_robot.lock()) {
            simulator_robot = SimulatorRobot(physics_robot);
        }else {
            ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
        }

        SimulatorBall simulator_ball;
        auto physics_ball = physics_world->getPhysicsBall();
        if(physics_ball.lock()) {
            simulator_ball = SimulatorBall(physics_ball);
        }else {
            ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
        }

        return std::make_tuple(physics_world, simulator_robot, simulator_ball);
    }
};

TEST_F(SimulatorRobotTest, test_robot_id) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot.getRobotId(), 7);
}

TEST_F(SimulatorRobotTest, test_get_position) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot.getPositionX(), 1.04);
    EXPECT_FLOAT_EQ(simulator_robot.getPositionY(), -0.8);
}

TEST_F(SimulatorRobotTest, test_get_orientation) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot.getOrientation(), 2.12);
}

TEST_F(SimulatorRobotTest, test_get_linear_velocity) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_NEAR(simulator_robot.getVelocityX(), -1.5, 0.01);
    EXPECT_NEAR(simulator_robot.getVelocityY(), 0.0, 0.01);
}

TEST_F(SimulatorRobotTest, test_get_angular_velocity) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot.getVelocityAngular(), -1.0);
}

TEST_F(SimulatorRobotTest, test_get_battery_voltage) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot.getBatteryVoltage(), 16.0);
}

TEST_F(SimulatorRobotTest, test_enabling_and_disabling_autokick) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FALSE(simulator_robot.getAutokickSpeed());
    simulator_robot.enableAutokick(1.5);
    ASSERT_TRUE(simulator_robot.getAutokickSpeed());
    EXPECT_NEAR(simulator_robot.getAutokickSpeed().value(), 1.5, 1e-6);
    simulator_robot.disableAutokick();
    EXPECT_FALSE(simulator_robot.getAutokickSpeed());
}

TEST_F(SimulatorRobotTest, test_enabling_and_disabling_autochip) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FALSE(simulator_robot.getAutochipDistance());
    simulator_robot.enableAutochip(2.1);
    ASSERT_TRUE(simulator_robot.getAutochipDistance());
    EXPECT_NEAR(simulator_robot.getAutochipDistance().value(), 2.1, 1e-6);
    simulator_robot.disableAutochip();
    EXPECT_FALSE(simulator_robot.getAutochipDistance());
}

TEST_F(SimulatorRobotTest, test_changing_dribbler_speed) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 0);
    simulator_robot.setDribblerSpeed(50);
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 50);
    simulator_robot.setDribblerSpeed(10499);
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 10499);
    simulator_robot.setDribblerSpeed(0);
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 0);
}

TEST_F(SimulatorRobotTest, test_dribbler_coast) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 0);
    simulator_robot.setDribblerSpeed(50);
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 50);
    simulator_robot.dribblerCoast();
    EXPECT_EQ(simulator_robot.getDribblerSpeed(), 0);
}

TEST_F(SimulatorRobotTest, test_get_dribbler_temperature) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot.getDribblerTemperatureDegC(), 25);
}

TEST_F(SimulatorRobotTest, test_robot_kicks_ball_with_autokick_enabled) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball.velocity() - Vector(-1, 0)).length(), 0.001);

    simulator_robot.enableAutokick(5.0);

    // Simulate for 1 second
    for(unsigned int i = 0; i < 60; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    auto v = simulator_ball.velocity();
    EXPECT_LT((v - Vector(4, 0)).length(), 0.001);
}
