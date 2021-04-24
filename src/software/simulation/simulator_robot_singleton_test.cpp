#include "software/simulation/simulator_robot_singleton.h"

#include <gtest/gtest.h>

#include <cmath>

#include "shared/proto/robot_log_msg.nanopb.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "software/simulation/physics/physics_world.h"
#include "software/simulation/physics_simulator_ball.h"
#include "software/simulation/physics_simulator_robot.h"
#include "software/simulation/simulator_ball.h"
#include "software/simulation/simulator_robot.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/firmware_robot.h"
}

#include "shared/test_util/test_util.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"
#include "software/world/world.h"


class SimulatorRobotSingletonTest : public testing::Test
{
   protected:
    void SetUp() override
    {
        // We use the TestUtil::handleTestRobotLog function here
        // to not log the robot ID and the colour of the Robot, as
        // it's not relevant info in a test environment.
        app_logger_init(0, &TestUtil::handleTestRobotLog);
    }

    std::tuple<std::shared_ptr<PhysicsWorld>,
               std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>,
               std::shared_ptr<SimulatorBall>>
    createWorldWithEnemyRobots(Robot robot, Ball ball,
                               std::vector<Point> enemy_robot_positions)
    {
        auto physics_world = std::make_shared<PhysicsWorld>(
            Field::createSSLDivisionBField(), std::make_shared<const SimulatorConfig>());
        physics_world->setBallState(ball.currentState());
        RobotStateWithId robot_state{.id          = robot.id(),
                                     .robot_state = robot.currentState()};
        physics_world->addYellowRobots({robot_state});

        for (const auto& pos : enemy_robot_positions)
        {
            auto state = RobotStateWithId{
                .id          = physics_world->getAvailableBlueRobotId(),
                .robot_state = RobotState(pos, Vector(0, 0), Angle::zero(),
                                          AngularVelocity::zero())};
            physics_world->addBlueRobots({state});
        }

        std::shared_ptr<SimulatorRobot> simulator_robot;
        auto physics_robot = physics_world->getYellowPhysicsRobots().at(0);
        if (physics_robot.lock())
        {
            simulator_robot = std::make_shared<PhysicsSimulatorRobot>(physics_robot);
            SimulatorRobotSingleton::setSimulatorRobot(simulator_robot, FieldSide::NEG_X);
        }
        else
        {
            ADD_FAILURE()
                << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid"
                << std::endl;
        }

        std::shared_ptr<SimulatorBall> simulator_ball;
        auto physics_ball = physics_world->getPhysicsBall();
        if (physics_ball.lock())
        {
            simulator_ball = std::make_shared<PhysicsSimulatorBall>(physics_ball);
        }
        else
        {
            ADD_FAILURE()
                << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid"
                << std::endl;
        }

        return std::make_tuple(physics_world,
                               SimulatorRobotSingleton::createFirmwareRobot(),
                               simulator_ball);
    }

    std::tuple<std::shared_ptr<PhysicsWorld>,
               std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>,
               std::shared_ptr<SimulatorBall>>
    createWorld(Robot robot, Ball ball)
    {
        return createWorldWithEnemyRobots(robot, ball, {});
    }

    Point getDribblingPoint(
        const std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>& robot)
    {
        float position_x          = app_firmware_robot_getPositionX(robot.get());
        float position_y          = app_firmware_robot_getPositionY(robot.get());
        float orientation_radians = app_firmware_robot_getOrientation(robot.get());
        Point robot_position(static_cast<double>(position_x),
                             static_cast<double>(position_y));
        Angle robot_orientation =
            Angle::fromRadians(static_cast<double>(orientation_radians));
        return getDribblingPoint(robot_position, robot_orientation);
    }

    Point getDribblingPoint(const Point& robot_position, const Angle& robot_orientation)
    {
        double dribbler_depth = PhysicsRobot::DRIBBLER_DEPTH;
        Point dribbling_point =
            robot_position + Vector::createFromAngle(robot_orientation)
                                 .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                            BALL_MAX_RADIUS_METERS - dribbler_depth);
        return dribbling_point;
    }

    const Robot robot_non_zero_state =
        Robot(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12),
              AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
    const Ball ball_zero_state =
        Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
};

TEST_F(SimulatorRobotSingletonTest, test_get_position)
{
    auto [world, firmware_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(app_firmware_robot_getPositionX(firmware_robot.get()), 1.04f);
    EXPECT_FLOAT_EQ(app_firmware_robot_getPositionY(firmware_robot.get()), -0.8f);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_orientation)
{
    auto [world, firmware_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(app_firmware_robot_getOrientation(firmware_robot.get()), 2.12f);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_linear_velocity)
{
    auto [world, firmware_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_NEAR(app_firmware_robot_getVelocityX(firmware_robot.get()), -1.5, 0.01);
    EXPECT_NEAR(app_firmware_robot_getVelocityY(firmware_robot.get()), 0.0, 0.01);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_angular_velocity)
{
    auto [world, firmware_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(app_firmware_robot_getVelocityAngular(firmware_robot.get()), -1.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_battery_voltage)
{
    auto [world, firmware_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(app_firmware_robot_getBatteryVoltage(firmware_robot.get()), 16.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_dribbler_temperature)
{
    auto [world, firmware_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    EXPECT_EQ(app_dribbler_getTemperature(dribbler), 25);
    UNUSED(world);
    UNUSED(simulator_ball);
}

class SimulatorRobotSingletonKickTest : public SimulatorRobotSingletonTest,
                                        public ::testing::WithParamInterface<Angle>
{
};

TEST_P(SimulatorRobotSingletonKickTest, test_kick_stationary_ball_with_stationary_robot)
{
    Angle robot_orientation = GetParam();
    Robot robot(0, Point(0, 0), Vector(0, 0), robot_orientation, AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    // Simulate for 1/2 second without kicking
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't kick
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_kick(chicker, 5.0);

    // Simulate for 1/2 second after kicking
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT((simulator_ball->velocity() -
               Vector::createFromAngle(robot.orientation()).normalize(5))
                  .length(),
              0.005);
}

INSTANTIATE_TEST_CASE_P(All, SimulatorRobotSingletonKickTest,
                        ::testing::Values(Angle::fromDegrees(0), Angle::fromDegrees(58),
                                          Angle::fromDegrees(110),
                                          Angle::fromDegrees(200),
                                          Angle::fromDegrees(331)));


class SimulatorRobotSingletonChipTest : public SimulatorRobotSingletonTest,
                                        public ::testing::WithParamInterface<Angle>
{
};

TEST_P(SimulatorRobotSingletonChipTest, test_chip_stationary_ball_with_stationary_robot)
{
    Angle robot_orientation = GetParam();
    Robot robot(0, Point(0, 0), Vector(0, 0), robot_orientation, AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    // Simulate for 1/2 second without chipping
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't chip
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_chip(chicker, 5.0);

    // Simulate for 1/2 second after chipping
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().orientation().minDiff(robot_orientation),
              Angle::fromDegrees(1));
    EXPECT_GT(simulator_ball->velocity().length(), 3);
    EXPECT_LT(simulator_ball->velocity().length(), 5);
}

INSTANTIATE_TEST_CASE_P(All, SimulatorRobotSingletonChipTest,
                        ::testing::Values(Angle::fromDegrees(0), Angle::fromDegrees(58),
                                          Angle::fromDegrees(110),
                                          Angle::fromDegrees(200),
                                          Angle::fromDegrees(331)));


class SimulatorRobotSingletonAutokickTest : public SimulatorRobotSingletonTest,
                                            public ::testing::WithParamInterface<Angle>
{
};

TEST_P(SimulatorRobotSingletonAutokickTest,
       test_autokick_ball_moving_towards_stationary_robot)
{
    Angle robot_orientation = GetParam();
    Robot robot(0, Point(0, 0), Vector(0, 0), robot_orientation, AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point initial_ball_position =
        robot.position() + Vector::createFromAngle(robot_orientation).normalize(0.25);
    Vector initial_ball_velocity =
        (robot.position() - initial_ball_position).normalize(1.0);
    Ball ball(initial_ball_position, initial_ball_velocity, Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutokick(chicker, 5.0);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    Vector expected_velocity = Vector::createFromAngle(robot_orientation).normalize(5.03);
    EXPECT_LT((simulator_ball->velocity() - expected_velocity).length(), 0.001);
}

TEST_P(SimulatorRobotSingletonAutokickTest,
       test_autokick_ball_moving_towards_moving_robot)
{
    Angle robot_orientation = GetParam();
    Vector robot_velocity   = Vector::createFromAngle(robot_orientation).normalize(1.5);
    Robot robot(0, Point(0, 0), robot_velocity, robot_orientation,
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Point initial_ball_position =
        robot.position() + Vector::createFromAngle(robot_orientation).normalize(0.25);
    Vector initial_ball_velocity =
        (robot.position() - initial_ball_position).normalize(1.0);
    Ball ball(initial_ball_position, initial_ball_velocity, Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutokick(chicker, 5.0);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    Vector expected_velocity = Vector::createFromAngle(robot_orientation).normalize(5.16);
    EXPECT_LT((simulator_ball->velocity() - expected_velocity).length(), 0.01);
}

INSTANTIATE_TEST_CASE_P(All, SimulatorRobotSingletonAutokickTest,
                        ::testing::Values(Angle::fromDegrees(0), Angle::fromDegrees(58),
                                          Angle::fromDegrees(110),
                                          Angle::fromDegrees(200),
                                          Angle::fromDegrees(331)));

TEST_F(SimulatorRobotSingletonTest, autokick_ball_already_in_dribbler)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    // Simulate for 1/2 second without kicking
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't kick
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutokick(chicker, 5.0);

    // Simulate for 1/2 second after kicking
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT((simulator_ball->velocity() -
               Vector::createFromAngle(robot.orientation()).normalize(5))
                  .length(),
              0.01);
}

class SimulatorRobotSingletonAutochipTest : public SimulatorRobotSingletonTest,
                                            public ::testing::WithParamInterface<Angle>
{
};

TEST_P(SimulatorRobotSingletonAutochipTest,
       test_autochip_ball_moving_towards_stationary_robot_with_no_obstacle)
{
    Angle robot_orientation = GetParam();
    Robot robot(0, Point(0, 0), Vector(0, 0), robot_orientation, AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point initial_ball_position =
        robot.position() + Vector::createFromAngle(robot_orientation).normalize(0.25);
    Vector initial_ball_velocity =
        (robot.position() - initial_ball_position).normalize(1.0);
    Ball ball(initial_ball_position, initial_ball_velocity, Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutochip(chicker, 5.0);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().orientation().minDiff(robot_orientation),
              Angle::fromDegrees(1));
    EXPECT_NEAR(3.53, simulator_ball->velocity().length(), 0.05);
}

TEST_P(SimulatorRobotSingletonAutochipTest,
       test_autochip_ball_moving_towards_moving_robot_with_no_obstacle)
{
    Angle robot_orientation = GetParam();
    Vector robot_velocity   = Vector::createFromAngle(robot_orientation).normalize(1.5);
    Robot robot(0, Point(0, 0), robot_velocity, robot_orientation,
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Point initial_ball_position =
        robot.position() + Vector::createFromAngle(robot_orientation).normalize(0.25);
    Vector initial_ball_velocity =
        (robot.position() - initial_ball_position).normalize(1.0);
    Ball ball(initial_ball_position, initial_ball_velocity, Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutochip(chicker, 5.0);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().orientation().minDiff(robot_orientation),
              Angle::fromDegrees(1));
    EXPECT_NEAR(3.65, simulator_ball->velocity().length(), 0.05);
}

INSTANTIATE_TEST_CASE_P(All, SimulatorRobotSingletonAutochipTest,
                        ::testing::Values(Angle::fromDegrees(0), Angle::fromDegrees(58),
                                          Angle::fromDegrees(110),
                                          Angle::fromDegrees(200),
                                          Angle::fromDegrees(331)));

TEST_F(SimulatorRobotSingletonTest, autochip_ball_already_in_dribbler)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    // Simulate for 1/2 second without kicking
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't chip
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutochip(chicker, 5.0);

    // Simulate for 1/2 second after kicking
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().orientation().minDiff(robot.orientation()),
              Angle::fromDegrees(1));
    EXPECT_NEAR(3.53, simulator_ball->velocity().length(), 0.05);
}

TEST_F(SimulatorRobotSingletonTest,
       test_robot_chips_ball_over_obstacle_and_lands_in_free_space)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    // Add an enemy close to the chipping robot, and an enemy a short distance
    // past where the ball will land
    std::vector<Point> enemy_robot_positions = {Point(1, 0), Point(3.0, 0)};
    auto [world, firmware_robot, simulator_ball] =
        createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    // Simulate for 1/2 second without chipping
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't chip
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_chip(chicker, 2.0);

    for (unsigned int i = 0; i < 300; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // The ball should have made it past the first robot, but been stopped by the second
    EXPECT_GT(simulator_ball->position().x(), 1.0);
    EXPECT_LT(simulator_ball->position().x(), 3.0);
}

TEST_F(SimulatorRobotSingletonTest,
       test_robot_chips_ball_over_obstacle_and_lands_in_another_obstacle)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    // Add an enemy close to the chipping robot, an enemy right where
    // the ball will land, and an enemy past where the ball will land
    std::vector<Point> enemy_robot_positions = {Point(1, 0), Point(2, 0), Point(3, 0)};
    auto [world, firmware_robot, simulator_ball] =
        createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    // Simulate for 0.5 second without chipping
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't chip
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_chip(chicker, 2.0);

    for (unsigned int i = 0; i < 300; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // The ball should have made it past the first and second robots, but been stopped by
    // the third
    EXPECT_GT(simulator_ball->position().x(), 2.0);
    EXPECT_LT(simulator_ball->position().x(), 3.0);
}

TEST_F(SimulatorRobotSingletonTest, test_ball_maintains_speed_throughout_chipping)
{
    // This is a regression test to help catch future issues with chipping speed.
    // The original bug was that not all collisions were properly disabled while
    // the ball was "in flight", so when it passed over a robot it would still
    // be damped by the dribbler and slow down the chip. This test verifies
    // the speed of the ball does not change for the duration of a chip as the
    // ball passes over obstacles

    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    // Add an enemy close to the chipping robot
    std::vector<Point> enemy_robot_positions = {Point(1, 0)};
    auto [world, firmware_robot, simulator_ball] =
        createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    // Simulate for 1/2 second without chipping
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we didn't chip
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_chip(chicker, 2.0);

    double initial_chip_speed = 0.0;
    for (unsigned int i = 0; i < 200; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
        if (i == 0)
        {
            initial_chip_speed = simulator_ball->velocity().length();
        }
        if (i >= 5)
        {
            // Once the ball is chipped its speed should not change until it hits the
            // obstacle after it lands
            EXPECT_NEAR(initial_chip_speed, simulator_ball->velocity().length(), 1e-6);
        }
    }
}

TEST_F(SimulatorRobotSingletonTest,
       test_robot_does_not_kick_or_chip_ball_when_autokick_and_autochip_disabled)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);
    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_disableAutokick(chicker);
    app_chicker_disableAutochip(chicker);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().length(), 0.25);
}

TEST_F(SimulatorRobotSingletonTest, test_disable_autokick)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutokick(chicker, 5.0);
    app_chicker_disableAutokick(chicker);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().length(), 0.25);
}

TEST_F(SimulatorRobotSingletonTest, test_disable_autochip)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutochip(chicker, 5.0);
    app_chicker_disableAutochip(chicker);

    // Simulate for 1/2 second
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(simulator_ball->velocity().length(), 0.25);
}

TEST_F(SimulatorRobotSingletonTest, test_enabling_autokick_disables_autochip)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    // Place an enemy in front of the robot
    std::vector<Point> enemy_robot_positions = {Point(1.0, 0)};
    auto [world, firmware_robot, simulator_ball] =
        createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutochip(chicker, 5.0);
    app_chicker_enableAutokick(chicker, 5.0);

    // Simulate for 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // We expect the robot to kick the ball, so we expect the ball to have collided
    // with the enemy robot and not gone past it
    EXPECT_LT(simulator_ball->position().x(), 1.0);
}

TEST_F(SimulatorRobotSingletonTest, test_enabling_autochip_disables_autokick)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    // Place an enemy in front of the robot
    std::vector<Point> enemy_robot_positions = {Point(1.0, 0)};
    auto [world, firmware_robot, simulator_ball] =
        createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutokick(chicker, 5.0);
    app_chicker_enableAutochip(chicker, 5.0);

    // Simulate for 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // We expect the robot to chip the ball, so we expect the ball to have gone over
    // the enemy robot and made it past its position
    EXPECT_GT(simulator_ball->position().x(), 1.0);
}

TEST_F(SimulatorRobotSingletonTest,
       test_ball_does_not_bounce_off_front_of_robot_when_dribbler_on)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-3, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-3, 0)).length(), 0.001);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    // We use an arbitrarily large number here for speed
    app_dribbler_setSpeed(dribbler, 10000);

    // Simulate for 2 seconds
    for (unsigned int i = 0; i < 120; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Make sure we haven't caused the robot to move significantly by dribbling
    EXPECT_NEAR(app_firmware_robot_getPositionX(firmware_robot.get()), 0.0, 0.003);
    EXPECT_NEAR(app_firmware_robot_getPositionY(firmware_robot.get()), 0.0, 0.003);
    // Check the ball has stuck to the dribbler
    EXPECT_LT((simulator_ball->velocity() - Vector(0, 0)).length(), 0.001);
    Point dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);
}

TEST_F(SimulatorRobotSingletonTest, test_dribble_ball_while_moving_backwards)
{
    Robot robot(0, Point(0, 0), Vector(-0.5, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    // We use max force dribbler speed
    app_dribbler_setSpeed(dribbler, MAX_FORCE_DRIBBLER_SPEED);

    // Simulate for 2 seconds
    for (unsigned int i = 0; i < 120; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Check the ball has stuck to the dribbler
    float robot_velocity_x = app_firmware_robot_getVelocityX(firmware_robot.get());
    float robot_velocity_y = app_firmware_robot_getVelocityY(firmware_robot.get());
    Vector robot_velocity  = Vector(static_cast<double>(robot_velocity_x),
                                   static_cast<double>(robot_velocity_y));
    EXPECT_LT((simulator_ball->velocity() - robot_velocity).length(), 0.001);
    Point final_dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_LT((simulator_ball->position() - final_dribbling_point).length(), 0.01);
}

TEST_F(SimulatorRobotSingletonTest, test_losing_ball_while_zipping_backwards)
{
    Robot robot(0, Point(0, 0), Vector(-4, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    // We use max force dribbler speed
    app_dribbler_setSpeed(dribbler, MAX_FORCE_DRIBBLER_SPEED);

    // Simulate for 2 seconds
    for (unsigned int i = 0; i < 120; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Check the ball has not stuck to the dribbler
    Point final_dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_GT((simulator_ball->position() - final_dribbling_point).length(), 1);
}

TEST_F(SimulatorRobotSingletonTest, test_dribble_ball_while_moving_forwards)
{
    Robot robot(0, Point(0, 0), Vector(0.5, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    // We use an arbitrarily large number here for speed
    app_dribbler_setSpeed(dribbler, 10000);

    // Simulate for 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Check the ball has stuck to the dribbler
    float robot_velocity_x = app_firmware_robot_getVelocityX(firmware_robot.get());
    float robot_velocity_y = app_firmware_robot_getVelocityY(firmware_robot.get());
    Vector robot_velocity  = Vector(static_cast<double>(robot_velocity_x),
                                   static_cast<double>(robot_velocity_y));
    EXPECT_LT((simulator_ball->velocity() - robot_velocity).length(), 0.02);
    Point dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);
}

TEST_F(SimulatorRobotSingletonTest, test_dribble_ball_while_moving_spinning_in_place)
{
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.5, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    // We use an arbitrarily large number here for speed
    app_dribbler_setSpeed(dribbler, 1.5 * MAX_FORCE_DRIBBLER_SPEED);

    // Simulate for 0.5 second so the ball makes contact with the dribbler
    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Check the ball has stuck to the dribbler
    EXPECT_LT(simulator_ball->velocity().length(), 0.005);
    Point dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    // Accelerate the robot up to an angular velocity of 4*pi rad/s (ie. 2 rpm)
    float wheel_force = 0.3f;
    while (app_firmware_robot_getVelocityAngular(firmware_robot.get()) < 4 * M_PI)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_applyForce(front_left_wheel, wheel_force);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_applyForce(back_left_wheel, wheel_force);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_applyForce(back_right_wheel, wheel_force);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_applyForce(front_right_wheel, wheel_force);

        world->stepSimulation(Duration::fromSeconds(1.0 / 120.0));
        wheel_force += 0.0001f;
    }

    // Check the ball has stuck to the dribbler
    dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    // Simulate a bit long to check the ball remains stuck to the dribbler
    for (unsigned int i = 0; i < 120; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_applyForce(front_left_wheel, wheel_force);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_applyForce(back_left_wheel, wheel_force);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_applyForce(back_right_wheel, wheel_force);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_applyForce(front_right_wheel, wheel_force);

        world->stepSimulation(Duration::fromSeconds(1.0 / 120.0));
    }

    dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.015);
}

TEST_F(SimulatorRobotSingletonTest, test_dribbler_coast)
{
    Point robot_position    = Point(0, 0);
    Angle robot_orientation = Angle::zero();
    Point dribbling_point   = getDribblingPoint(robot_position, robot_orientation);
    Robot robot(0, robot_position, Vector(-0.5, 0), robot_orientation,
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(dribbling_point, Vector(-0.1, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    app_dribbler_coast(dribbler);

    // Simulate for 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Check the ball has not stuck to the dribbler
    dribbling_point = getDribblingPoint(firmware_robot);
    EXPECT_GT((simulator_ball->position() - dribbling_point).length(), 0.1);
}

TEST_F(SimulatorRobotSingletonTest, test_dribbler_centers_the_ball)
{
    Point robot_position    = Point(0, 0);
    Angle robot_orientation = Angle::zero();
    Point dribbling_point   = getDribblingPoint(robot_position, robot_orientation);
    Robot robot(0, robot_position, Vector(0, 0), robot_orientation,
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Start the ball at the side of the dribbler so that self-centering forces will be
    // applied
    Point ball_position = dribbling_point + Vector::createFromAngle(robot.orientation())
                                                .perpendicular()
                                                .normalize(DRIBBLER_WIDTH_METERS / 2.1 -
                                                           BALL_MAX_RADIUS_METERS);
    Ball ball(ball_position, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    app_dribbler_setSpeed(dribbler, 10000);

    for (unsigned int i = 0; i < 120; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 120.0));
    }

    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);
    EXPECT_LT(simulator_ball->velocity().length(), 0.01);
}

TEST_F(SimulatorRobotSingletonTest, test_kick_while_dribbler_on)
{
    Point robot_position    = Point(0, 0);
    Angle robot_orientation = Angle::zero();
    Point dribbling_point   = getDribblingPoint(robot_position, robot_orientation);
    Robot robot(0, robot_position, Vector(0, 0), robot_orientation,
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    app_dribbler_setSpeed(dribbler, 10000);

    // Simulate to make sure the ball is in control of the dribbler
    for (unsigned int i = 0; i < 60; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Makes sure the robot has the ball
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_kick(chicker, 5.0);

    for (unsigned int i = 0; i < 30; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_GT((simulator_ball->position() - dribbling_point).length(), 0.1);
    EXPECT_LT((simulator_ball->velocity() - Vector(5, 0)).length(), 0.1);
}

TEST_F(SimulatorRobotSingletonTest, test_angled_one_time_kick_while_dribbler_on)
{
    Point robot_position    = Point(0, -DIST_TO_FRONT_OF_ROBOT_METERS);
    Angle robot_orientation = Angle::zero();
    Point dribbling_point   = getDribblingPoint(robot_position, robot_orientation);
    Robot robot(0, robot_position, Vector(0, 0), robot_orientation,
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Point ball_position = Point(1, 1);
    Vector ball_vector  = (dribbling_point - ball_position).normalize(4);
    Ball ball(ball_position, ball_vector, Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot.get());
    app_dribbler_setSpeed(dribbler, 10000);
    Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot.get());
    app_chicker_enableAutokick(chicker, 5.0);

    for (unsigned int i = 0; i < 60; i++)
    {
        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // After the kick we expect the ball to be travelling in the x-direction at about the
    // speed it was kicked. We don't verify the y-component of the velocity since we don't
    // know exactly what angle the ball will be kicked at due to the damping
    EXPECT_NEAR(simulator_ball->velocity().x(), 5.0, 0.2);
}

TEST_F(SimulatorRobotSingletonTest, test_robot_drive_forward)
{
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_applyForce(front_left_wheel, -0.5);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_applyForce(back_left_wheel, -0.5);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_applyForce(back_right_wheel, 0.5);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_applyForce(front_right_wheel, 0.5);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_NEAR(app_firmware_robot_getVelocityX(firmware_robot.get()), 0, 1e-5);
    EXPECT_GT(app_firmware_robot_getVelocityY(firmware_robot.get()), 0.25);
    EXPECT_NEAR(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 0,
                1 * M_PI / 180.0);

    EXPECT_NEAR(app_firmware_robot_getPositionX(firmware_robot.get()), 0, 1e-5);
    EXPECT_GT(app_firmware_robot_getPositionY(firmware_robot.get()), 0.15);
    EXPECT_NEAR(app_firmware_robot_getOrientation(firmware_robot.get()), M_PI_2,
                1 * M_PI / 180.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_robot_drive_backwards)
{
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_applyForce(front_left_wheel, 0.5);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_applyForce(back_left_wheel, 0.5);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_applyForce(back_right_wheel, -0.5);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_applyForce(front_right_wheel, -0.5);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT(app_firmware_robot_getVelocityX(firmware_robot.get()), -0.25);
    EXPECT_NEAR(app_firmware_robot_getVelocityY(firmware_robot.get()), 0, 1e-5);
    EXPECT_NEAR(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 0,
                1 * M_PI / 180.0);

    EXPECT_LT(app_firmware_robot_getPositionX(firmware_robot.get()), -0.15);
    EXPECT_NEAR(app_firmware_robot_getPositionY(firmware_robot.get()), 0, 1e-5);
    EXPECT_NEAR(app_firmware_robot_getOrientation(firmware_robot.get()), 0,
                1 * M_PI / 180.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_robot_spin_clockwise)
{
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_applyForce(front_left_wheel, -0.5);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_applyForce(back_left_wheel, -0.5);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_applyForce(back_right_wheel, -0.5);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_applyForce(front_right_wheel, -0.5);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    float robot_position_x = app_firmware_robot_getPositionX(firmware_robot.get());
    float robot_position_y = app_firmware_robot_getPositionY(firmware_robot.get());
    Point robot_position   = Point(static_cast<double>(robot_position_x),
                                 static_cast<double>(robot_position_y));
    EXPECT_LT((robot_position - Point(0, 0)).length(), 0.05);
    float robot_velocity_x = app_firmware_robot_getVelocityX(firmware_robot.get());
    float robot_velocity_y = app_firmware_robot_getVelocityY(firmware_robot.get());
    Vector robot_velocity  = Vector(static_cast<double>(robot_velocity_x),
                                   static_cast<double>(robot_velocity_y));
    EXPECT_LT((robot_velocity - Vector(0, 0)).length(), 0.05);
    EXPECT_LT(app_firmware_robot_getVelocityAngular(firmware_robot.get()), -8);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_robot_spin_counterclockwise)
{
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_applyForce(front_left_wheel, 0.5);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_applyForce(back_left_wheel, 0.5);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_applyForce(back_right_wheel, 0.5);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_applyForce(front_right_wheel, 0.5);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    float robot_position_x = app_firmware_robot_getPositionX(firmware_robot.get());
    float robot_position_y = app_firmware_robot_getPositionY(firmware_robot.get());
    Point robot_position   = Point(static_cast<double>(robot_position_x),
                                 static_cast<double>(robot_position_y));
    EXPECT_LT((robot_position - Point(0, 0)).length(), 0.05);
    float robot_velocity_x = app_firmware_robot_getVelocityX(firmware_robot.get());
    float robot_velocity_y = app_firmware_robot_getVelocityY(firmware_robot.get());
    Vector robot_velocity  = Vector(static_cast<double>(robot_velocity_x),
                                   static_cast<double>(robot_velocity_y));
    EXPECT_LT((robot_velocity - Vector(0, 0)).length(), 0.05);
    EXPECT_GT(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 8);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_motor_speeds_when_robot_not_moving)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Wheel_t* front_left_wheel =
        app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
    float motor_speed_front_left = app_wheel_getMotorSpeedRPM(front_left_wheel);
    Wheel_t* back_left_wheel = app_firmware_robot_getBackLeftWheel(firmware_robot.get());
    float motor_speed_back_left = app_wheel_getMotorSpeedRPM(back_left_wheel);
    Wheel_t* back_right_wheel =
        app_firmware_robot_getBackRightWheel(firmware_robot.get());
    float motor_speed_back_right = app_wheel_getMotorSpeedRPM(back_right_wheel);
    Wheel_t* front_right_wheel =
        app_firmware_robot_getFrontRightWheel(firmware_robot.get());
    float motor_speed_front_right = app_wheel_getMotorSpeedRPM(front_right_wheel);
    EXPECT_EQ(motor_speed_front_left, 0.0);
    EXPECT_EQ(motor_speed_back_left, 0.0);
    EXPECT_EQ(motor_speed_back_right, 0.0);
    EXPECT_EQ(motor_speed_front_right, 0.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_motor_speeds_when_robot_moving_forwards)
{
    Robot robot(0, Point(0, 0), Vector(1, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Wheel_t* front_left_wheel =
        app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
    float motor_speed_front_left = app_wheel_getMotorSpeedRPM(front_left_wheel);
    Wheel_t* back_left_wheel = app_firmware_robot_getBackLeftWheel(firmware_robot.get());
    float motor_speed_back_left = app_wheel_getMotorSpeedRPM(back_left_wheel);
    Wheel_t* back_right_wheel =
        app_firmware_robot_getBackRightWheel(firmware_robot.get());
    float motor_speed_back_right = app_wheel_getMotorSpeedRPM(back_right_wheel);
    Wheel_t* front_right_wheel =
        app_firmware_robot_getFrontRightWheel(firmware_robot.get());
    float motor_speed_front_right = app_wheel_getMotorSpeedRPM(front_right_wheel);

    EXPECT_LT(motor_speed_front_left, -1.0);
    EXPECT_LT(motor_speed_back_left, -1.0);
    EXPECT_GT(motor_speed_back_right, 1.0);
    EXPECT_GT(motor_speed_front_right, 1.0);
    EXPECT_LT(motor_speed_front_left, motor_speed_back_left);
    EXPECT_GT(motor_speed_front_right, motor_speed_back_right);
    EXPECT_FLOAT_EQ(motor_speed_front_left, -motor_speed_front_right);
    EXPECT_FLOAT_EQ(motor_speed_back_left, -motor_speed_back_right);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest,
       test_get_motor_speeds_when_robot_moving_along_wheel_axis)
{
    // Move along the axis of the front-left wheel. This means the front-left wheel is
    // perpendicular to the direction of motion, and we don't expect it to be spinning
    Vector robot_velocity =
        Vector::createFromAngle(Angle::fromDegrees(ANGLE_TO_ROBOT_FRONT_WHEELS_DEG));
    Robot robot(0, Point(0, 0), robot_velocity, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Wheel_t* front_left_wheel =
        app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
    float motor_speed_front_left = app_wheel_getMotorSpeedRPM(front_left_wheel);
    Wheel_t* back_left_wheel = app_firmware_robot_getBackLeftWheel(firmware_robot.get());
    float motor_speed_back_left = app_wheel_getMotorSpeedRPM(back_left_wheel);
    Wheel_t* back_right_wheel =
        app_firmware_robot_getBackRightWheel(firmware_robot.get());
    float motor_speed_back_right = app_wheel_getMotorSpeedRPM(back_right_wheel);
    Wheel_t* front_right_wheel =
        app_firmware_robot_getFrontRightWheel(firmware_robot.get());
    float motor_speed_front_right = app_wheel_getMotorSpeedRPM(front_right_wheel);

    EXPECT_NEAR(motor_speed_front_left, 0.0, 0.1);
    EXPECT_LT(motor_speed_back_left, -1.0);
    EXPECT_LT(motor_speed_back_right, -0.1);
    EXPECT_GT(motor_speed_front_right, 1.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_get_motor_speeds_when_robot_spinning)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                AngularVelocity::threeQuarter(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    Wheel_t* front_left_wheel =
        app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
    float motor_speed_front_left = app_wheel_getMotorSpeedRPM(front_left_wheel);
    Wheel_t* back_left_wheel = app_firmware_robot_getBackLeftWheel(firmware_robot.get());
    float motor_speed_back_left = app_wheel_getMotorSpeedRPM(back_left_wheel);
    Wheel_t* back_right_wheel =
        app_firmware_robot_getBackRightWheel(firmware_robot.get());
    float motor_speed_back_right = app_wheel_getMotorSpeedRPM(back_right_wheel);
    Wheel_t* front_right_wheel =
        app_firmware_robot_getFrontRightWheel(firmware_robot.get());
    float motor_speed_front_right = app_wheel_getMotorSpeedRPM(front_right_wheel);

    EXPECT_GT(motor_speed_front_left, 1.0);
    EXPECT_GT(motor_speed_back_left, 1.0);
    EXPECT_GT(motor_speed_back_right, 1.0);
    EXPECT_GT(motor_speed_front_right, 1.0);
    UNUSED(world);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest,
       test_brake_motors_when_robot_spinning_with_positive_angular_velocity)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                AngularVelocity::threeQuarter(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_brake(front_left_wheel);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_brake(back_left_wheel);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_brake(back_right_wheel);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_brake(front_right_wheel);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_NEAR(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 0.0,
                1.0 * M_PI / 180.0);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest,
       test_brake_motors_when_robot_spinning_with_negative_angular_velocity)
{
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                AngularVelocity::fromDegrees(-2 * 360), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_brake(front_left_wheel);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_brake(back_left_wheel);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_brake(back_right_wheel);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_brake(front_right_wheel);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_NEAR(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 0.0,
                1.0 * M_PI / 180.0);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_brake_motors_when_robot_moving_linearly)
{
    Robot robot(0, Point(0, 0), Vector(2.5, 1.0), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 240; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_brake(front_left_wheel);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_brake(back_left_wheel);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_brake(back_right_wheel);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_brake(front_right_wheel);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    float robot_velocity_x = app_firmware_robot_getVelocityX(firmware_robot.get());
    float robot_velocity_y = app_firmware_robot_getVelocityY(firmware_robot.get());
    Vector robot_velocity  = Vector(static_cast<double>(robot_velocity_x),
                                   static_cast<double>(robot_velocity_y));
    EXPECT_LT((robot_velocity - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 0.0,
                1.0 * M_PI / 180.0);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_brake_motors_when_robot_moving_and_spinning)
{
    Robot robot(0, Point(0, 0), Vector(2.5, 1.0), Angle::threeQuarter(),
                AngularVelocity::full(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, firmware_robot, simulator_ball] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 240; i++)
    {
        Wheel_t* front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot.get());
        app_wheel_brake(front_left_wheel);
        Wheel_t* back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot.get());
        app_wheel_brake(back_left_wheel);
        Wheel_t* back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot.get());
        app_wheel_brake(back_right_wheel);
        Wheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot.get());
        app_wheel_brake(front_right_wheel);

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    float robot_velocity_x = app_firmware_robot_getVelocityX(firmware_robot.get());
    float robot_velocity_y = app_firmware_robot_getVelocityY(firmware_robot.get());
    Vector robot_velocity  = Vector(static_cast<double>(robot_velocity_x),
                                   static_cast<double>(robot_velocity_y));
    EXPECT_LT((robot_velocity - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(app_firmware_robot_getVelocityAngular(firmware_robot.get()), 0.0,
                1.0 * M_PI / 180.0);
    UNUSED(simulator_ball);
}

TEST_F(SimulatorRobotSingletonTest, test_change_simulator_robot)
{
    auto physics_world = std::make_unique<PhysicsWorld>(
        Field::createSSLDivisionBField(), std::make_shared<const SimulatorConfig>());
    auto robot_states = std::vector<RobotStateWithId>{
        RobotStateWithId{.id          = 7,
                         .robot_state = RobotState(Point(1.2, 0), Vector(-2.3, 0.2),
                                                   Angle::fromRadians(-1.2),
                                                   AngularVelocity::quarter())},
        RobotStateWithId{
            .id          = 2,
            .robot_state = RobotState(Point(0, -4.03), Vector(0, 1),
                                      Angle::fromRadians(0.3), AngularVelocity::half())}};
    physics_world->addYellowRobots(robot_states);

    auto friendly_physics_robots = physics_world->getYellowPhysicsRobots();
    ASSERT_EQ(2, friendly_physics_robots.size());
    auto simulator_robot_7 =
        std::make_shared<PhysicsSimulatorRobot>(friendly_physics_robots.at(0));

    SimulatorRobotSingleton::setSimulatorRobot(simulator_robot_7, FieldSide::NEG_X);
    auto firmware_robot_7 = SimulatorRobotSingleton::createFirmwareRobot();
    EXPECT_FLOAT_EQ(1.2f, app_firmware_robot_getPositionX(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(0.0f, app_firmware_robot_getPositionY(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(-2.3f, app_firmware_robot_getVelocityX(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(0.2f, app_firmware_robot_getVelocityY(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(-1.2f, app_firmware_robot_getOrientation(firmware_robot_7.get()));

    // The firmware functions should now return the data for simulator_robot_2, even
    // though we didn't need to create a new FirmwareRobot_t
    auto simulator_robot_2 =
        std::make_shared<PhysicsSimulatorRobot>(friendly_physics_robots.at(1));
    SimulatorRobotSingleton::setSimulatorRobot(simulator_robot_2, FieldSide::NEG_X);
    auto firmware_robot_2 = SimulatorRobotSingleton::createFirmwareRobot();
    EXPECT_FLOAT_EQ(0.0f, app_firmware_robot_getPositionX(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(-4.03f, app_firmware_robot_getPositionY(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(0.0f, app_firmware_robot_getVelocityX(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(1.0f, app_firmware_robot_getVelocityY(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(0.3f, app_firmware_robot_getOrientation(firmware_robot_2.get()));
}
