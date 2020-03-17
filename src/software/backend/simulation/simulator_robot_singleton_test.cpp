#include "software/backend/simulation/simulator_robot_singleton.h"

#include <gtest/gtest.h>

#include <cmath>

#include "software/backend/simulation/physics/physics_world.h"
#include "software/backend/simulation/simulator_robot.h"

extern "C"
{
#include "firmware/app/world/firmware_robot.h"
}

#include "software/test_util/test_util.h"
#include "software/world/robot.h"
#include "software/world/world.h"

TEST(PhysicsRobotTest, test_create_firmware_robot_with_single_simulator_robot)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot(7, Point(1.2, 0), Vector(-2.3, 0.2), Angle::fromRadians(-1.2),
                AngularVelocity::quarter(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    auto physics_world           = std::make_unique<PhysicsWorld>(world);
    auto friendly_physics_robots = physics_world->getFriendlyPhysicsRobots();
    ASSERT_EQ(1, friendly_physics_robots.size());
    auto simulator_robot =
        std::make_shared<SimulatorRobot>(friendly_physics_robots.at(0));

    SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
    auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
    EXPECT_FLOAT_EQ(1.2, app_firmware_robot_getPositionX(firmware_robot.get()));
    EXPECT_FLOAT_EQ(0, app_firmware_robot_getPositionY(firmware_robot.get()));
    EXPECT_FLOAT_EQ(-2.3, app_firmware_robot_getVelocityX(firmware_robot.get()));
    EXPECT_FLOAT_EQ(0.2, app_firmware_robot_getVelocityY(firmware_robot.get()));
    EXPECT_FLOAT_EQ(-1.2, app_firmware_robot_getOrientation(firmware_robot.get()));
}

TEST(PhysicsRobotTest, test_change_simulator_robot)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot7(7, Point(1.2, 0), Vector(-2.3, 0.2), Angle::fromRadians(-1.2),
                 AngularVelocity::quarter(), Timestamp::fromSeconds(0));
    Robot robot2(2, Point(0, -4.03), Vector(0, 1), Angle::fromRadians(0.3),
                 AngularVelocity::half(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot7, robot2});

    auto physics_world           = std::make_unique<PhysicsWorld>(world);
    auto friendly_physics_robots = physics_world->getFriendlyPhysicsRobots();
    ASSERT_EQ(2, friendly_physics_robots.size());
    auto simulator_robot_7 =
        std::make_shared<SimulatorRobot>(friendly_physics_robots.at(0));

    SimulatorRobotSingleton::setSimulatorRobot(simulator_robot_7);
    auto firmware_robot_7 = SimulatorRobotSingleton::createFirmwareRobot();
    EXPECT_FLOAT_EQ(1.2, app_firmware_robot_getPositionX(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(0, app_firmware_robot_getPositionY(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(-2.3, app_firmware_robot_getVelocityX(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(0.2, app_firmware_robot_getVelocityY(firmware_robot_7.get()));
    EXPECT_FLOAT_EQ(-1.2, app_firmware_robot_getOrientation(firmware_robot_7.get()));

    // The firmware functions should now return the data for simulator_robot_2, even
    // though we didn't need to create a new FirmwareRobot_t
    auto simulator_robot_2 =
        std::make_shared<SimulatorRobot>(friendly_physics_robots.at(1));
    SimulatorRobotSingleton::setSimulatorRobot(simulator_robot_2);
    auto firmware_robot_2 = SimulatorRobotSingleton::createFirmwareRobot();
    EXPECT_FLOAT_EQ(0, app_firmware_robot_getPositionX(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(-4.03, app_firmware_robot_getPositionY(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(0, app_firmware_robot_getVelocityX(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(1, app_firmware_robot_getVelocityY(firmware_robot_2.get()));
    EXPECT_FLOAT_EQ(0.3, app_firmware_robot_getOrientation(firmware_robot_2.get()));
}
