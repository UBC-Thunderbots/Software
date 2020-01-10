#include "software/backend/simulation/simulator_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include <cmath>

#include "app/world/firmware_robot.h"
#include "shared/constants.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_simulator.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"

// This is a temporary tests to validate that the SimulatorRobot works as expected
// when binding and switching robots behind the scenes. It will be replaced with a
// better test once its functions are implemented properly
TEST(PhysicsRobotTest, test_simulator_robot_manages_multiple_robots_correctly)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 2)},
                                                        Timestamp::fromSeconds(0));

    PhysicsSimulator physics_simulator(world);
    auto friendly_physics_robots = physics_simulator.getFriendlyPhysicsRobots();

    FirmwareRobot_t *firmware_robot = nullptr;
    SimulatorRobot::setPhysicsRobots(friendly_physics_robots);
    for (const auto &physics_robot : friendly_physics_robots)
    {
        if (auto physics_robot_lock = physics_robot.lock())
        {
            SimulatorRobot::setRobotId(physics_robot_lock->getRobotId());
            firmware_robot = SimulatorRobot::createFirmwareRobot();
            std::cout << app_firmware_robot_getPositionX(firmware_robot) << std::endl;
        }
    }
}
