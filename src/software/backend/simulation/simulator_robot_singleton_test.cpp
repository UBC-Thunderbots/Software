#include "software/backend/simulation/simulator_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include <cmath>

extern "C"
{
#include "firmware/main/app/world/firmware_robot.h"
}
#include "shared/constants.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_simulator.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"

// This is a temporary tests to validate that the SimulatorRobotSingleton works as
// expected when binding and switching robots behind the scenes. It will be replaced with
// a better test once its functions are implemented properly
TEST(PhysicsRobotTest, test_simulator_robot_manages_multiple_robots_correctly)
{
//    World world = ::Test::TestUtil::createBlankTestingWorld();
//    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 2)},
//                                                        Timestamp::fromSeconds(0));
//
//    PhysicsSimulator physics_simulator(world);
//    auto friendly_physics_robots = physics_simulator.getFriendlyPhysicsRobots();
//
//    SimulatorRobotSingleton::setPhysicsRobots(friendly_physics_robots);
//    for (const auto &physics_robot : friendly_physics_robots)
//    {
//        if (auto physics_robot_lock = physics_robot.lock())
//        {
//            SimulatorRobotSingleton::setRobotId(physics_robot_lock->getRobotId());
//            auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
//            std::cout << app_firmware_robot_getPositionX(firmware_robot.get())
//                      << std::endl;
//        }
//    }
}
