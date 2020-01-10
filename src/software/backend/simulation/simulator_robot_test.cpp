#include "software/backend/simulation/simulator_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <cmath>

#include "shared/constants.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"
#include "software/backend/simulation/physics/physics_simulator.h"
#include "app/world/firmware_robot.h"

// Roll the ball along the left side of the robot just inside of the robot radius
// and check it does collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_left_side_inside_radius)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 2)}, Timestamp::fromSeconds(0));

    PhysicsSimulator physics_simulator(world);
    auto friendly_physics_robots = physics_simulator.getFriendlyPhysicsRobots();

    FirmwareRobot_t *firmware_robot = nullptr;
    SimulatorRobot::setPhysicsRobots(friendly_physics_robots);
    for (const auto &physics_robot : friendly_physics_robots) {
        if(auto physics_robot_lock = physics_robot.lock()) {
            SimulatorRobot::setRobotId(physics_robot_lock->getRobotId());
            firmware_robot = SimulatorRobot::createFirmwareRobot();
            std::cout << app_firmware_robot_getPositionX(firmware_robot) << std::endl;
        }
    }
}

