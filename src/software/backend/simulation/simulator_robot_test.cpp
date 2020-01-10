#include "software/backend/simulation/simulator_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "shared/constants.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"
#include "software/backend/simulation/physics/physics_simulator.h"

// Roll the ball along the left side of the robot just inside of the robot radius
// and check it does collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_left_side_inside_radius)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 2)}, Timestamp::fromSeconds(0));

    PhysicsSimulator physics_simulator(world);
    auto friendly_physics_robots = physics_simulator.getFriendlyPhysicsRobots();


//    // We have to take lots of small steps because a significant amount of accuracy
//    // is lost if we take a single step of 1 second
//    for (unsigned int i = 0; i < 60; i++)
//    {
//        // 5 and 8 here are somewhat arbitrary values for the velocity and position
//        // iterations but are the recommended defaults from
//        // https://www.iforce2d.net/b2dtut/worlds
//        world->Step(1.0 / 60.0, 5, 8);
//    }
//
//    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
//    EXPECT_GT(ball.velocity().orientation().minDiff(Angle::half()).toDegrees(), 0.1);
}

