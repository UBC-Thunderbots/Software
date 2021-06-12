#include "software/simulation/physics_simulator_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/simulation/physics/physics_world.h"
#include "software/simulation/physics_simulator_ball.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"
#include "software/world/world.h"

class PhysicsSimulatorRobotTest : public testing::Test
{
   protected:
    std::tuple<std::shared_ptr<PhysicsWorld>, std::shared_ptr<PhysicsSimulatorRobot>,
               std::shared_ptr<PhysicsSimulatorBall>>
    createWorld(Robot robot, Ball ball)
    {
        auto physics_world = std::make_shared<PhysicsWorld>(
            Field::createSSLDivisionBField(), std::make_shared<const PhysicsConfig>());
        physics_world->setBallState(ball.currentState());
        physics_world->addYellowRobots(
            {RobotStateWithId{.id = robot.id(), .robot_state = robot.currentState()}});

        std::shared_ptr<PhysicsSimulatorRobot> simulator_robot;
        auto physics_robot = physics_world->getYellowPhysicsRobots().at(0);
        if (physics_robot.lock())
        {
            simulator_robot = std::make_shared<PhysicsSimulatorRobot>(physics_robot);
        }
        else
        {
            ADD_FAILURE()
                << "Failed to create a PhysicsSimulatorRobot because a PhysicsRobot was invalid"
                << std::endl;
        }

        std::shared_ptr<PhysicsSimulatorBall> simulator_ball;
        auto physics_ball = physics_world->getPhysicsBall();
        if (physics_ball.lock())
        {
            simulator_ball = std::make_shared<PhysicsSimulatorBall>(physics_ball);
        }
        else
        {
            ADD_FAILURE()
                << "Failed to create a PhysicsSimulatorRobot because a PhysicsRobot was invalid"
                << std::endl;
        }

        return std::make_tuple(physics_world, simulator_robot, simulator_ball);
    }

    const Robot robot_non_zero_state =
        Robot(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12),
              AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
    const Ball ball_zero_state =
        Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
};

TEST_F(PhysicsSimulatorRobotTest, test_robot_id)
{
    auto [world, simulator_robot, simulator_ball] =
        createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getRobotId(), 7);
    UNUSED(world);
    UNUSED(simulator_ball);
}
