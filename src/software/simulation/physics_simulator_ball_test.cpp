#include "software/simulation/physics_simulator_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "shared/2015_robot_constants.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_world.h"
#include "software/test_util/test_util.h"

class PhysicsSimulatorBallTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        physics_world = std::make_unique<PhysicsWorld>(Field::createSSLDivisionBField(),
                                                       robot_constants, wheel_constants,
                                                       TbotsProto::SimulatorConfig());
        physics_world->setBallState(BallState(Point(1.01, -0.4), Vector(0.02, -4.5)));

        auto physics_ball = physics_world->getPhysicsBall();
        if (physics_ball.lock())
        {
            simulator_ball = std::make_unique<PhysicsSimulatorBall>(physics_ball);
        }
        else
        {
            ADD_FAILURE()
                << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid"
                << std::endl;
        }
    }

    std::unique_ptr<PhysicsSimulatorBall> simulator_ball;

   private:
    std::unique_ptr<PhysicsWorld> physics_world;
    RobotConstants_t robot_constants = create2015RobotConstants();
    WheelConstants_t wheel_constants = create2015WheelConstants();
};

TEST_F(PhysicsSimulatorBallTest, test_get_position)
{
    EXPECT_LT((simulator_ball->position() - Point(1.01, -0.4)).length(), 0.001);
}

TEST_F(PhysicsSimulatorBallTest, test_get_linear_velocity)
{
    EXPECT_LT((simulator_ball->velocity() - Vector(0.02, -4.5)).length(), 0.001);
}
