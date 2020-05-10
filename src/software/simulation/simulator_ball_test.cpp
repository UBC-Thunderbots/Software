#include "software/simulation/simulator_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_world.h"
#include "software/test_util/test_util.h"

class SimulatorBallTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        World world = ::Test::TestUtil::createBlankTestingWorld();
        Ball non_zero_state_ball_parameter(Point(1.01, -0.4), Vector(0.02, -4.5),
                                           Timestamp::fromSeconds(0));
        world.mutableBall() = non_zero_state_ball_parameter;

        physics_world     = std::make_unique<PhysicsWorld>(world);
        auto physics_ball = physics_world->getPhysicsBall();
        if (physics_ball.lock())
        {
            simulator_ball = std::make_unique<SimulatorBall>(physics_ball);
        }
        else
        {
            ADD_FAILURE()
                << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid"
                << std::endl;
        }
    }

    std::unique_ptr<SimulatorBall> simulator_ball;

   private:
    std::unique_ptr<PhysicsWorld> physics_world;
};

TEST_F(SimulatorBallTest, test_get_position)
{
    EXPECT_LT((simulator_ball->position() - Point(1.01, -0.4)).length(), 0.001);
}

TEST_F(SimulatorBallTest, test_get_linear_velocity)
{
    EXPECT_LT((simulator_ball->velocity() - Vector(0.02, -4.5)).length(), 0.001);
}
