#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/physics/physics_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

class SimulatorBallTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        b2Vec2 gravity(0, 0);
        world = std::make_shared<b2World>(gravity);

        Ball non_zero_state_ball_parameter(Point(1.01, -0.4), Vector(0.02, -4.5), Timestamp::fromSeconds(0));
        physics_ball = std::make_shared<PhysicsBall>(world, non_zero_state_ball_parameter, 1.0, 9.8);
        physics_ball_weak_ptr = std::weak_ptr<PhysicsBall>(physics_ball);
        simulator_ball = std::make_unique<SimulatorBall>(physics_ball_weak_ptr);
    }

    std::unique_ptr<SimulatorBall> simulator_ball;

    // Note: we declare the b2World before the physics objects so it is destroyed last.
    // If it is destroyed before the physics robots, segfaults will occur due to how Box2D
    // manages pointers internally
    std::shared_ptr<b2World> world;

private:
    std::shared_ptr<PhysicsBall> physics_ball;
    std::weak_ptr<PhysicsBall> physics_ball_weak_ptr;

};

TEST_F(SimulatorBallTest, test_get_position) {
    EXPECT_LT((simulator_ball->position() - Point(1.01, -0.4)).length(), 0.001);
}

TEST_F(SimulatorBallTest, test_get_linear_velocity) {
    EXPECT_LT((simulator_ball->velocity() - Vector(0.02, -4.5)).length(), 0.001);
}
