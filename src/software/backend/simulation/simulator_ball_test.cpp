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
        physics_ball_non_zero_state = std::make_shared<PhysicsBall>(world, non_zero_state_ball_parameter, 1.0);
        physics_ball_non_zero_state_weak_ptr = std::weak_ptr<PhysicsBall>(physics_ball_non_zero_state);
        simulator_ball_non_zero_state = SimulatorBall(physics_ball_non_zero_state_weak_ptr);

        Ball zero_state_ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
        physics_ball_zero_state = std::make_shared<PhysicsBall>(world, zero_state_ball_parameter, 1.0);
        physics_ball_zero_state_weak_ptr = std::weak_ptr<PhysicsBall>(physics_ball_zero_state);
        simulator_ball_zero_state = SimulatorBall(physics_ball_zero_state_weak_ptr);
    }

    SimulatorBall simulator_ball_non_zero_state;
    SimulatorBall simulator_ball_zero_state;
    // Note: we declare the b2World before the physics objects so it is destroyed last.
    // If it is destroyed before the physics robots, segfaults will occur due to how Box2D
    // manages pointers internally
    std::shared_ptr<b2World> world;

private:
    std::shared_ptr<PhysicsBall> physics_ball_non_zero_state;
    std::weak_ptr<PhysicsBall> physics_ball_non_zero_state_weak_ptr;
    std::shared_ptr<PhysicsBall> physics_ball_zero_state;
    std::weak_ptr<PhysicsBall> physics_ball_zero_state_weak_ptr;
};

TEST_F(SimulatorBallTest, test_get_position) {
    EXPECT_FLOAT_EQ(simulator_ball_non_zero_state.getPositionX(), 1.01);
    EXPECT_FLOAT_EQ(simulator_ball_non_zero_state.getPositionY(), -0.4);
}

TEST_F(SimulatorBallTest, test_get_linear_velocity) {
    EXPECT_NEAR(simulator_ball_non_zero_state.getVelocityX(), 0.02, 0.01);
    EXPECT_NEAR(simulator_ball_non_zero_state.getVelocityY(), -4.5, 0.01);
}

TEST_F(SimulatorBallTest, test_apply_force_to_ball) {
    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for(unsigned int i = 0; i < 60; i++) {
        simulator_ball_zero_state.applyForce(Vector(1, 2));
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    Vector velocity(simulator_ball_zero_state.getVelocityX(), simulator_ball_zero_state.getVelocityY());
    EXPECT_LT((velocity - Vector(1, 2)).length(), 0.05);
}

TEST_F(SimulatorBallTest, test_apply_impulse_to_ball) {
    simulator_ball_zero_state.applyImpulse(Vector(1, 2));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);
    Vector velocity(simulator_ball_zero_state.getVelocityX(), simulator_ball_zero_state.getVelocityY());
    EXPECT_LT((velocity - Vector(1, 2)).length(), 0.05);
}
