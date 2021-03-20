#include "software/simulation/simulator_ball_singleton.h"

#include <gtest/gtest.h>

#include <cmath>

#include "software/simulation/physics/physics_world.h"
#include "software/simulation/simulator_ball.h"
#include "software/world/ball.h"
#include "software/world/world.h"

extern "C"
{
#include "firmware/app/world/firmware_ball.h"
}

TEST(SimulatorBallSingletonTest, test_create_firmware_ball_with_single_simulator_ball)
{
    auto physics_world = std::make_unique<PhysicsWorld>(
        Field::createSSLDivisionBField(), std::make_shared<const SimulatorConfig>());
    physics_world->setBallState(BallState(Point(0.4, 0), Vector(-1.3, 2.01)));
    auto simulator_ball =
        std::make_shared<SimulatorBall>(physics_world->getPhysicsBall());

    SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
    auto firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    EXPECT_FLOAT_EQ(0.4f, app_firmware_ball_getPositionX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(0.0f, app_firmware_ball_getPositionY(firmware_ball.get()));
    EXPECT_FLOAT_EQ(-1.3f, app_firmware_ball_getVelocityX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(2.01f, app_firmware_ball_getVelocityY(firmware_ball.get()));
}

TEST(SimulatorBallSingletonTest, test_change_simulator_ball)
{
    auto physics_world_1 = std::make_unique<PhysicsWorld>(
        Field::createSSLDivisionBField(), std::make_shared<const SimulatorConfig>());
    physics_world_1->setBallState(BallState(Point(0.4, 0), Vector(-1.3, 2.01)));
    auto simulator_ball_1 =
        std::make_shared<SimulatorBall>(physics_world_1->getPhysicsBall());

    auto physics_world_2 = std::make_unique<PhysicsWorld>(
        Field::createSSLDivisionBField(), std::make_shared<const SimulatorConfig>());
    physics_world_2->setBallState(BallState(Point(0, -3), Vector(0, 1)));

    auto simulator_ball_2 =
        std::make_shared<SimulatorBall>(physics_world_2->getPhysicsBall());

    SimulatorBallSingleton::setSimulatorBall(simulator_ball_1, FieldSide::NEG_X);
    auto firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    EXPECT_FLOAT_EQ(0.4f, app_firmware_ball_getPositionX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(0.0f, app_firmware_ball_getPositionY(firmware_ball.get()));
    EXPECT_FLOAT_EQ(-1.3f, app_firmware_ball_getVelocityX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(2.01f, app_firmware_ball_getVelocityY(firmware_ball.get()));

    // The firmware functions should now return the data for simulator_ball_2, even though
    // we didn't need to create a new FirmwareBall_t
    SimulatorBallSingleton::setSimulatorBall(simulator_ball_2, FieldSide::NEG_X);
    firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    EXPECT_FLOAT_EQ(0.0f, app_firmware_ball_getPositionX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(-3.0f, app_firmware_ball_getPositionY(firmware_ball.get()));
    EXPECT_FLOAT_EQ(0.0f, app_firmware_ball_getVelocityX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(1.0f, app_firmware_ball_getVelocityY(firmware_ball.get()));
}
