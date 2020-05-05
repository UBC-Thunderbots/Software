#include "software/simulation/simulator_ball_singleton.h"

#include <gtest/gtest.h>

#include <cmath>

#include "software/simulation/physics/physics_world.h"
#include "software/simulation/simulator_ball.h"
#include "software/test_util/test_util.h"
#include "software/world/ball.h"
#include "software/world/world.h"

extern "C"
{
#include "firmware/app/world/firmware_ball.h"
}

TEST(SimulatorBallSingletonTest, test_create_firmware_ball_with_single_simulator_ball)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() =
        Ball(Point(0.4, 0), Vector(-1.3, 2.01), Timestamp::fromSeconds(0));
    auto physics_world = std::make_unique<PhysicsWorld>(world);
    auto simulator_ball =
        std::make_shared<SimulatorBall>(physics_world->getPhysicsBall());

    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    auto firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    EXPECT_FLOAT_EQ(0.4, app_firmware_ball_getPositionX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(0.0, app_firmware_ball_getPositionY(firmware_ball.get()));
    EXPECT_FLOAT_EQ(-1.3, app_firmware_ball_getVelocityX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(2.01, app_firmware_ball_getVelocityY(firmware_ball.get()));
}

TEST(SimulatorBallSingletonTest, test_change_simulator_ball)
{
    World world_1 = ::Test::TestUtil::createBlankTestingWorld();
    world_1.mutableBall() =
        Ball(Point(0.4, 0), Vector(-1.3, 2.01), Timestamp::fromSeconds(0));
    auto physics_world_1 = std::make_unique<PhysicsWorld>(world_1);
    auto simulator_ball_1 =
        std::make_shared<SimulatorBall>(physics_world_1->getPhysicsBall());

    World world_2         = ::Test::TestUtil::createBlankTestingWorld();
    world_2.mutableBall() = Ball(Point(0, -3), Vector(0, 1), Timestamp::fromSeconds(0));
    auto physics_world_2  = std::make_unique<PhysicsWorld>(world_2);
    auto simulator_ball_2 =
        std::make_shared<SimulatorBall>(physics_world_2->getPhysicsBall());

    SimulatorBallSingleton::setSimulatorBall(simulator_ball_1);
    auto firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    EXPECT_FLOAT_EQ(0.4, app_firmware_ball_getPositionX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(0.0, app_firmware_ball_getPositionY(firmware_ball.get()));
    EXPECT_FLOAT_EQ(-1.3, app_firmware_ball_getVelocityX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(2.01, app_firmware_ball_getVelocityY(firmware_ball.get()));

    // The firmware functions should now return the data for simulator_ball_2, even though
    // we didn't need to create a new FirmwareBall_t
    SimulatorBallSingleton::setSimulatorBall(simulator_ball_2);
    firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    EXPECT_FLOAT_EQ(0, app_firmware_ball_getPositionX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(-3, app_firmware_ball_getPositionY(firmware_ball.get()));
    EXPECT_FLOAT_EQ(0, app_firmware_ball_getVelocityX(firmware_ball.get()));
    EXPECT_FLOAT_EQ(1, app_firmware_ball_getVelocityY(firmware_ball.get()));
}
