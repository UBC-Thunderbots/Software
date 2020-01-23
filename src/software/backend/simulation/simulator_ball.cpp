#include "software/backend/simulation/simulator_ball.h"

std::weak_ptr<PhysicsBall> SimulatorBallSingleton::physics_ball_weak_ptr =
    std::weak_ptr<PhysicsBall>();

void SimulatorBallSingleton::setPhysicsBall(std::weak_ptr<PhysicsBall> ball)
{
    physics_ball_weak_ptr = ball;
}

std::unique_ptr<FirmwareBall_t, FirmwareBallDeleter>
SimulatorBallSingleton::createFirmwareBall()
{
    // TODO: Make sure all objects de-allocated properly
    // See issue https://github.com/UBC-Thunderbots/Software/issues/1128
    FirmwareBall_t* firmware_ball =
        app_firmware_ball_create(&(SimulatorBallSingleton::getBallPositionX),
                                 &(SimulatorBallSingleton::getBallPositionY),
                                 &(SimulatorBallSingleton::getBallVelocityX),
                                 &(SimulatorBallSingleton::getBallVelocityY));

    return std::unique_ptr<FirmwareBall_t, FirmwareBallDeleter>(firmware_ball,
                                                                FirmwareBallDeleter());
}

float SimulatorBallSingleton::getBallPositionX()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
    return 0.0;
}

float SimulatorBallSingleton::getBallPositionY()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
    return 0.0;
}

float SimulatorBallSingleton::getBallVelocityX()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
    return 0.0;
}

float SimulatorBallSingleton::getBallVelocityY()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
    return 0.0;
}
