#include "software/backend/simulation/simulator_ball.h"

std::weak_ptr<PhysicsBall> SimulatorBall::physics_ball_weak_ptr =
    std::weak_ptr<PhysicsBall>();

void SimulatorBall::setPhysicsBall(std::weak_ptr<PhysicsBall> ball)
{
    physics_ball_weak_ptr = ball;
}

FirmwareBall_t* SimulatorBall::createFirmwareBall()
{
    FirmwareBall_t* firmware_ball = app_firmware_ball_create(
        &(SimulatorBall::getBallPositionX), &(SimulatorBall::getBallPositionY),
        &(SimulatorBall::getBallVelocityX), &(SimulatorBall::getBallVelocityY));

    return firmware_ball;
}

float SimulatorBall::getBallPositionX()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
}

float SimulatorBall::getBallPositionY()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
}

float SimulatorBall::getBallVelocityX()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
}

float SimulatorBall::getBallVelocityY()
{
    if (auto physics_ball = physics_ball_weak_ptr.lock())
    {
        // TODO: implement me
    }
}
