#include "software/backend/simulation/simulator_ball.h"

void SimulatorBall::setPhysicsBall(std::shared_ptr<PhysicsBall> ball)
{
    physics_ball = ball;
    checkPhysicsBallValid();
}

FirmwareBall_t* SimulatorBall::createFirmwareBall()
{
    checkPhysicsBallValid();

    FirmwareBall_t* firmware_ball = app_firmware_ball_create(
        &(SimulatorBall::getBallPositionX), &(SimulatorBall::getBallPositionY),
        &(SimulatorBall::getBallVelocityX), &(SimulatorBall::getBallVelocityY));

    return firmware_ball;
}

float SimulatorBall::getBallPositionX() {}

float SimulatorBall::getBallPositionY() {}

float SimulatorBall::getBallVelocityX() {}

float SimulatorBall::getBallVelocityY() {}

void SimulatorBall::checkPhysicsBallValid() {
    if (!physics_ball)
    {
        throw std::invalid_argument("PhysicsBall is a nullptr");
    }
}
