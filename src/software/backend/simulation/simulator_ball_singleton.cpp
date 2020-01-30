#include "software/backend/simulation/simulator_ball_singleton.h"

std::shared_ptr<SimulatorBall> SimulatorBallSingleton::simulator_ball = nullptr;

// TODO: should have a null check on all functions in case this is not called first
void SimulatorBallSingleton::setSimulatorBall(std::shared_ptr<SimulatorBall> ball)
{
    simulator_ball = ball;
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
    return simulator_ball->getPositionX();
}

float SimulatorBallSingleton::getBallPositionY()
{
    return simulator_ball->getPositionY();
}

float SimulatorBallSingleton::getBallVelocityX()
{
    return simulator_ball->getVelocityX();
}

float SimulatorBallSingleton::getBallVelocityY()
{
    return simulator_ball->getVelocityY();
}
