#include "software/backend/simulation/simulator_ball_singleton.h"

#include "software/logger/init.h"

std::shared_ptr<SimulatorBall> SimulatorBallSingleton::simulator_ball = nullptr;

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
    if (simulator_ball)
    {
        return simulator_ball->position().x();
    }
    LOG(WARNING)
        << "SimulatorBallSingleton called without setting the SimulatorBall first"
        << std::endl;
    return 0.0f;
}

float SimulatorBallSingleton::getBallPositionY()
{
    if (simulator_ball)
    {
        return simulator_ball->position().y();
    }
    LOG(WARNING)
        << "SimulatorBallSingleton called without setting the SimulatorBall first"
        << std::endl;
    return 0.0f;
}

float SimulatorBallSingleton::getBallVelocityX()
{
    if (simulator_ball)
    {
        return simulator_ball->velocity().x();
    }
    LOG(WARNING)
        << "SimulatorBallSingleton called without setting the SimulatorBall first"
        << std::endl;
    return 0.0f;
}

float SimulatorBallSingleton::getBallVelocityY()
{
    if (simulator_ball)
    {
        return simulator_ball->velocity().y();
    }
    LOG(WARNING)
        << "SimulatorBallSingleton called without setting the SimulatorBall first"
        << std::endl;
    return 0.0f;
}
