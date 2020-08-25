#include "software/simulation/simulator_ball_singleton.h"

#include "software/logger/logger.h"

std::shared_ptr<SimulatorBall> SimulatorBallSingleton::simulator_ball = nullptr;
FieldSide SimulatorBallSingleton::field_side_                         = FieldSide::NEG_X;

void SimulatorBallSingleton::setSimulatorBall(std::shared_ptr<SimulatorBall> ball,
                                              FieldSide field_side)
{
    simulator_ball = ball;
    field_side_    = field_side;
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

float SimulatorBallSingleton::checkValidAndReturnFloat(
    std::function<float(std::shared_ptr<SimulatorBall>)> func)
{
    if (simulator_ball)
    {
        return func(simulator_ball);
    }
    LOG(WARNING)
        << "SimulatorBallSingleton called without setting the SimulatorBall first"
        << std::endl;
    return 0.0f;
}

float SimulatorBallSingleton::invertValueToMatchFieldSide(double value)
{
    switch (field_side_)
    {
        case FieldSide::NEG_X:
            return static_cast<float>(value);
        case FieldSide::POS_X:
            return static_cast<float>(-value);
        default:
            throw std::invalid_argument("Unhandled value of FieldSide");
    }
}

float SimulatorBallSingleton::getBallPositionX()
{
    return checkValidAndReturnFloat(
        [](auto ball) { return invertValueToMatchFieldSide(ball->position().x()); });
}

float SimulatorBallSingleton::getBallPositionY()
{
    return checkValidAndReturnFloat(
        [](auto ball) { return invertValueToMatchFieldSide(ball->position().y()); });
}

float SimulatorBallSingleton::getBallVelocityX()
{
    return checkValidAndReturnFloat(
        [](auto ball) { return invertValueToMatchFieldSide(ball->velocity().x()); });
}

float SimulatorBallSingleton::getBallVelocityY()
{
    return checkValidAndReturnFloat(
        [](auto ball) { return invertValueToMatchFieldSide(ball->velocity().y()); });
}
