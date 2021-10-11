#include "software/simulation/er_force_simulator_ball.h"

ErForceSimulatorBall::ErForceSimulatorBall(BallState ball_state) : ball_state_(ball_state)
{
}

Point ErForceSimulatorBall::position() const
{
    return ball_state_.position();
}

Vector ErForceSimulatorBall::velocity() const
{
    return ball_state_.velocity();
}

void ErForceSimulatorBall::setState(BallState ball_state)
{
    ball_state_ = ball_state;
}
