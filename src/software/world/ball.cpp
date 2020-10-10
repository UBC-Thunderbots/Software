#include "software/world/ball.h"

#include "shared/constants.h"
#include "software/world/ball_model/two_stage_linear_ball_model.h"

Ball::Ball(const Point &position, const Vector &velocity, const Timestamp &timestamp)
    : Ball(TimestampedBallState(position, velocity, timestamp))
{
}

Ball::Ball(const TimestampedBallState &initial_state)
    : current_state_(initial_state),
      ball_model_(std::make_shared<TwoStageLinearBallModel>(
          TwoStageLinearBallModel(initial_state.state())))
{
}

TimestampedBallState Ball::currentState() const
{
    return current_state_;
}

const std::shared_ptr<BallModel> &Ball::ballModel() const
{
    return ball_model_;
}

void Ball::updateState(const TimestampedBallState &new_state)
{
    if (new_state.timestamp() < timestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }
    ball_model_ = std::make_shared<TwoStageLinearBallModel>(
        TwoStageLinearBallModel(new_state.state()));
    current_state_ = new_state;
}

Timestamp Ball::timestamp() const
{
    return current_state_.timestamp();
}

Point Ball::position() const
{
    return current_state_.state().position();
}

Vector Ball::velocity() const
{
    return current_state_.state().velocity();
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
