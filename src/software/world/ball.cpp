#include "software/world/ball.h"

#include "shared/constants.h"
#include "software/world/ball_model/two_stage_linear_ball_model.h"

Ball::Ball(const Point &position, const Vector &velocity, const Timestamp &timestamp)
    : Ball(BallState(position, velocity), timestamp)
{
}

Ball::Ball(const BallState &initial_state, const Timestamp &timestamp)
    : current_state_(initial_state),
      timestamp_(timestamp),
      ball_model_(std::make_shared<TwoStageLinearBallModel>(
          TwoStageLinearBallModel(initial_state)))
{
}

BallState Ball::currentState() const
{
    return current_state_;
}

const std::shared_ptr<BallModel> &Ball::ballModel() const
{
    return ball_model_;
}

void Ball::updateState(const BallState &new_state, const Timestamp &new_timestamp)
{
    if (new_timestamp < timestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }
    ball_model_ =
        std::make_shared<TwoStageLinearBallModel>(TwoStageLinearBallModel(new_state));
    current_state_ = new_state;
    timestamp_     = new_timestamp;
}

Timestamp Ball::timestamp() const
{
    return timestamp_;
}

Point Ball::position() const
{
    return current_state_.position();
}

Vector Ball::velocity() const
{
    return current_state_.velocity();
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
