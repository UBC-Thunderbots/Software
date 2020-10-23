#include "software/world/ball.h"

#include "shared/constants.h"
#include "software/physics/physics.h"

Ball::Ball(const Point &position, const Vector &velocity, const Timestamp &timestamp,
           const Vector &acceleration)
    : Ball(BallState(position, velocity), timestamp, acceleration)
{
}

Ball::Ball(const BallState &initial_state, const Timestamp &timestamp,
           const Vector &acceleration)
    : current_state_(initial_state), timestamp_(timestamp), acceleration_(acceleration)
{
}

BallState Ball::currentState() const
{
    return current_state_;
}

void Ball::updateState(const BallState &new_state, const Timestamp &new_timestamp,
                       const Vector &new_acceleration)
{
    if (new_timestamp < timestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }
    current_state_ = new_state;
    timestamp_     = new_timestamp;
    acceleration_  = new_acceleration;
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

Vector Ball::acceleration() const
{
    return acceleration_;
}

BallState Ball::estimateFutureState(const Duration &duration_in_future) const
{
    const Point future_position =
        calculateFuturePosition(current_state_.position(), current_state_.velocity(),
                                acceleration_, duration_in_future);
    const Vector future_velocity = calculateFutureVelocity(
        current_state_.velocity(), acceleration_, duration_in_future);

    return BallState(future_position, future_velocity);
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
