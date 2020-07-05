#include "software/world/ball.h"

#include "shared/constants.h"

Ball::Ball(const Point &position, const Vector &velocity, const Timestamp &timestamp,
           unsigned int history_size)
    : states_(history_size)
{
    if (history_size <= 0)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateState(TimestampedBallState(position, velocity, timestamp));
}

Ball::Ball(const TimestampedBallState &initial_state, unsigned int history_size)
    : states_(history_size)
{
    if (history_size <= 0)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateState(initial_state);
}

TimestampedBallState Ball::currentState() const
{
    return states_.front();
}

void Ball::updateState(const TimestampedBallState &new_state)
{
    if (!states_.empty() && new_state.timestamp() < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }

    states_.push_front(new_state);
}

Timestamp Ball::lastUpdateTimestamp() const
{
    return states_.front().timestamp();
}

Point Ball::position() const
{
    return states_.front().state().position();
}

Vector Ball::velocity() const
{
    return states_.front().state().velocity();
}

BallHistory Ball::getPreviousStates() const
{
    return states_;
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
