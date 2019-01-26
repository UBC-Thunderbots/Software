#include "ball.h"

Ball::Ball(Point position, Vector velocity,
           std::chrono::steady_clock::time_point timestamp)
    : position_(position), velocity_(velocity), last_update_timestamp(timestamp)
{
}

void Ball::updateState(const Ball &new_ball_data)
{
    updateState(new_ball_data.position(), new_ball_data.velocity(),
                new_ball_data.lastUpdateTimestamp());
}

void Ball::updateState(const Point &new_position, const Vector &new_velocity,
                       std::chrono::steady_clock::time_point timestamp)
{
    if (timestamp < last_update_timestamp)
    {
        throw std::invalid_argument(
            "Error: State of ball is updating times from the past");
    }

    position_             = new_position;
    velocity_             = new_velocity;
    last_update_timestamp = timestamp;
}

void Ball::updateStateToPredictedState(std::chrono::steady_clock::time_point timestamp)
{
    if (timestamp < last_update_timestamp)
    {
        throw std::invalid_argument(
            "Error: Predicted state is updating times from the past");
    }

    auto milliseconds_in_future = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp - last_update_timestamp);
    Point new_position = estimatePositionAtFutureTime(milliseconds_in_future);
    Point new_velocity = estimateVelocityAtFutureTime(milliseconds_in_future);

    updateState(new_position, new_velocity, timestamp);
}

std::chrono::steady_clock::time_point Ball::lastUpdateTimestamp() const
{
    return last_update_timestamp;
}

Point Ball::position() const
{
    return position_;
}

Point Ball::estimatePositionAtFutureTime(
    const std::chrono::milliseconds &milliseconds_in_future) const
{
    if (milliseconds_in_future < std::chrono::milliseconds(0))
    {
        throw std::invalid_argument(
            "Error: Position estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/47
    typedef std::chrono::duration<double> double_seconds;
    double seconds_in_future =
        std::chrono::duration_cast<double_seconds>(milliseconds_in_future).count();
    return position_ + (velocity_.norm(seconds_in_future * velocity_.len()));
}

Vector Ball::velocity() const
{
    return velocity_;
}

Vector Ball::estimateVelocityAtFutureTime(
    const std::chrono::milliseconds &milliseconds_in_future) const
{
    if (milliseconds_in_future < std::chrono::milliseconds(0))
    {
        throw std::invalid_argument(
            "Error: Velocity estimate is updating times from the past");
    }

    // TODO: This is a implementation with an empirically determined time constant that
    // does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/47
    typedef std::chrono::duration<double> double_seconds;
    double seconds_in_future =
        std::chrono::duration_cast<double_seconds>(milliseconds_in_future).count();
    return velocity_ * exp(-0.1 * seconds_in_future);
}

bool Ball::operator==(const Ball &other) const
{
    return this->position_ == other.position_ && this->velocity_ == other.velocity_;
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
