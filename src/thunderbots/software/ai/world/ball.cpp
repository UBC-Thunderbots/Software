#include "ball.h"

Ball::Ball(Point position, Vector velocity, std::chrono::time_point<std::chrono::steady_clock> last_update_timestamp) : position_(position), velocity_(velocity), last_update_timestamp(last_update_timestamp)
{
}

void Ball::update(const Ball &new_ball_data)
{
    update(new_ball_data.position(), new_ball_data.velocity(), new_ball_data.lastUpdateTimestamp());
}

void Ball::update(const Point &new_position, const Vector &new_velocity, std::chrono::time_point<std::chrono::steady_clock> timestamp)
{
    if (timestamp < last_update_timestamp) {
        // TODO: Error. We should never be updating with times from the past
        // TODO: Throw a proper exception here
        // https://github.com/UBC-Thunderbots/Software/issues/16
        exit(1);
    }

    position_ = new_position;
    velocity_ = new_velocity;
    last_update_timestamp = timestamp;
}

void Ball::updateState(std::chrono::time_point<std::chrono::steady_clock> timestamp) {
    if (timestamp < last_update_timestamp) {
        // TODO: Error. We should never be updating with times from the past
        // TODO: Throw a proper exception here
        // https://github.com/UBC-Thunderbots/Software/issues/16
        exit(1);
    }

    auto milliseconds_in_future = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - last_update_timestamp);
    Point new_position = estimatePositionAtFutureTime(milliseconds_in_future);
    Point new_velocity = estimateVelocityAtFutureTime(milliseconds_in_future);

    update(new_position, new_velocity, timestamp);
}

std::chrono::time_point<std::chrono::steady_clock> Ball::lastUpdateTimestamp() const {
    return last_update_timestamp;
}

Point Ball::position() const
{
    return position_;
}

Point Ball::estimatePositionAtFutureTime(const std::chrono::milliseconds& milliseconds_in_future) const
{
    if (milliseconds_in_future < std::chrono::milliseconds(0)) {
        // TODO: Error. We should never be updating with times from the past
        // TODO: Throw a proper exception here
        // https://github.com/UBC-Thunderbots/Software/issues/16
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/47
    typedef std::chrono::duration<double> double_seconds;
    double seconds_in_future = std::chrono::duration_cast<double_seconds>(milliseconds_in_future).count();
    return position_ + (velocity_.norm(seconds_in_future * velocity_.len()));
}

Vector Ball::velocity() const
{
    return velocity_;
}

Vector Ball::estimateVelocityAtFutureTime(const std::chrono::milliseconds& milliseconds_in_future) const
{
    if (milliseconds_in_future < std::chrono::milliseconds(0)) {
        // TODO: Error. We should never be updating with times from the past
        // TODO: Throw a proper exception here
        // https://github.com/UBC-Thunderbots/Software/issues/16
    }

    // TODO: This is a implementation with an empirically determined time constant that
    // does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/47
    typedef std::chrono::duration<double> double_seconds;
    double seconds_in_future = std::chrono::duration_cast<double_seconds>(milliseconds_in_future).count();
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
