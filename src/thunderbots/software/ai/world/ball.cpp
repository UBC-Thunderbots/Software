#include "ball.h"

Ball::Ball(Point position, Vector velocity, const Timestamp &timestamp,
           unsigned int history_duration)
    : positions_(history_duration),
      velocities_(history_duration),
      last_update_timestamps(history_duration)
{
    addStateToBallHistory(position, velocity, timestamp);
}

void Ball::updateState(const Ball &new_ball_data)
{
    updateState(new_ball_data.position(), new_ball_data.velocity(),
                new_ball_data.lastUpdateTimestamp());
}

void Ball::updateState(const Point &new_position, const Vector &new_velocity,
                       const Timestamp &timestamp)
{
    if (timestamp < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: State of ball is updating times from the past");
    }

    addStateToBallHistory(new_position, new_velocity, timestamp);
}

void Ball::updateStateToPredictedState(const Timestamp &timestamp)
{
    if (timestamp < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: Predicted state is updating times from the past");
    }

    auto duration_in_future = timestamp - lastUpdateTimestamp();
    Point new_position      = estimatePositionAtFutureTime(duration_in_future);
    Point new_velocity      = estimateVelocityAtFutureTime(duration_in_future);

    updateState(new_position, new_velocity, timestamp);
}

Timestamp Ball::lastUpdateTimestamp() const
{
    return last_update_timestamps.front();
}

Point Ball::position() const
{
    return positions_.front();
}

Point Ball::estimatePositionAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Position estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/47
    double seconds_in_future = duration_in_future.getSeconds();
    return position() + (velocity().norm(seconds_in_future * velocity().len()));
}

Vector Ball::velocity() const
{
    return velocities_.front();
}

Vector Ball::estimateVelocityAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Velocity estimate is updating times from the past");
    }

    // TODO: This is a implementation with an empirically determined time constant that
    // does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/47
    double seconds_in_future = duration_in_future.getSeconds();
    return velocity() * exp(-0.1 * seconds_in_future);
}

std::vector<Point> Ball::getPreviousPositions()
{
    std::vector<Point> retval{};
    for (Point p : positions_)
        retval.push_back(p);

    return retval;
}

std::vector<Vector> Ball::getPreviousVelocities()
{
    std::vector<Vector> retval{};
    for (Vector v : velocities_)
        retval.push_back(v);

    return retval;
}

std::vector<Timestamp> Ball::getPreviousTimestamps()
{
    std::vector<Timestamp> retval{};
    for (Timestamp t : last_update_timestamps)
        retval.push_back(t);

    return retval;
}

void Ball::addStateToBallHistory(const Point &position, const Vector &velocity,
                                 const Timestamp &timestamp)
{
    positions_.push_front(position);
    velocities_.push_front(velocity);
    last_update_timestamps.push_front(timestamp);
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
