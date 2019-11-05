#include "software/world/ball.h"

#include "shared/constants.h"
#include "software/world/ball_state.h"

Ball::Ball(Point position, Vector velocity, const Timestamp &timestamp,
           unsigned int history_size)
    : states_(history_size)
{
    if (history_size <= 0)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateCurrentState(position, velocity, timestamp);
}

Ball::Ball(BallState &ball_state, unsigned int history_size) : states_(history_size)
{
    if (history_size <= 0)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateCurrentState(ball_state);
}

BallState Ball::currentState() const
{
    return states_.front();
}

void Ball::updateCurrentState(const BallState &new_state)
{
    if (!states_.empty() && new_state.timestamp() < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }

    states_.push_front(new_state);
}

void Ball::updateCurrentState(const Point &new_position, const Vector &new_velocity,
                              const Timestamp &timestamp)
{
    return updateCurrentState(BallState(new_position, new_velocity, timestamp));
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
    Vector new_velocity     = estimateVelocityAtFutureTime(duration_in_future);

    updateCurrentState(BallState(new_position, new_velocity, timestamp));
}

Timestamp Ball::lastUpdateTimestamp() const
{
    return states_.front().timestamp();
}

std::vector<Timestamp> Ball::getPreviousTimestamps() const
{
    std::vector<Timestamp> timestamps{};
    for (const BallState &state : states_)
    {
        timestamps.push_back(state.timestamp());
    }

    return timestamps;
}

Point Ball::position() const
{
    return states_.front().position();
}

std::vector<Point> Ball::getPreviousPositions() const
{
    std::vector<Point> positions{};
    for (const BallState &state : states_)
    {
        positions.push_back(state.position());
    }

    return positions;
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
    return position() + (velocity().normalize(seconds_in_future * velocity().length()));
}

Vector Ball::velocity() const
{
    return states_.front().velocity();
}

std::vector<Vector> Ball::getPreviousVelocities() const
{
    std::vector<Vector> velocities{};
    for (const BallState &state : states_)
    {
        velocities.push_back(state.velocity());
    }

    return velocities;
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

boost::circular_buffer<BallState> Ball::getPreviousStates() const
{
    return states_;
}

std::optional<int> Ball::getHistoryIndexFromTimestamp(Timestamp &timestamp) const
{
    std::vector<Timestamp> timestamp_history = getPreviousTimestamps();
    for (unsigned i = 0; i < timestamp_history.size(); i++)
    {
        double timestamp_diff =
            fabs((timestamp - timestamp_history[i]).getMilliseconds());
        if (timestamp_diff < POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS)
            return i;  // If timestamp is close to desired timestamp, return the index.
    }
    return std::nullopt;
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
