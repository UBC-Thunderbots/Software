#include "software/world/robot.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/world/robot_state.h"

Robot::Robot(RobotId id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp, unsigned int history_size,
             const std::set<RobotCapabilities::Capability> &capabilities)
    : id_(id), states_(history_size), capabilities_(capabilities)
{
    if (history_size < 1)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateState(RobotState(position, velocity, orientation, angular_velocity, timestamp));
}

void Robot::updateState(const RobotState &new_state)
{
    if (!states_.empty() && new_state.timestamp() < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update robot state using a state older then the current state");
    }

    states_.push_front(new_state);
}

RobotState Robot::currentState() const
{
    return states_.front();
}

void Robot::updateStateToPredictedState(const Timestamp &timestamp)
{
    updateStateToPredictedState(timestamp - lastUpdateTimestamp());
}

void Robot::updateStateToPredictedState(const Duration &duration_in_future)
{
    if (duration_in_future.getSeconds() < 0)
    {
        throw std::invalid_argument(
            "Error: Predicted state is updating times from the past");
    }
    Point new_position    = estimatePositionAtFutureTime(duration_in_future);
    Vector new_velocity   = estimateVelocityAtFutureTime(duration_in_future);
    Angle new_orientation = estimateOrientationAtFutureTime(duration_in_future);
    AngularVelocity new_angular_velocity =
        estimateAngularVelocityAtFutureTime(duration_in_future);

    updateState(RobotState(new_position, new_velocity, new_orientation,
                           new_angular_velocity,
                           lastUpdateTimestamp() + duration_in_future));
}

Timestamp Robot::lastUpdateTimestamp() const
{
    return states_.front().timestamp();
}

RobotId Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return states_.front().position();
}

Point Robot::estimatePositionAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Position estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    double seconds_in_future = duration_in_future.getSeconds();
    return position() + velocity().normalize(velocity().length() * seconds_in_future);
}

Vector Robot::velocity() const
{
    return states_.front().velocity();
}

Vector Robot::estimateVelocityAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Velocity estimate is updating times from the past");
    }

    // TODO: This simple implementation that assumes the robot maintains the same velocity
    // and does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/50
    return velocity();
}

Angle Robot::orientation() const
{
    return states_.front().orientation();
}

Angle Robot::estimateOrientationAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Orientation estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Orientation prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    double seconds_in_future = duration_in_future.getSeconds();
    return orientation() + angularVelocity() * seconds_in_future;
}

AngularVelocity Robot::angularVelocity() const
{
    return states_.front().angularVelocity();
}

AngularVelocity Robot::estimateAngularVelocityAtFutureTime(
    const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Angular velocity estimate is updating times from the past");
    }

    // TODO: This simple implementation that assumes the robot maintains the same
    // angular velocity and does not necessarily reflect real-world behavior. Angular
    // velocity prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    return angularVelocity();
}

boost::circular_buffer<RobotState> Robot::getPreviousStates() const
{
    return states_;
}

std::optional<int> Robot::getHistoryIndexFromTimestamp(Timestamp &timestamp) const
{
    for (unsigned i = 0; i < states_.size(); i++)
    {
        double timestamp_diff =
            fabs((timestamp - states_.at(i).timestamp()).getMilliseconds());

        // If timestamp is close to desired timestamp, return the index.
        if (timestamp_diff < POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS)
            return i;
    }
    return std::nullopt;
}

bool Robot::operator==(const Robot &other) const
{
    return this->id_ == other.id_ && this->position() == other.position() &&
           this->velocity() == other.velocity() &&
           this->orientation() == other.orientation() &&
           this->angularVelocity() == other.angularVelocity();
}

bool Robot::operator!=(const Robot &other) const
{
    return !(*this == other);
}

const std::set<RobotCapabilities::Capability> &Robot::getCapabiltiesBlacklist() const
{
    return unavailable_capabilities_;
}

std::set<RobotCapabilities::Capability> &Robot::getMutableRobotCapabilities()
{
    return unavailable_capabilities_;
}
