#include "software/world/robot.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/world/robot_state.h"

Robot::Robot(unsigned int id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp, unsigned int history_size,
             const std::set<RobotCapabilities::Capability> &capabilities)
    : id_(id),
      states_(history_size),
      capabilities_(capabilities)
{
    if (history_size <= 0)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateCurrentState(position, velocity, orientation, angular_velocity, timestamp);
}

void Robot::updateCurrentState(const RobotState &new_state) {
    if (!states_.empty() && new_state.timestamp() < lastUpdateTimestamp()) {
        throw std::invalid_argument(
                "Error: Trying to update ball state using a state older then the current state");
    }

    states_.push_front(new_state);
}

void Robot::updateCurrentState(const Point &new_position, const Vector &new_velocity,
                        const Angle &new_orientation,
                        const AngularVelocity &new_angular_velocity,
                        const Timestamp &timestamp)
{
    updateCurrentState(RobotState(new_position, new_velocity, new_orientation, new_angular_velocity, timestamp));
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

    updateCurrentState(new_position, new_velocity, new_orientation, new_angular_velocity,
                lastUpdateTimestamp() + duration_in_future);
}

Timestamp Robot::lastUpdateTimestamp() const
{
    return states_.front().timestamp();
}

unsigned int Robot::id() const
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
    return position() + velocity().norm(velocity().len() * seconds_in_future);
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

std::vector<Point> Robot::getPreviousPositions() const
{
    std::vector<Point> previousPositions{};
    for (const RobotState &state : states_)
    {
        previousPositions.push_back(state.position());
    }

    return previousPositions;
}

std::vector<Vector> Robot::getPreviousVelocities() const
{
    std::vector<Vector> previousVelocities{};
    for (const RobotState &state : states_)
    {
        previousVelocities.push_back(state.velocity());
    }

    return previousVelocities;
}

std::vector<Angle> Robot::getPreviousOrientations() const
{
    std::vector<Angle> previousOrientations{};
    for (const RobotState &state : states_)
    {
        previousOrientations.push_back(state.orientation());
    }

    return previousOrientations;
}

std::vector<AngularVelocity> Robot::getPreviousAngularVelocities() const
{
    std::vector<AngularVelocity> previousAngularVelocities{};
    for (const RobotState &state : states_)
    {
        previousAngularVelocities.push_back(state.angularVelocity());
    }

    return previousAngularVelocities;
}

std::vector<Timestamp> Robot::getPreviousTimestamps() const
{
    std::vector<Timestamp> previousTimestamps{};
    for (const RobotState &state : states_)
    {
        previousTimestamps.push_back(state.timestamp());
    }

    return previousTimestamps;
}

std::optional<int> Robot::getHistoryIndexFromTimestamp(Timestamp &timestamp) const
{
    std::vector<Timestamp> timestamp_history = getPreviousTimestamps();
    for (unsigned i = 0; i < timestamp_history.size(); i++)
    {
        double timestamp_diff =
            fabs((timestamp - timestamp_history[i]).getMilliseconds());

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

const std::set<RobotCapabilities::Capability> &Robot::getRobotCapabilities() const
{
    return capabilities_;
}

std::set<RobotCapabilities::Capability> &Robot::getMutableRobotCapabilities()
{
    return capabilities_;
}
