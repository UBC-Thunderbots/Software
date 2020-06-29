#include "software/world/robot.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/timestamped_robot_state.h"

Robot::Robot(RobotId id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp, unsigned int history_size,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id), states_(history_size), unavailable_capabilities_(unavailable_capabilities)
{
    if (history_size < 1)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateState(TimestampedRobotState(position, velocity, orientation, angular_velocity,
                                      timestamp));
}

Robot::Robot(RobotId id, const TimestampedRobotState &initial_state,
             unsigned int history_size,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id), states_(history_size), unavailable_capabilities_(unavailable_capabilities)
{
    if (history_size < 1)
    {
        throw std::invalid_argument("Error: history_size must be greater than 0");
    }

    updateState(initial_state);
}

void Robot::updateState(const TimestampedRobotState &new_state)
{
    if (!states_.empty() && new_state.timestamp() < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update robot state using a state older then the current state");
    }

    states_.push_front(new_state);
}

TimestampedRobotState Robot::currentState() const
{
    return states_.front();
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
    return states_.front().state().position();
}

Vector Robot::velocity() const
{
    return states_.front().state().velocity();
}

Angle Robot::orientation() const
{
    return states_.front().state().orientation();
}

AngularVelocity Robot::angularVelocity() const
{
    return states_.front().state().angularVelocity();
}

RobotHistory Robot::getPreviousStates() const
{
    return states_;
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

const std::set<RobotCapability> &Robot::getCapabilitiesBlacklist() const
{
    return unavailable_capabilities_;
}

std::set<RobotCapability> Robot::getCapabilitiesWhitelist() const
{
    // robot capabilities = all possible capabilities - unavailable capabilities

    std::set<RobotCapability> all_capabilities = allRobotCapabilities();
    std::set<RobotCapability> robot_capabilities;
    std::set_difference(all_capabilities.begin(), all_capabilities.end(),
                        getCapabilitiesBlacklist().begin(),
                        getCapabilitiesBlacklist().end(),
                        std::inserter(robot_capabilities, robot_capabilities.begin()));

    return robot_capabilities;
}

std::set<RobotCapability> &Robot::getMutableRobotCapabilities()
{
    return unavailable_capabilities_;
}
