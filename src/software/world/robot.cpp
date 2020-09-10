#include "software/world/robot.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/timestamped_robot_state.h"

Robot::Robot(RobotId id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id),
      current_state(TimestampedRobotState(position, velocity, orientation,
                                          angular_velocity, timestamp)),
      unavailable_capabilities_(unavailable_capabilities)
{
}

Robot::Robot(RobotId id, const TimestampedRobotState &initial_state,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id),
      current_state(initial_state),
      unavailable_capabilities_(unavailable_capabilities)
{
}

void Robot::updateState(const TimestampedRobotState &new_state)
{
    current_state = new_state;
}

TimestampedRobotState Robot::currentState() const
{
    return current_state;
}

Timestamp Robot::lastUpdateTimestamp() const
{
    return current_state.timestamp();
}

RobotId Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return current_state.state().position();
}

Vector Robot::velocity() const
{
    return current_state.state().velocity();
}

Angle Robot::orientation() const
{
    return current_state.state().orientation();
}

AngularVelocity Robot::angularVelocity() const
{
    return current_state.state().angularVelocity();
}

bool Robot::isNearDribbler(const Point &test_point) const
{
    Vector vector_to_test_point                     = test_point - position();
    static const double POSSESSION_THRESHOLD_METERS = ROBOT_MAX_RADIUS_METERS + 0.2;
    if (vector_to_test_point.length() > POSSESSION_THRESHOLD_METERS)
    {
        return false;
    }
    else
    {
        // check that ball is in a 90-degree cone in front of the robot
        auto ball_to_robot_angle =
            orientation().minDiff(vector_to_test_point.orientation());
        return (ball_to_robot_angle < Angle::fromDegrees(45.0));
    }
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
