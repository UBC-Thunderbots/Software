#include "software/world/robot.h"

#include "shared/constants.h"
#include "software/logger/logger.h"

Robot::Robot(RobotId id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id),
      current_state_(position, velocity, orientation, angular_velocity),
      timestamp_(timestamp),
      unavailable_capabilities_(unavailable_capabilities)
{
}

Robot::Robot(RobotId id, const RobotState &initial_state, const Timestamp &timestamp,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id),
      current_state_(initial_state),
      timestamp_(timestamp),
      unavailable_capabilities_(unavailable_capabilities)
{
}

void Robot::updateState(const RobotState &state, const Timestamp &timestamp)
{
    current_state_ = state;
    timestamp_     = timestamp;
}

RobotState Robot::currentState() const
{
    return current_state_;
}

Timestamp Robot::timestamp() const
{
    return timestamp_;
}

RobotId Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return current_state_.position();
}

Vector Robot::velocity() const
{
    return current_state_.velocity();
}

Angle Robot::orientation() const
{
    return current_state_.orientation();
}

AngularVelocity Robot::angularVelocity() const
{
    return current_state_.angularVelocity();
}

bool Robot::isNearDribbler(const Point &test_point) const
{
    static const double POSSESSION_THRESHOLD_METERS = ROBOT_MAX_RADIUS_METERS;

    Vector vector_to_test_point = test_point - position();
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

const std::set<RobotCapability> &Robot::getUnavailableCapabilities() const
{
    return unavailable_capabilities_;
}

std::set<RobotCapability> Robot::getAvailableCapabilities() const
{
    // robot capabilities = all possible capabilities - unavailable capabilities

    std::set<RobotCapability> all_capabilities = allRobotCapabilities();
    std::set<RobotCapability> robot_capabilities;
    std::set_difference(all_capabilities.begin(), all_capabilities.end(),
                        getUnavailableCapabilities().begin(),
                        getUnavailableCapabilities().end(),
                        std::inserter(robot_capabilities, robot_capabilities.begin()));

    return robot_capabilities;
}

std::set<RobotCapability> &Robot::getMutableRobotCapabilities()
{
    return unavailable_capabilities_;
}
