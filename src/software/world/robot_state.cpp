#include "robot_state.h"

RobotState::RobotState(const Point &position, const Vector &velocity,
                       const Angle &orientation, const AngularVelocity &angular_velocity,
                       const Timestamp &timestamp)
{
    this->position_         = position;
    this->velocity_         = velocity;
    this->orientation_      = orientation;
    this->angular_velocity_ = angular_velocity;
    this->timestamp_        = timestamp;
}

Point RobotState::position() const
{
    return position_;
}

Vector RobotState::velocity() const
{
    return velocity_;
}

Angle RobotState::orientation() const
{
    return orientation_;
}

AngularVelocity RobotState::angularVelocity() const
{
    return angular_velocity_;
}

Timestamp RobotState::timestamp() const
{
    return timestamp_;
}

bool RobotState::operator==(const RobotState &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity() &&
           this->orientation() == other.orientation() &&
           this->angularVelocity() == other.angularVelocity();
}

bool RobotState::operator!=(const RobotState &other) const
{
    return !(*this == other);
}
