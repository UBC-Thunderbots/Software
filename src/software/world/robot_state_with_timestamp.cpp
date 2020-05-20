#include "robot_state_with_timestamp.h"

RobotStateWithTimestamp::RobotStateWithTimestamp(const Point &position, const Vector &velocity,
                                                 const Angle &orientation, const AngularVelocity &angular_velocity,
                                                 const Timestamp &timestamp)
{
    this->position_         = position;
    this->velocity_         = velocity;
    this->orientation_      = orientation;
    this->angular_velocity_ = angular_velocity;
    this->timestamp_        = timestamp;
}

Point RobotStateWithTimestamp::position() const
{
    return position_;
}

Vector RobotStateWithTimestamp::velocity() const
{
    return velocity_;
}

Angle RobotStateWithTimestamp::orientation() const
{
    return orientation_;
}

AngularVelocity RobotStateWithTimestamp::angularVelocity() const
{
    return angular_velocity_;
}

Timestamp RobotStateWithTimestamp::timestamp() const
{
    return timestamp_;
}

bool RobotStateWithTimestamp::operator==(const RobotStateWithTimestamp &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity() &&
           this->orientation() == other.orientation() &&
           this->angularVelocity() == other.angularVelocity();
}

bool RobotStateWithTimestamp::operator!=(const RobotStateWithTimestamp &other) const
{
    return !(*this == other);
}
