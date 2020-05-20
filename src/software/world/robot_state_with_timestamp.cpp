#include "robot_state_with_timestamp.h"

RobotStateWithTimestamp::RobotStateWithTimestamp(const Point &position,
                                                 const Vector &velocity,
                                                 const Angle &orientation,
                                                 const AngularVelocity &angular_velocity,
                                                 const Timestamp &timestamp)
    : RobotState(position, velocity, orientation, angular_velocity), timestamp_(timestamp)
{
}

RobotStateWithTimestamp::RobotStateWithTimestamp(const RobotState &robot_state,
                                                 const Timestamp &timestamp)
    : RobotState(robot_state), timestamp_(timestamp)
{
}

Timestamp RobotStateWithTimestamp::timestamp() const
{
    return timestamp_;
}

RobotState RobotStateWithTimestamp::getRobotState() const
{
    return RobotState(position_, velocity_, orientation_, angular_velocity_);
}

bool RobotStateWithTimestamp::operator==(const RobotStateWithTimestamp &other) const
{
    return this->getRobotState() == other.getRobotState();
}

bool RobotStateWithTimestamp::operator!=(const RobotStateWithTimestamp &other) const
{
    return !(*this == other);
}
