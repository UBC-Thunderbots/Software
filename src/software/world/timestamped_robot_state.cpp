#include "timestamped_robot_state.h"

TimestampedRobotState::TimestampedRobotState(const Point &position,
                                             const Vector &velocity,
                                             const Angle &orientation,
                                             const AngularVelocity &angular_velocity,
                                             const Timestamp &timestamp)
    : RobotState(position, velocity, orientation, angular_velocity), timestamp_(timestamp)
{
}

TimestampedRobotState::TimestampedRobotState(const RobotState &robot_state,
                                             const Timestamp &timestamp)
    : RobotState(robot_state), timestamp_(timestamp)
{
}

Timestamp TimestampedRobotState::timestamp() const
{
    return timestamp_;
}

RobotState TimestampedRobotState::getRobotState() const
{
    return RobotState(position_, velocity_, orientation_, angular_velocity_);
}

bool TimestampedRobotState::operator==(const TimestampedRobotState &other) const
{
    return this->getRobotState() == other.getRobotState();
}

bool TimestampedRobotState::operator!=(const TimestampedRobotState &other) const
{
    return !(*this == other);
}
