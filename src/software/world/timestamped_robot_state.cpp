#include "software/world/timestamped_robot_state.h"

TimestampedRobotState::TimestampedRobotState(const Point &position,
                                             const Vector &velocity,
                                             const Angle &orientation,
                                             const AngularVelocity &angular_velocity,
                                             const Timestamp &timestamp)
    : robot_state_(position, velocity, orientation, angular_velocity),
      timestamp_(timestamp)
{
}

TimestampedRobotState::TimestampedRobotState(const RobotState &robot_state,
                                             const Timestamp &timestamp)
    : robot_state_(robot_state), timestamp_(timestamp)
{
}

Timestamp TimestampedRobotState::timestamp() const
{
    return timestamp_;
}

RobotState TimestampedRobotState::robotState() const
{
    return robot_state_;
}

bool TimestampedRobotState::operator==(const TimestampedRobotState &other) const
{
    return this->robotState() == other.robotState();
}

bool TimestampedRobotState::operator!=(const TimestampedRobotState &other) const
{
    return !(*this == other);
}
