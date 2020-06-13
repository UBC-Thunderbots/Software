#include "software/world/robot_state.h"

RobotState::RobotState(const Point &position, const Vector &velocity,
                       const Angle &orientation, const AngularVelocity &angular_velocity)
    : position_(position),
      velocity_(velocity),
      orientation_(orientation),
      angular_velocity_(angular_velocity)
{
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

bool RobotState::ballInBeam() const
{
    return ball_in_beam_;
}

void RobotState::updateBallInBeam(bool ball_in_beam)
{
    ball_in_beam_ = ball_in_beam;
}

unsigned long RobotState::timeSinceLastChip() const
{
    return time_since_last_chip_ms_;
}

void RobotState::updateTimeSinceLastChip(unsigned long time)
{
    time_since_last_chip_ms_ = time;
}

unsigned long RobotState::timeSinceLastKick() const
{
    return time_since_last_kick_ms_;
}

void RobotState::updateTimeSinceLastKick(unsigned long time)
{
    time_since_last_kick_ms_ = time;
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
