#include "software/world/robot_state.h"

RobotState::RobotState(const Point &position, const Vector &velocity,
                       const Angle &orientation, const AngularVelocity &angular_velocity)
    : position_(position),
      velocity_(velocity),
      orientation_(orientation),
      angular_velocity_(angular_velocity),
      ball_in_beam_(false),
      time_since_last_chip_ms_(std::nullopt),
      time_since_last_kick_ms_(std::nullopt)
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

bool RobotState::ballInMouth() const
{
    return ball_in_beam_;
}

void RobotState::setBallInMouth(bool ball_in_beam)
{
    ball_in_beam_ = ball_in_beam;
}

std::optional<unsigned long> RobotState::timeSinceLastChip() const
{
    return time_since_last_chip_ms_;
}

void RobotState::setTimeSinceLastChip(unsigned long time_ms)
{
    time_since_last_chip_ms_ = time_ms;
}

std::optional<unsigned long> RobotState::timeSinceLastKick() const
{
    return time_since_last_kick_ms_;
}

void RobotState::setTimeSinceLastKick(unsigned long time_ms)
{
    time_since_last_kick_ms_ = time_ms;
}

bool RobotState::operator==(const RobotState &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity() &&
           this->orientation() == other.orientation() &&
           this->ballInMouth() == other.ballInMouth() &&
           this->timeSinceLastChip() == other.timeSinceLastChip() &&
           this->timeSinceLastKick() == other.timeSinceLastKick() &&
           this->angularVelocity() == other.angularVelocity();
}

bool RobotState::operator!=(const RobotState &other) const
{
    return !(*this == other);
}
