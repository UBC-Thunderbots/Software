#include "software/world/robot_state.h"

RobotState::RobotState(const Point &position, const Vector &velocity,
                       const Angle &orientation, const AngularVelocity &angular_velocity, const double height)
    : position_(position),
      velocity_(velocity),
      orientation_(orientation),
      angular_velocity_(angular_velocity),
      height_(height)
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

double RobotState::height() const {
    return height_;
}

bool RobotState::operator==(const RobotState &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity() &&
           this->orientation() == other.orientation() &&
           this->angularVelocity() == other.angularVelocity() &&
           this->height() == other.height();
}

bool RobotState::operator!=(const RobotState &other) const
{
    return !(*this == other);
}
