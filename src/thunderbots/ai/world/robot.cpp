#include "robot.h"

Robot::Robot(const unsigned int id)
    : id_(id),
      position_(Point()),
      velocity_(Point()),
      orientation_(Angle::zero()),
      angularVelocity_(Angle::zero())
{
}

void Robot::update(
    const Point &new_position, const Point &new_velocity, const Angle &new_orientation)
{
    position_        = new_position;
    velocity_        = new_velocity;
    orientation_     = new_orientation;
    angularVelocity_ = Angle::zero();
}

unsigned int Robot::id() const
{
    return id_;
}

Point Robot::position(const double time_delta) const
{
    return position_;
}

Point Robot::velocity(const double time_delta) const
{
    return velocity_;
}

Angle Robot::orientation(const double time_delta) const
{
    return orientation_;
}

Angle Robot::angularVelocity(const double time_delta) const
{
    return angularVelocity_;
}
