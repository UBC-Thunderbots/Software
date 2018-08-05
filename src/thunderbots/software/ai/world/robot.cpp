#include "robot.h"

Robot::Robot(const unsigned int id)
    : id_(id),
      position_(Point()),
      velocity_(Vector()),
      orientation_(Angle::zero()),
      angularVelocity_(AngularVelocity::zero())
{
}

void Robot::update(
    const Point &new_position, const Vector &new_velocity, const Angle &new_orientation,
    const AngularVelocity &new_angular_velocity)
{
    position_        = new_position;
    velocity_        = new_velocity;
    orientation_     = new_orientation;
    angularVelocity_ = new_angular_velocity;
}

void Robot::update(const thunderbots_msgs::Robot &robot_msg)
{
    assert(robot_msg.id == id_);

    Point position         = Point(robot_msg.position.x, robot_msg.position.y);
    Point velocity         = Vector(robot_msg.velocity.x, robot_msg.velocity.y);
    Angle orientation      = Angle::ofRadians(robot_msg.orientation);
    Angle angular_velocity = AngularVelocity::ofRadians(robot_msg.angular_velocity);

    update(position, velocity, orientation, angular_velocity);
}

unsigned int Robot::id() const
{
    return id_;
}

Point Robot::position(const double time_delta) const
{
    return position_;
}

Vector Robot::velocity(const double time_delta) const
{
    return velocity_;
}

Angle Robot::orientation(const double time_delta) const
{
    return orientation_;
}

AngularVelocity Robot::angularVelocity(const double time_delta) const
{
    return angularVelocity_;
}
