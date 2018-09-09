#include "ai/world/robot.h"

Robot::Robot(const unsigned int id)
    : id_(id),
      position_(Point()),
      velocity_(Vector()),
      orientation_(Angle::zero()),
      angularVelocity_(AngularVelocity::zero())
{
}

Robot::Robot(unsigned int id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity)
    : id_(id),
      position_(position),
      velocity_(velocity),
      orientation_(orientation),
      angularVelocity_(angular_velocity)
{
}

Robot::Robot(const thunderbots_msgs::Robot &robot_msg)
    : id_(robot_msg.id),
      position_(Point(robot_msg.position.x, robot_msg.position.y)),
      velocity_(Vector(robot_msg.velocity.x, robot_msg.velocity.y)),
      orientation_(Angle::ofRadians(robot_msg.orientation)),
      angularVelocity_(AngularVelocity::ofRadians(robot_msg.angular_velocity))
{
}

void Robot::update(const Point &new_position, const Vector &new_velocity,
                   const Angle &new_orientation,
                   const AngularVelocity &new_angular_velocity)
{
    position_        = new_position;
    velocity_        = new_velocity;
    orientation_     = new_orientation;
    angularVelocity_ = new_angular_velocity;
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

bool Robot::operator==(const Robot &other) const
{
    return this->id_ == other.id_ && this->position_ == other.position_ &&
           this->velocity_ == other.velocity_ &&
           this->orientation_ == other.orientation_ &&
           this->angularVelocity_ == other.angularVelocity_;
}

bool Robot::operator!=(const Robot &other) const
{
    return !(*this == other);
}