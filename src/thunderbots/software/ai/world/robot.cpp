#include "robot.h"

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

void Robot::update(const Point &new_position, const Vector &new_velocity,
                   const Angle &new_orientation,
                   const AngularVelocity &new_angular_velocity)
{
    position_        = new_position;
    velocity_        = new_velocity;
    orientation_     = new_orientation;
    angularVelocity_ = new_angular_velocity;
}

void Robot::update(const Robot &new_robot_data)
{
    if (new_robot_data.id() != id())
    {
        // TODO: Throw a proper exception here. We should not update a robot using a robot
        // with a different id (a different robot)
        // https://github.com/UBC-Thunderbots/Software/issues/16
        std::cerr << "Error: Robot updated using a robot with a mismatched id"
                  << std::endl;
        exit(1);
    }

    update(new_robot_data.position(), new_robot_data.velocity(),
           new_robot_data.orientation(), new_robot_data.angularVelocity());
}

unsigned int Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return position_;
}

Point Robot::estimatePositionAtFutureTime(double time_delta) const
{
    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    return position_ + velocity_.norm(velocity_.len() * time_delta);
}

Vector Robot::velocity() const
{
    return velocity_;
}

Vector Robot::estimateVelocityAtFutureTime(double time_delta) const
{
    // TODO: This simple implementation that assumes the robot maintains the same velocity
    // and does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/50
    return velocity_;
}

Angle Robot::orientation() const
{
    return orientation_;
}

Angle Robot::estimateOrientationAtFutureTime(double time_delta) const
{
    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Orientation prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    return orientation_ + angularVelocity_ * time_delta;
}

AngularVelocity Robot::angularVelocity() const
{
    return angularVelocity_;
}

AngularVelocity Robot::estimateAngularVelocityAtFutureTime(double time_delta) const
{
    // TODO: This simple implementation that assumes the robot maintains the same
    // angular velocity and does not necessarily reflect real-world behavior. Angular
    // velocity prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
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
