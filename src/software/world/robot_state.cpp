#include "software/world/robot_state.h"

RobotState::RobotState(const Point &position, const Vector &velocity,
                       const Angle &orientation, const AngularVelocity &angular_velocity)
    : position_(position),
      velocity_(velocity),
      orientation_(orientation),
      angular_velocity_(angular_velocity)
{
}

RobotState::RobotState(const TbotsProto::RobotState &robot_state_proto)
    : position_(Point(robot_state_proto.global_position().x_meters(),
                      robot_state_proto.global_position().y_meters())),
      velocity_(Vector(robot_state_proto.global_velocity().x_component_meters(),
                       robot_state_proto.global_velocity().y_component_meters())),
      orientation_(Angle::fromRadians(robot_state_proto.global_orientation().radians())),
      angular_velocity_(AngularVelocity::fromRadians(
          robot_state_proto.global_angular_velocity().radians_per_second()))
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
