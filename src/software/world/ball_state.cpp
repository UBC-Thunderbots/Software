#include "software/world/ball_state.h"

BallState::BallState(const Point& position, const Vector& velocity,
                     const double distance_from_ground)
    : position_(position), velocity_(velocity), height_(distance_from_ground)
{
}

BallState::BallState(const TbotsProto::BallState& ball_state_proto)
    : position_(Point(ball_state_proto.global_position().x_meters(),
                      ball_state_proto.global_position().y_meters())),
      velocity_(Vector(ball_state_proto.global_velocity().x_component_meters(),
                       ball_state_proto.global_velocity().y_component_meters())),
      height_(ball_state_proto.distance_from_ground())
{
}

Point BallState::position() const
{
    return position_;
}

Vector BallState::velocity() const
{
    return velocity_;
}

double BallState::distanceFromGround() const
{
    return height_;
}

bool BallState::operator==(const BallState& other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity() &&
           this->distanceFromGround() == other.distanceFromGround();
}

bool BallState::operator!=(const BallState& other) const
{
    return !(*this == other);
}
