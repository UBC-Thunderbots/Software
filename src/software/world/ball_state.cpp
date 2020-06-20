#include "software/world/ball_state.h"

BallState::BallState(const Point& position, const Vector& velocity,
                     const double distance_from_ground)
    : position_(position), velocity_(velocity), height_(distance_from_ground)
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
