#include "software/world/ball_state.h"

BallState::BallState(const Point& position, const Vector& velocity)
    : position_(position), velocity_(velocity)
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

bool BallState::operator==(const BallState& other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool BallState::operator!=(const BallState& other) const
{
    return !(*this == other);
}
