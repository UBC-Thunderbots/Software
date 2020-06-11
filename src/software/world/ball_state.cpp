#include "software/world/ball_state.h"

BallState::BallState(const Point& position, const Vector& velocity, const double height)
    : position_(position), velocity_(velocity), height_(height)
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

double BallState::height() const
{
    return height_;
}

bool BallState::operator==(const BallState& other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity() &&
           this->height() == other.height();
}

bool BallState::operator!=(const BallState& other) const
{
    return !(*this == other);
}
