#include "software/world/ball_state.h"

BallState::BallState(Point position, Vector velocity, const Timestamp &timestamp)
{
    this->position_  = position;
    this->velocity_  = velocity;
    this->timestamp_ = timestamp;
}

Point BallState::position() const
{
    return position_;
}

Vector BallState::velocity() const
{
    return velocity_;
}

Timestamp BallState::timestamp() const
{
    return timestamp_;
}

bool BallState::operator==(const BallState &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool BallState::operator!=(const BallState &other) const
{
    return false;
}
