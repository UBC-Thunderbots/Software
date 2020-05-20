#include "software/world/ball_state_with_timestamp.h"

BallStateWithTimestamp::BallStateWithTimestamp(Point position, Vector velocity, const Timestamp &timestamp)
{
    this->position_  = position;
    this->velocity_  = velocity;
    this->timestamp_ = timestamp;
}

Point BallStateWithTimestamp::position() const
{
    return position_;
}

Vector BallStateWithTimestamp::velocity() const
{
    return velocity_;
}

Timestamp BallStateWithTimestamp::timestamp() const
{
    return timestamp_;
}

bool BallStateWithTimestamp::operator==(const BallStateWithTimestamp &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool BallStateWithTimestamp::operator!=(const BallStateWithTimestamp &other) const
{
    return !(*this == other);
}
