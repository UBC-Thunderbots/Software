#include "ball_state.h"

BallState::BallState(Point position, Vector velocity, const Timestamp &timestamp)
{
    this->pos = position;
    this->vel = velocity;
    this->time = timestamp;
}

Point BallState::position() const {
    return pos;
}

Vector BallState::velocity() const {
    return vel;
}

Timestamp BallState::timestamp() const {
    return time;
}

bool BallState::operator==(const BallState &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool BallState::operator!=(const BallState &other) const
{
    return false;
}

