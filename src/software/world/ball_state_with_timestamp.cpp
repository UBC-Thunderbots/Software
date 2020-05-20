#include "software/world/ball_state_with_timestamp.h"

BallStateWithTimestamp::BallStateWithTimestamp(const Point& position, const Vector& velocity, const Timestamp &timestamp) : BallState(position, velocity), timestamp_(timestamp)
{
}

BallStateWithTimestamp::BallStateWithTimestamp(const BallState& ball_state, const Timestamp &timestamp) : BallState(ball_state), timestamp_(timestamp){
}

Timestamp BallStateWithTimestamp::timestamp() const
{
    return timestamp_;
}

BallState BallStateWithTimestamp::getBallState() const {
    return BallState(position_, velocity_);
}

bool BallStateWithTimestamp::operator==(const BallStateWithTimestamp &other) const
{
    return this->getBallState() == other.getBallState();
}

bool BallStateWithTimestamp::operator!=(const BallStateWithTimestamp &other) const
{
    return !(*this == other);
}
