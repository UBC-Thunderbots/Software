#include "software/world/timestamped_ball_state.h"

TimestampedBallState::TimestampedBallState(const Point &position,
                                           const Vector &velocity,
                                           const Timestamp &timestamp)
    : BallState(position, velocity), timestamp_(timestamp)
{
}

TimestampedBallState::TimestampedBallState(const BallState &ball_state,
                                           const Timestamp &timestamp)
    : BallState(ball_state), timestamp_(timestamp)
{
}

Timestamp TimestampedBallState::timestamp() const
{
    return timestamp_;
}

BallState TimestampedBallState::getBallState() const
{
    return BallState(position(), velocity());
}

bool TimestampedBallState::operator==(const TimestampedBallState &other) const
{
    return this->getBallState() == other.getBallState();
}

bool TimestampedBallState::operator!=(const TimestampedBallState &other) const
{
    return !(*this == other);
}
