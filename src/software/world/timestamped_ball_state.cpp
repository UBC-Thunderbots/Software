#include "software/world/timestamped_ball_state.h"

TimestampedBallState::TimestampedBallState(const Point &position, const Vector &velocity,
                                           const Timestamp &timestamp)
    : ball_state_(position, velocity), timestamp_(timestamp)
{
}

TimestampedBallState::TimestampedBallState(const BallState &ball_state,
                                           const Timestamp &timestamp)
    : ball_state_(ball_state), timestamp_(timestamp)
{
}

Timestamp TimestampedBallState::timestamp() const
{
    return timestamp_;
}

BallState TimestampedBallState::state() const
{
    return ball_state_;
}

bool TimestampedBallState::operator==(const TimestampedBallState &other) const
{
    return this->state() == other.state();
}

bool TimestampedBallState::operator!=(const TimestampedBallState &other) const
{
    return !(*this == other);
}
