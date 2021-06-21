#include "software/world/ball.h"

#include "shared/constants.h"
#include "software/geom/algorithms/almost_equal.h"
#include "software/physics/physics.h"


Ball::Ball(const Point &position, const Vector &velocity, const Timestamp &timestamp,
           const Vector &acceleration)
    : Ball(BallState(position, velocity), timestamp, acceleration)
{
}

Ball::Ball(const BallState &initial_state, const Timestamp &timestamp,
           const Vector &acceleration)
    : current_state_(initial_state), timestamp_(timestamp), acceleration_(acceleration)
{
}

BallState Ball::currentState() const
{
    return current_state_;
}

void Ball::updateState(const BallState &new_state, const Timestamp &new_timestamp,
                       const Vector &new_acceleration)
{
    if (new_timestamp < timestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older than the current state");
    }
    current_state_ = new_state;
    timestamp_     = new_timestamp;
    acceleration_  = new_acceleration;
}

Timestamp Ball::timestamp() const
{
    return timestamp_;
}

Point Ball::position() const
{
    return current_state_.position();
}

Vector Ball::velocity() const
{
    return current_state_.velocity();
}

Vector Ball::acceleration() const
{
    return acceleration_;
}

BallState Ball::estimateFutureState(const Duration &duration_in_future) const
{
    Vector future_velocity = calculateFutureVelocity(current_state_.velocity(),
                                                     acceleration_, duration_in_future);

    // since acceleration is due to friction, the ball will slow down until it stops and
    // will not move backwards. This means final velocity cannot be in the opposite
    // direction to initial velocity. if that happens, we find a new velocity using a more
    // realistic Duration
    Duration effective_duration(duration_in_future);

    bool velocities_in_opposite_direction =
        almostEqual(future_velocity.rotate(Angle::half()).orientation().toRadians(),
                    current_state_.velocity().orientation().toRadians(), FIXED_EPSILON,
                    ULPS_EPSILON_TEN);

    if (acceleration_.length() > 0 && velocities_in_opposite_direction)
    {
        // this is the time it takes for velocity to accelerate to 0
        double time        = current_state_.velocity().length() / acceleration_.length();
        effective_duration = Duration::fromSeconds(time);
        future_velocity    = calculateFutureVelocity(current_state_.velocity(),
                                                  acceleration_, effective_duration);
    }

    const Point future_position =
        calculateFuturePosition(current_state_.position(), current_state_.velocity(),
                                acceleration_, effective_duration);

    return BallState(future_position, future_velocity);
}

bool Ball::hasBallBeenKicked(const Angle &expected_kick_direction,
                             double min_kick_speed) const
{
    // 20deg arbitrarily chosen as the maximum angle difference to determine
    // if ball has been kicked in the approximate direction as expected
    static constexpr Angle MAX_ANGLE_DIFFERENCE = Angle::fromDegrees(20);

    Angle kick_orientation_difference =
        velocity().orientation().minDiff(expected_kick_direction);

    return (kick_orientation_difference.abs() < MAX_ANGLE_DIFFERENCE &&
            velocity().length() >= min_kick_speed);
}

bool Ball::operator==(const Ball &other) const
{
    return this->position() == other.position() && this->velocity() == other.velocity();
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
