#include "software/world/ball.h"

#include "shared/constants.h"
#include "software/physics/physics.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/almost_equal.h"
#include <cmath>

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

Ball::Ball(const TbotsProto::Ball &ball_proto)
    : current_state_(BallState(ball_proto.current_state())),
      timestamp_(Timestamp::fromTimestampProto(ball_proto.timestamp()))
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

    Duration effective_duration(duration_in_future);

    Vector future_velocity = calculateFutureVelocity(current_state_.velocity(),
                                                     acceleration_, duration_in_future);

//    bool velocities_in_opposite_direction = future_velocity.x() * current_state_.velocity().x() < 0 || future_velocity.y() * current_state_.velocity().y() < 0;
    bool velocities_in_opposite_direction =  std::abs(future_velocity.cross(current_state_.velocity())) < 1e-6;


    if(acceleration_.length() > 0 && velocities_in_opposite_direction){
        // this is the time it takes for velocity to accelerate to a stop
        effective_duration        = Duration::fromSeconds(current_state_.velocity().length() / acceleration_.length());

        //ball will stop
        future_velocity    = Vector();
    }

    const Point future_position =
        calculateFuturePosition(current_state_.position(), current_state_.velocity(),
                                acceleration_, effective_duration);

    return BallState(future_position, future_velocity);
}

Duration Ball::getTimeToPosition(const Point &destination) const{
    double time_to_stop = current_state_.velocity().length() / acceleration_.length();
    double d = distance(destination,current_state_.position());
    double a = -1 * std::max(1e-6, acceleration_.length());

    //solving for t using quadratic formula applied to kinematic equation
    double t_total =
            (-current_state_.velocity().length() + std::sqrt(std::pow(current_state_.velocity().length(), 2) + 2 * a* d)) /
            a;

    return Duration::fromSeconds(std::min(time_to_stop, t_total));
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
