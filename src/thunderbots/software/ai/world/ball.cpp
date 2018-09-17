#include "ball.h"

Ball::Ball(Point position, Vector velocity) : position_(position), velocity_(velocity)
{
}

void Ball::update(const Ball &new_ball_data)
{
    update(new_ball_data.position(), new_ball_data.velocity());
}

void Ball::update(const Point &new_position, const Vector &new_velocity)
{
    position_ = new_position;
    velocity_ = new_velocity;
}

Point Ball::position() const
{
    return position_;
}

Point Ball::estimatePositionAtFutureTime(double time_delta) const
{
    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/47
    return position_ + (velocity_.norm(time_delta * velocity_.len()));
}

Vector Ball::velocity() const
{
    return velocity_;
}

Vector Ball::estimateVelocityAtFutureTime(double time_delta) const
{
    // TODO: This is a implementation with an empirically determined time constant that
    // does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/47
    return velocity_ * exp(-0.1 * time_delta);
}

bool Ball::operator==(const Ball &other) const
{
    return this->position_ == other.position_ && this->velocity_ == other.velocity_;
}

bool Ball::operator!=(const Ball &other) const
{
    return !(*this == other);
}
