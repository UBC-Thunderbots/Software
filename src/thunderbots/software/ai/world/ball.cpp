#include "ball.h"

Ball::Ball() : position_(Point()), velocity_(Point())
{
}

void Ball::update(Point &new_position, Point &new_velocity)
{
    position_ = new_position;
    velocity_ = new_velocity;
}

Point Ball::position(double time_delta) const
{
    return position_;
}

Point Ball::velocity(double time_delta) const
{
    return velocity_;
}