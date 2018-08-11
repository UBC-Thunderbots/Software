#include "ball.h"

Ball::Ball() : position_(Point()), velocity_(Vector())
{
}

void Ball::update(const thunderbots_msgs::Ball &ball_msg)
{
    position_ = Point(ball_msg.position.x, ball_msg.position.y);
    velocity_ = Vector(ball_msg.velocity.x, ball_msg.velocity.y);
}

void Ball::update(Point &new_position, Vector &new_velocity)
{
    position_ = new_position;
    velocity_ = new_velocity;
}

Point Ball::position(double time_delta) const
{
    return position_;
}

Vector Ball::velocity(double time_delta) const
{
    return velocity_;
}