#include "ball_filter.h"

BallFilter::BallFilter()
{
    current_ball_position = Point();
    current_ball_velocity = Point();
}

void BallFilter::update(std::vector<SSLBallData> new_ball_data)
{
    if (!new_ball_data.empty())
    {
        Point old_position    = current_ball_position;
        current_ball_position = new_ball_data[0].position;
        current_ball_velocity = current_ball_position - old_position;
    }
}

Point BallFilter::getBallPosition()
{
    return current_ball_position;
}

Point BallFilter::getBallVelocity()
{
    return current_ball_velocity;
}
