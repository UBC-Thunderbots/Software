#include "ball_filter.h"

BallFilter::BallFilter()
{
    ball_data.position  = Point();
    ball_data.velocity  = Vector();
    ball_data.timestamp = 0;
}

FilteredBallData BallFilter::getFilteredData(std::vector<SSLBallData> new_ball_data)
{
    // The input messages do not seem to be transmitting ball velocity correctly so
    // calculate it based on the balls current and previous position.
    double time_diff = std::fabs(new_ball_data[0].timestamp - ball_data.timestamp);

    if (time_diff != 0)
    {
        Vector ball_velocity =
            (new_ball_data[0].position - ball_data.position) / time_diff;
        ball_data.velocity  = ball_velocity;
        ball_data.position  = new_ball_data[0].position;
        ball_data.timestamp = new_ball_data[0].timestamp;
    }

    FilteredBallData filtered_data;
    filtered_data.position = new_ball_data[0].position;
    filtered_data.velocity = ball_data.velocity;
    // TODO: This is a placeholder timestamp. It should be
    // This is a placeholder timestamp for now. Timestamps should be fixed in
    // https://github.com/UBC-Thunderbots/Software/issues/228
    filtered_data.timestamp = Timestamp::fromSeconds(new_ball_data[0].timestamp);

    return filtered_data;
}
