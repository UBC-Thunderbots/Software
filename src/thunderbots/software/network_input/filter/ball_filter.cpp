#include "ball_filter.h"

BallFilter::BallFilter() {}

FilteredBallData BallFilter::getFilteredData(std::vector<SSLBallData> new_ball_data)
{
    // The input messages do not seem to be transmitting ball velocity correctly so
    // calculate it based on the balls current and previous position.
    if (ball_data == nullptr)
    {
        ball_data = new BallData(new_ball_data[0].position, Vector(), new_ball_data[0].timestamp);
    }
    else
    {
        double time_diff = std::fabs(new_ball_data[0].timestamp - ball_data->timestamp);

        if (time_diff != 0)
        {
            Vector ball_velocity = (new_ball_data[0].position - ball_data->position) / time_diff;
            ball_data->velocity = ball_velocity;
        }

        ball_data->position = new_ball_data[0].position;
        ball_data->timestamp = new_ball_data[0].timestamp;
    }

    FilteredBallData filtered_data;
    filtered_data.position = new_ball_data[0].position;
    filtered_data.velocity = ball_data->velocity;
    // This is a placeholder timestamp for now. Timestamps should be fixed in
    // https://github.com/UBC-Thunderbots/Software/issues/228
    filtered_data.timestamp = Timestamp::getTimestampNow();

    return filtered_data;
}
