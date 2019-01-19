#include "ball_filter.h"

BallFilter::BallFilter() {}

FilteredBallData BallFilter::getFilteredData(std::vector<SSLBallData> new_ball_data)
{
    FilteredBallData filtered_data;
    filtered_data.position = new_ball_data[0].position;
    filtered_data.velocity = Vector();
    // This is a placeholder timestamp for now. Timestamps should be fixed in
    // https://github.com/UBC-Thunderbots/Software/issues/228
    filtered_data.timestamp = Timestamp::getTimestampNow();

    return filtered_data;
}
