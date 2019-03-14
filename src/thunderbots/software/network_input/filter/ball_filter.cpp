#include "ball_filter.h"

Ball BallFilter::getFilteredData(const Ball& current_ball_state,
                                 const std::vector<SSLBallDetection>& new_ball_detections)
{
    if (new_ball_detections.empty())
    {
        return current_ball_state;
    }

    SSLBallDetection filtered_detection = new_ball_detections[0];
    Duration time_diff =
        filtered_detection.timestamp - current_ball_state.lastUpdateTimestamp();

    // The direction of the velocity
    Vector ball_velocity = filtered_detection.position - current_ball_state.position();
    // Set the magnitude based on the time minDiff
    ball_velocity = ball_velocity.norm(ball_velocity.len() / time_diff.getSeconds());

    return Ball{filtered_detection.position, ball_velocity, filtered_detection.timestamp};
}
