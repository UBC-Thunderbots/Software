#include "ball_filter.h"


#include <fstream>
#include <iostream>
#include <iomanip>

static std::ofstream outfile;

BallFilter::BallFilter() : previous_ball_readings(5) {}


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

    previous_ball_readings.push_back(ball_velocity);

    auto average_ball_velocity = Vector(0, 0);
    for (auto v : previous_ball_readings)
    {
        average_ball_velocity = average_ball_velocity + v;
    }
    average_ball_velocity = average_ball_velocity.norm(average_ball_velocity.len() /
                                                       previous_ball_readings.size());
    if(!outfile.is_open()){
        outfile.open("/home/mathew/ball_filter_log.csv");
    }

    for(auto fd : new_ball_detections) {
            outfile << std::setprecision(17) << fd.timestamp.getSeconds() << ", "
            << fd.position.x() << ", "
            << fd.position.y() << ", "
            << std::endl;
    }

//        outfile.close();



    return Ball{filtered_detection.position, average_ball_velocity,
                filtered_detection.timestamp};
}
