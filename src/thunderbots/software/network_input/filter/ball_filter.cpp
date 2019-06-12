#include "ball_filter.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <shared/constants.h>
#include <Eigen/Dense>
#include "geom/util.h"
#include <algorithm>

BallFilter::BallFilter() : ball_detection_buffer(5), LINEAR_REGRESSION_BUFFER_SIZE(3) {}

void BallFilter::addNewDetectionsToBuffer(
        std::vector<SSLBallDetection> new_ball_detections, const Field &field) {
    // Sort the detections in increasing order before processing. This places the oldest detections (with the
    // smallest timestamp) at the front of the buffer, and the most recent detections (largest timestamp) at the end of
    // the buffer.
    std::sort(new_ball_detections.begin(), new_ball_detections.end());

    for(const auto& detection : new_ball_detections) {
        // Ignore any detections that are not anywhere within the field
        // and ignore any detections that are in the past
        if(!field.pointInEntireField(detection.position)) {
            continue;
        }

        if(!ball_detection_buffer.empty()) {
            auto detection_with_smallest_timestamp = *std::min(ball_detection_buffer.begin(), ball_detection_buffer.end());
            Duration time_diff = detection.timestamp - detection_with_smallest_timestamp.timestamp;

            // Ignore any data from the past, and any data that is as old as the oldest data in the buffer since it
            // provides no additional value. This also prevents division by 0 when calculating the estimated velocity
            if(time_diff.getSeconds() <= 0) {
                continue;
            }

            // We determine if the detection is noise based on how far it is from a ball
            // detection in the buffer. From this, we can calculate how fast the ball
            // must have moved to reach the new detection position. If this estimated
            // velocity is too far above the maximum allowed velocity, then there is a
            // good chance the detection is just noise and not the real ball. In this
            // case, we ignore the new "noise" data
            double detection_distance = (detection.position - detection_with_smallest_timestamp.position).len();
            double estimated_detection_velocity_magnitude = detection_distance / time_diff.getSeconds();

            // Make the maximum acceptable velocity a bit larger than the strict limits according to the game rules
            // to account for measurement error, and to be a bit on the safe side. We
            // don't want to risk discarding real data
            double maximum_acceptable_velocity_magnitude = BALL_MAX_SPEED_METERS_PER_SECOND + 1.0;
            if(estimated_detection_velocity_magnitude > maximum_acceptable_velocity_magnitude) {
                // If we determine the data to be noise, remove an entry from the buffer.
                // This way if we have messed up and now the ball is too far away for the
                // buffer to track, the buffer will rapidly shrink and start tracking the
                // ball at its new location once the buffer is empty.
                // We sort the vector in decreasing order first so that we can always ensure any elements that are
                // ejected from the end of the buffer are the oldest data
                std::sort(ball_detection_buffer.rbegin(), ball_detection_buffer.rend());
                ball_detection_buffer.pop_back();
            }else {
                // We sort the vector in decreasing order first so that we can always ensure any elements that are
                // ejected from the end of the buffer are the oldest data
                std::sort(ball_detection_buffer.rbegin(), ball_detection_buffer.rend());
                ball_detection_buffer.push_front(detection);
            }
        }else {
            // If there is no data in the buffer, we always add the new data
            // We sort the vector in decreasing order first so that we can always ensure any elements that are
            // ejected from the end of the buffer are the oldest data
            std::sort(ball_detection_buffer.rbegin(), ball_detection_buffer.rend());
            ball_detection_buffer.push_front(detection);
        }
    }
}

std::vector<Vector> BallFilter::calculateBallVelocities(
        boost::circular_buffer<SSLBallDetection> ball_detections) {
    // Sort the detections in decreasing order before processing. This places the most recent detections (with the
    // largest timestamp) at the front of the buffer, and the oldest detections (smallest timestamp) at the end of
    // the buffer
    std::sort(ball_detections.rbegin(), ball_detections.rend());

    std::vector<Vector> ball_velocities;
    for(int i = 0; i < ball_detections.size() - 1; i++) {
        SSLBallDetection current_detection = ball_detections.at(i);
        SSLBallDetection previous_detection = ball_detections.at(i + 1);

        Duration time_diff = current_detection.timestamp - previous_detection.timestamp;
        // Avoid division by 0. If we have adjacent detections with the same timestamp the velocity cannot be
        // calculated
        if(time_diff.getSeconds() == 0){
            continue;
        }

        Vector velocity_vector = current_detection.position - previous_detection.position;
        double velocity_magnitude = velocity_vector.len() / time_diff.getSeconds();
        Vector velocity = velocity_vector.norm(velocity_magnitude);

        ball_velocities.emplace_back(velocity);
    }

    return ball_velocities;
}

Ball
BallFilter::getLinearRegressionPositionAndVelocity(boost::circular_buffer<SSLBallDetection> ball_detections,
                                                   std::ofstream &outfile, bool draw) {
    // Sort the detections in decreasing order before processing. This places the most recent detections (with the
    // largest timestamp) at the front of the buffer, and the oldest detections (smallest timestamp) at the end of
    // the buffer
    std::sort(ball_detections.rbegin(), ball_detections.rend());
    // We use a slightly smaller buffer for the linear regression so that it is more responsive to bounces
    // and sudden changes in trajectory
    ball_detections.resize(LINEAR_REGRESSION_BUFFER_SIZE);

    // Construct matrix A and vector b for linear regression. The first column of A contains the bias variable, and the
    // second column contains the x coordinates of the ball. Vector b contains the y coordinates of the ball.
    Eigen::MatrixXf A(ball_detections.size(), 2);
    Eigen::VectorXf b(ball_detections.size());
    for(int i = 0; i < ball_detections.size(); i++) {
        A(i, 0) = 1.0;
        A(i, 1) = ball_detections.at(i).position.x();

        b(i) = ball_detections.at(i).position.y();
    }

    // Perform linear regression to find the line of best fit through the ball positions.
    // This is solving the formula Ax = b, where x is the vector we want to solve for.
    Eigen::Vector2f result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Find 2 points on the line of best fit that we solved for, and use these points to create a Vector along the
    // line of best fit. We will use this as the velocity vector of the ball.
    Eigen::Vector2f p1_vec(1, 0);
    Point p1(0, p1_vec.dot(result));
    Eigen::Vector2f p2_vec(1, 1);
    Point p2(1, p2_vec.dot(result));
    Vector best_fit_line_vector = p2 - p1;

    // Take the position of the most recent ball position and project it onto the line of best fit. We do this because
    // we assume the ball must be travelling along its velocity vector, and this allows us to return more stable
    // position values since the line of best fit is less likely to fluctuate compared to the raw position of a ball
    // detection
    SSLBallDetection latest_ball_detection = ball_detections.front();
    Point filtered_ball_position = closestPointOnLine(latest_ball_detection.position, p1, p2);

    // Calculate the velocity of the ball using the average of the velocities calculated from the values currently
    // in the detection buffer
    auto velocities = calculateBallVelocities(ball_detections);
    Vector average_velocity = Vector(0, 0);
    for(const auto& v : velocities) {
        average_velocity = average_velocity + v;
    }
    average_velocity = average_velocity.norm(average_velocity.len() / velocities.size());
    double average_velocity_magnitude = average_velocity.len();

    // Project the velocity so it also lies along the line of best fit calculated above. Again, this gives more stable
    // values because the line of best fit is more robust to fluctiations in ball position, so will not vary as much
    // as using the "raw" ball velocity
    Vector vel_along_line = average_velocity.project(best_fit_line_vector);
    Vector filtered_velocity = vel_along_line.norm(average_velocity_magnitude);

    if(draw) {
        Point r1 = ball_detections.begin()->position;
        Point r2 = (ball_detections.end()-1)->position;
        Point g1 = closestPointOnLine(r1, p1, p2);
        Point g2 = closestPointOnLine(r2, p1, p2);
        for(auto d : ball_detections) {
            outfile << d.position.x() << ", " << d.position.y() << ", "
            << g1.x() << ", " << g1.y() << ", "
            << g2.x() << ", " << g2.y() << ", "
            << std::endl;
        }
    }


    Ball ball(filtered_ball_position, filtered_velocity, latest_ball_detection.timestamp);
    return ball;
}

std::optional<Ball>
BallFilter::getFilteredData(const std::vector<SSLBallDetection> &new_ball_detections, const Field &field,
                            std::ofstream &outfile,
                            bool draw)
{
    addNewDetectionsToBuffer(new_ball_detections, field);

    if(ball_detection_buffer.size() >= 2) {
        return getLinearRegressionPositionAndVelocity(ball_detection_buffer, outfile, draw);
    }else if(ball_detection_buffer.size() == 1) {
        // If there is only 1 entry in the buffer, we can't calculate a velocity so
        // just set it to 0
        Ball filtered_ball = Ball(ball_detection_buffer.front().position, Vector(0, 0), ball_detection_buffer.front().timestamp);
        return filtered_ball;
    }else {
        return std::nullopt;
    }
}
