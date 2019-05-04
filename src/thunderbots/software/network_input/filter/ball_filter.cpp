#include "ball_filter.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <shared/constants.h>
#include <Eigen/Dense>
#include "geom/util.h"

static std::ofstream outfile;

BallFilter::BallFilter() : ball_detection_buffer(5), LINEAR_REGRESSION_BUFFER_SIZE(3) {}

void BallFilter::addNewDetectionsToBuffer(
        std::vector<SSLBallDetection> new_ball_detections, const Field &field) {
    // Sort the given ball detections in increasing order. This means that detections with
    // smaller timestamps are at the front of the vector, and detections with larger
    // timestamps are at the back
    std::sort(new_ball_detections.rbegin(), new_ball_detections.rend());

    // Get the previous ball detection if it exists
    std::optional<SSLBallDetection> previous_ball_detection = ball_detection_buffer.empty() ? std::nullopt : std::make_optional(ball_detection_buffer.front());

    for(const auto& detection : new_ball_detections) {
        // Ignore any detections that are not anywhere within the field
        // and ignore any detections that are in the past
        if(!field.pointInEntireField(detection.position) || (previous_ball_detection && detection.timestamp < previous_ball_detection->timestamp)) {
            continue;
        }

        if(previous_ball_detection) {
            // Estimate the velocity of the ball by comparing the new detection against
            // the most recent data in the buffer
            Duration time_diff = detection.timestamp - previous_ball_detection->timestamp;
            if(time_diff.getSeconds() < 0) {
                // Ignore any data from the past
                continue;
            }
            // TODO: division by 0
            // We determine if the detection is noise based on how far it is from the
            // last known ball position. From this, we can calculate how fast the ball
            // must have moved to reach the new detection position. If this estimated
            // velocity is too far above the maximum allowed velocity, then there is a
            // good chance the detection is just noise and not the real ball. In this
            // case, we ignore the new "noise" data
            double detection_distance = (detection.position - previous_ball_detection->position).len();
            double estimated_detection_velocity_magnitude = detection_distance / time_diff.getSeconds();
            // Make the maximum acceptable velocity a bit larger than the strict limits
            // to account for measurement error, and to be a bit on the safe side. We
            // don't want to risk discarding real data
            double maximum_acceptable_velocity_magnitude = BALL_MAX_SPEED_METERS_PER_SECOND + 0.5;
            if(estimated_detection_velocity_magnitude > maximum_acceptable_velocity_magnitude) {
                // If we determine the data to be noise, remove an entry from the buffer.
                // This way if we have messed up and now the ball is too far away for the
                // buffer to track, the buffer will rapidly shrink and start tracking the
                // ball at its new location once the buffer is empty
                ball_detection_buffer.pop_back();
            }else {
                ball_detection_buffer.push_front(detection);
            }
        }else {
            // If there is no data in the buffer, we always add the new data
            ball_detection_buffer.push_front(detection);
        }
    }
}

std::vector<Vector> BallFilter::calculateBallVelocities(
        const boost::circular_buffer<SSLBallDetection> &ball_detections) {
    std::vector<Vector> ball_velocities;
    for(int i = 0; i < ball_detections.size() - 1; i++) {
        SSLBallDetection current_detection = ball_detections.at(i);
        SSLBallDetection previous_detection = ball_detections.at(i + 1);

        Duration time_diff = current_detection.timestamp - previous_detection.timestamp;
        // TODO: division by 0
        Vector velocity_vector = current_detection.position - previous_detection.position;
        double velocity_magnitude = velocity_vector.len() / time_diff.getSeconds();
        Vector velocity = velocity_vector.norm(velocity_magnitude);

        ball_velocities.emplace_back(velocity);
    }

    return ball_velocities;
}

Ball BallFilter::getAveragedBallPositionAndVelocity() {
    Point average_position = Point(0, 0);
    for(const auto& detection : ball_detection_buffer) {
        average_position = average_position + detection.position;
    }

    average_position = average_position.norm(average_position.len() / ball_detection_buffer.size());

    std::vector<Vector> buffer_velocities = calculateBallVelocities(ball_detection_buffer);
    Vector average_velocity = Vector(0, 0);
    for(const auto& velocity : buffer_velocities) {
        average_velocity = average_velocity + velocity;
    }

    average_velocity = average_velocity.norm(average_velocity.len() / buffer_velocities.size());

    Ball filtered_ball = Ball(average_position, average_velocity, ball_detection_buffer.front().timestamp);
    return filtered_ball;
}

using namespace Eigen;
using namespace std;
Ball BallFilter::getLinearRegressionPositionAndVelocity() {
    MatrixXf A(ball_detection_buffer.size(), 2);
    VectorXf b(ball_detection_buffer.size());
    for(int i = 0; i < ball_detection_buffer.size(); i++) {
        A(i, 0) = 1.0;
        A(i, 1) = ball_detection_buffer.at(i).position.x();

        b(i) = ball_detection_buffer.at(i).position.y();
    }
//    cout << "Here is the matrix A:\n" << A << endl;
////    VectorXf b = VectorXf::Random(2);
//    cout << "Here is the right hand side b:\n" << b << endl;
//    cout << "The least-squares solution is:\n"
//         << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;

    Vector2f result = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
//    std::cout << "result again\n" << result << std::endl;


    volatile int ff = 2;

    Vector2f p1_vec(1, 0);
    Point p1(0, p1_vec.dot(result));
    Vector2f p2_vec(1, 1);
    Point p2(1, p2_vec.dot(result));
    Vector best_fit_line_vector = p2 - p1;

    SSLBallDetection latest_detection = ball_detection_buffer.front();

    Point p = closestPointOnLine(latest_detection.position, p1, p2);


    // TODO: project along the line?
    auto velocities = calculateBallVelocities(ball_detection_buffer);
    Vector average_velocity = Vector(0, 0);
    for(const auto& v : velocities) {
        average_velocity = average_velocity + v;
    }
    average_velocity = average_velocity.norm(average_velocity.len() / velocities.size());
    double average_velocity_magnitude = average_velocity.len();

    Vector vel_along_line = average_velocity.project(best_fit_line_vector);
    Vector filtered_velocity = vel_along_line.norm(average_velocity_magnitude);

    Ball ball(p, filtered_velocity, latest_detection.timestamp);
    return ball;
}

std::optional<Ball> BallFilter::getFilteredData(const std::vector<SSLBallDetection>& new_ball_detections, const Field& field)
{
    addNewDetectionsToBuffer(new_ball_detections, field);

    if(ball_detection_buffer.size() >= 2) {
        Vector estimated_ball_velocity_vector = ball_detection_buffer.at(0).position - ball_detection_buffer.at(1).position;
        Duration time_diff = ball_detection_buffer.at(0).timestamp - ball_detection_buffer.at(1).timestamp;
        double estimated_ball_velocity_magnitude = estimated_ball_velocity_vector.len() / time_diff.getSeconds();

        if(estimated_ball_velocity_magnitude < 0.5) {
            // If the ball is moving slowly, we use the average position and velocity
            // of the buffer
            return getAveragedBallPositionAndVelocity();
        }else {
            // If the ball is moving quickly, we apply linear regression to the positions
            // in the buffer to calculate the filtered position. We then calculate
            // the velocity from the filtered position
            return getLinearRegressionPositionAndVelocity();
        }
    }else if(ball_detection_buffer.size() == 1) {
        // If there is only 1 entry in the buffer, we can't calculate a velocity so
        // just set it to 0
        Ball filtered_ball = Ball(ball_detection_buffer.front().position, Vector(0, 0), ball_detection_buffer.front().timestamp);
        return filtered_ball;
    }else {
        return std::nullopt;
    }





//    if (new_ball_detections.empty())
//    {
//        return current_ball_state;
//    }
//
//    SSLBallDetection filtered_detection = new_ball_detections[0];
//    Duration time_diff =
//        filtered_detection.timestamp - current_ball_state.lastUpdateTimestamp();
//
//    // The direction of the velocity
//    Vector ball_velocity = filtered_detection.position - current_ball_state.position();
//    // Set the magnitude based on the time minDiff
//    ball_velocity = ball_velocity.norm(ball_velocity.len() / time_diff.getSeconds());
//
//    previous_ball_readings.push_back(ball_velocity);
//
//    auto average_ball_velocity = Vector(0, 0);
//    for (auto v : previous_ball_readings)
//    {
//        average_ball_velocity = average_ball_velocity + v;
//    }
//    average_ball_velocity = average_ball_velocity.norm(average_ball_velocity.len() /
//                                                       previous_ball_readings.size());
//    if(!outfile.is_open()){
//        outfile.open("/home/mathew/ball_filter_log.csv");
//    }
//
//    for(auto fd : new_ball_detections) {
//            outfile << std::setprecision(17) << fd.timestamp.getSeconds() << ", "
//            << fd.position.x() << ", "
//            << fd.position.y() << ", "
//            << ball_velocity.x() << ", "
//            << ball_velocity.y() << ", "
//            << average_ball_velocity.x() << ", "
//            << average_ball_velocity.y() << ", "
//            << std::endl;
//    }
//
////        outfile.close();
//
//
//
//    return Ball{filtered_detection.position, average_ball_velocity,
//                filtered_detection.timestamp};
}
