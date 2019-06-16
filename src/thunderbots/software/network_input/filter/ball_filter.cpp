#include "ball_filter.h"

#include <shared/constants.h>
#include <Eigen/Dense>
#include "geom/util.h"
#include <algorithm>
#include <limits>
#include "util/math_functions.h"

BallFilter::BallFilter(unsigned int min_buffer_size, unsigned int max_buffer_size) : _min_buffer_size(min_buffer_size), _max_buffer_size(max_buffer_size), ball_detection_buffer(max_buffer_size) {}

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
            auto detection_with_smallest_timestamp = *std::min_element(ball_detection_buffer.begin(), ball_detection_buffer.end());
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
            double maximum_acceptable_velocity_magnitude = BALL_MAX_SPEED_METERS_PER_SECOND + 2.0;
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
            ball_detection_buffer.push_front(detection);
        }
    }
}

BallVelocityEstimate BallFilter::estimateBallVelocity(boost::circular_buffer<SSLBallDetection> ball_detections) {
    // Sort the detections in decreasing order before processing. This places the most recent detections (with the
    // largest timestamp) at the front of the buffer, and the oldest detections (smallest timestamp) at the end of
    // the buffer
    std::sort(ball_detections.begin(), ball_detections.end());

    std::vector<double> ball_velocity_magnitudes;
    for(int i = 0; i < ball_detections.size() - 1; i++) {
        for(int j = i + 1; j < ball_detections.size(); j++) {
            SSLBallDetection previous_detection = ball_detections.at(i);
            SSLBallDetection current_detection = ball_detections.at(j);

            Duration time_diff = current_detection.timestamp - previous_detection.timestamp;
            // Avoid division by 0. If we have adjacent detections with the same timestamp the velocity cannot be
            // calculated
            if(time_diff.getSeconds() == 0){
                continue;
            }

            Vector velocity_vector = current_detection.position - previous_detection.position;
            double velocity_magnitude = velocity_vector.len() / time_diff.getSeconds();

            ball_velocity_magnitudes.emplace_back(velocity_magnitude);
        }
    }

    double velocity_magnitude_sum = 0;
    for(const auto& velocity_magnitude : ball_velocity_magnitudes) {
        velocity_magnitude_sum += velocity_magnitude;
    }
    double average_velocity_magnitude = velocity_magnitude_sum / ball_velocity_magnitudes.size();

    double velocity_magnitude_max = *std::max_element(ball_velocity_magnitudes.begin(), ball_velocity_magnitudes.end());
    double velocity_magnitude_min = *std::min_element(ball_velocity_magnitudes.begin(), ball_velocity_magnitudes.end());
    double min_max_average = (velocity_magnitude_min + velocity_magnitude_max) / 2.0;

    BallVelocityEstimate velocity_data({average_velocity_magnitude, min_max_average});

    return velocity_data;
}

boost::circular_buffer<SSLBallDetection> BallFilter::getResizedDetectionBuffer(
        boost::circular_buffer<SSLBallDetection> ball_detections) {
    // Sort the detections in decreasing order before processing. This places the most recent detections (with the
    // largest timestamp) at the front of the buffer, and the oldest detections (smallest timestamp) at the end of
    // the buffer
    std::sort(ball_detections.rbegin(), ball_detections.rend());

    double max_buffer_size_velocity_magnitude = BALL_MAX_SPEED_METERS_PER_SECOND - 1.5;
    double min_buffer_size_velocity_magnitude = 0.5;
    double buffer_size_velocity_magnitude_diff = max_buffer_size_velocity_magnitude - min_buffer_size_velocity_magnitude;

    unsigned int max_buffer_size = std::min(this->_max_buffer_size, static_cast<unsigned int>(ball_detections.size()));
    unsigned int min_buffer_size = std::min(this->_min_buffer_size, static_cast<unsigned int>(ball_detections.size()));
    double buffer_size_diff = max_buffer_size - min_buffer_size;

    double min_max_magnitude_average = estimateBallVelocity(ball_detections).min_max_magnitude_average;

    double linear_offset =  min_buffer_size_velocity_magnitude + (buffer_size_velocity_magnitude_diff / 2);
    double linear_scaling_factor = Util::linear(min_max_magnitude_average, linear_offset, buffer_size_velocity_magnitude_diff);

    int buffer_size = min_buffer_size + std::floor(linear_scaling_factor * buffer_size_diff);

    ball_detections.resize(buffer_size);

    return ball_detections;
}

LinearRegressionResults BallFilter::getLinearRegressionLine(
        const boost::circular_buffer<SSLBallDetection> &ball_detections) {
    // Construct matrix A and vector b for linear regression. The first column of A contains the bias variable, and the
    // second column contains the x coordinates of the ball. Vector b contains the y coordinates of the ball.
    Eigen::MatrixXf A(ball_detections.size(), 2);
    Eigen::VectorXf b(ball_detections.size());
    for(int i = 0; i < ball_detections.size(); i++) {
        // This extra column of 1's is the bias variable, so that we can regress with a y-intercept
        A(i, 0) = 1.0;
        A(i, 1) = ball_detections.at(i).position.x();

        b(i) = ball_detections.at(i).position.y();
    }

    // Perform linear regression to find the line of best fit through the ball positions.
    // This is solving the formula Ax = b, where x is the vector we want to solve for.
    Eigen::Vector2f regression_vector = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    // How to calculate the error is from https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    double regression_error = (A*regression_vector - b).norm() / b.norm(); // norm() is L2 norm

    // Find 2 points on the regression line that we solved for, and use this to construct our own Line class
    Eigen::Vector2f p1_vec(1, 0);
    Point p1(0, p1_vec.dot(regression_vector));
    Eigen::Vector2f p2_vec(1, 1);
    Point p2(1, p2_vec.dot(regression_vector));
    Line regression_line = Line(p1, p2);

    LinearRegressionResults results({regression_line, regression_error});

    return results;
}

Ball
BallFilter::getLinearRegressionPositionAndVelocity(boost::circular_buffer<SSLBallDetection> ball_detections) {
    auto resized_ball_detections = getResizedDetectionBuffer(ball_detections);

    // Sort the detections in decreasing order before processing. This places the most recent detections (with the
    // largest timestamp) at the front of the buffer, and the oldest detections (smallest timestamp) at the end of
    // the buffer
    std::sort(ball_detections.rbegin(), ball_detections.rend());

    auto x_vs_y_regression = getLinearRegressionLine(ball_detections);

     boost::circular_buffer<SSLBallDetection> inverse_ball_detections = ball_detections;
     for(auto& detection : ball_detections) {
         detection.position = Point(detection.position.y(), detection.position.x());
     }
     auto y_vs_x_regression = getLinearRegressionLine(inverse_ball_detections);
     y_vs_x_regression.regression_line = Line(Point(y_vs_x_regression.regression_line.getFirst().y(), y_vs_x_regression.regression_line.getFirst().x()),
             Point(y_vs_x_regression.regression_line.getSecond().y(), y_vs_x_regression.regression_line.getSecond().x()));

    Line regression_line = x_vs_y_regression.regression_error < y_vs_x_regression.regression_error ? x_vs_y_regression.regression_line : y_vs_x_regression.regression_line;

    // Take the position of the most recent ball position and project it onto the line of best fit. We do this because
    // we assume the ball must be travelling along its velocity vector, and this allows us to return more stable
    // position values since the line of best fit is less likely to fluctuate compared to the raw position of a ball
    // detection
    SSLBallDetection latest_ball_detection = ball_detections.front();
    Point filtered_ball_position = closestPointOnLine(latest_ball_detection.position, regression_line);

    // Project the velocity so it also lies along the line of best fit calculated above. Again, this gives more stable
    // values because the line of best fit is more robust to fluctiations in ball position, so will not vary as much
    // as using the "raw" ball velocity
    double ball_velocity_magnitude = estimateBallVelocity(resized_ball_detections).average_velocity_magnitude;
    Vector filtered_velocity = (regression_line.getSecond() - regression_line.getFirst()).norm(ball_velocity_magnitude);

    Ball ball(filtered_ball_position, filtered_velocity, latest_ball_detection.timestamp);
    return ball;
}

std::optional<Ball> BallFilter::getFilteredData(const std::vector<SSLBallDetection>& new_ball_detections, const Field& field)
{
    addNewDetectionsToBuffer(new_ball_detections, field);

    if(ball_detection_buffer.size() >= 2) {
        return getLinearRegressionPositionAndVelocity(ball_detection_buffer);
    }else if(ball_detection_buffer.size() == 1) {
        // If there is only 1 entry in the buffer, we can't calculate a velocity so
        // just set it to 0
        Ball filtered_ball = Ball(ball_detection_buffer.front().position, Vector(0, 0), ball_detection_buffer.front().timestamp);
        return filtered_ball;
    }else {
        return std::nullopt;
    }
}
