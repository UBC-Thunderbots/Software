#include "software/sensor_fusion/filter/ball_filter.h"

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <vector>

#include "shared/constants.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"


BallFilter::BallFilter() : ball_detection_buffer(MAX_BUFFER_SIZE) {}

std::optional<Ball> BallFilter::estimateBallState(
    const std::vector<BallDetection> &new_ball_detections, const Rectangle &filter_area)
{
    addNewDetectionsToBuffer(new_ball_detections, filter_area);
    return estimateBallStateFromBuffer(ball_detection_buffer);
}

void BallFilter::addNewDetectionsToBuffer(std::vector<BallDetection> new_ball_detections,
                                          const Rectangle &filter_area)
{
    // Sort the detections in increasing order before processing. This places the oldest
    // detections (with the smallest timestamp) at the front of the buffer, and the most
    // recent detections (largest timestamp) at the end of the buffer.
    std::sort(new_ball_detections.begin(), new_ball_detections.end());

    for (const auto &detection : new_ball_detections)
    {
        // Remove any detections outside the filter area
        if (!contains(filter_area, detection.position))
        {
            continue;
        }

        if (!ball_detection_buffer.empty())
        {
            // Use the smallest timestamp to minimize time_diffs of 0
            auto detection_with_smallest_timestamp = *std::min_element(
                ball_detection_buffer.begin(), ball_detection_buffer.end());
            Duration time_diff =
                detection.timestamp - detection_with_smallest_timestamp.timestamp;

            // Ignore any data from the past, and any data that is as old as the oldest
            // data in the buffer since it provides no additional value. This also
            // prevents division by 0 when calculating the estimated velocity
            if (time_diff.toSeconds() <= 0)
            {
                continue;
            }

            // We determine if the detection is noise based on how far it is from a ball
            // detection in the buffer. From this, we can calculate how fast the ball
            // must have moved to reach the new detection position. If this estimated
            // velocity is too far above the maximum allowed velocity, then there is a
            // good chance the detection is just noise and not the real ball. In this
            // case, we ignore the new "noise" data
            double detection_distance =
                (detection.position - detection_with_smallest_timestamp.position)
                    .length();
            double estimated_detection_velocity_magnitude =
                detection_distance / time_diff.toSeconds();

            // Make the maximum acceptable velocity a bit larger than the strict limits
            // according to the game rules to account for measurement error, and to be a
            // bit on the safe side. We don't want to risk discarding real data.
            double maximum_acceptable_velocity_magnitude =
                BALL_MAX_SPEED_METERS_PER_SECOND + MAX_ACCEPTABLE_BALL_SPEED_BUFFER;
            if (estimated_detection_velocity_magnitude >
                maximum_acceptable_velocity_magnitude
                || ((detection.is_from_break_beam || detection_with_smallest_timestamp.is_from_break_beam)
                && detection_distance > MAX_ACCEPTABLE_BREAK_BEAM_DETECTION_DISTANCE))
            {
                // If we determine the data to be noise, remove an entry from the buffer.
                // This way if we have messed up and now the ball is too far away for the
                // buffer to track, the buffer will rapidly shrink and start tracking the
                // ball at its new location once the buffer is empty.
                // We sort the vector in decreasing order first so that we can always
                // ensure any elements that are ejected from the end of the buffer are the
                // oldest data
                std::sort(ball_detection_buffer.rbegin(), ball_detection_buffer.rend());
                ball_detection_buffer.pop_back();
            }
            else
            {
                // We sort the vector in decreasing order first so that we can always
                // ensure any elements that are ejected from the end of the buffer are the
                // oldest data
                std::sort(ball_detection_buffer.rbegin(), ball_detection_buffer.rend());
                ball_detection_buffer.push_front(detection);
            }
        }
        else
        {
            // If there is no data in the buffer, we always add the new data
            ball_detection_buffer.push_front(detection);
        }
    }
}

std::optional<Ball> BallFilter::estimateBallStateFromBuffer(
    boost::circular_buffer<BallDetection> ball_detections)
{
    // Sort the detections in decreasing order before processing. This places the most
    // recent detections (with the largest timestamp) at the front of the buffer, and the
    // oldest detections (smallest timestamp) at the end of the buffer
    std::sort(ball_detections.rbegin(), ball_detections.rend());

    if (ball_detections.empty())
    {
        return std::nullopt;
    }
    else if (ball_detections.size() == 1)
    {
        // If there is only 1 entry in the buffer, we can't fit a regression line
        // or calculate a velocity so we do our best with just the position
        BallState ball_state(ball_detections.front().position, Vector(0, 0),
                             ball_detections.front().distance_from_ground);
        Ball ball(ball_state, ball_detections.front().timestamp);
        return ball;
    }

    std::optional<size_t> adjusted_buffer_size = getAdjustedBufferSize(ball_detections);
    if (!adjusted_buffer_size)
    {
        return std::nullopt;
    }
    ball_detections.resize(*adjusted_buffer_size);

    auto regression = calculateLineOfBestFit(ball_detections);

    Point filtered_position =
        estimateBallPosition(ball_detections, regression.regression_line);

    auto estimated_velocity = estimateBallVelocity(ball_detections, std::nullopt);

    if (regression.regression_error < LINEAR_REGRESSION_ERROR_THRESHOLD)
    {
        estimated_velocity =
            estimateBallVelocity(ball_detections, regression.regression_line);
    }
    if (!estimated_velocity)
    {
        return std::nullopt;
    }

    BallState ball_state(filtered_position, estimated_velocity->average_velocity,
                         ball_detections.front().distance_from_ground);
    return Ball(ball_state, ball_detections.front().timestamp);
}

std::optional<size_t> BallFilter::getAdjustedBufferSize(
    boost::circular_buffer<BallDetection> ball_detections)
{
    // Sort the detections in decreasing order before processing. This places the most
    // recent detections (with the largest timestamp) at the front of the buffer, and the
    // oldest detections (smallest timestamp) at the end of the buffer
    std::sort(ball_detections.rbegin(), ball_detections.rend());

    double buffer_size_velocity_magnitude_diff =
        MAX_BUFFER_SIZE_VELOCITY_MAGNITUDE - MIN_BUFFER_SIZE_VELOCITY_MAGNITUDE;

    unsigned int max_buffer_size =
        std::min(MAX_BUFFER_SIZE, static_cast<unsigned int>(ball_detections.size()));
    unsigned int min_buffer_size =
        std::min(MIN_BUFFER_SIZE, static_cast<unsigned int>(ball_detections.size()));
    double buffer_size_diff = max_buffer_size - min_buffer_size;

    std::optional<BallVelocityEstimate> velocity_estimate =
        estimateBallVelocity(ball_detections);
    if (!velocity_estimate)
    {
        return std::nullopt;
    }
    // Use the average of the min and max velocity magnitudes in the buffer. We use this
    // rather than the average so we can quickly respond to drastic changes in the ball
    // velocity, such as when the ball goes from being stationary to moving quickly (like
    // when it's kicked). If the buffer is large, then it will take more time for the mean
    // speed to increase enough to start shrinking the buffer. However, the average of the
    // min and max values will immediately increase if the ball starts moving, so the
    // buffer can start shrinking more quickly and increase the filter response time to
    // these sorts of changes.
    double min_max_magnitude_average = velocity_estimate->min_max_magnitude_average;

    // Between the min and max velocity magnitudes, we linearly scale the size of the
    // buffer
    double linear_offset =
        MIN_BUFFER_SIZE_VELOCITY_MAGNITUDE + (buffer_size_velocity_magnitude_diff / 2);
    double linear_scaling_factor = linear(min_max_magnitude_average, linear_offset,
                                          buffer_size_velocity_magnitude_diff);
    int buffer_size =
        max_buffer_size -
        static_cast<unsigned int>(std::floor(linear_scaling_factor * buffer_size_diff));

    return static_cast<size_t>(buffer_size);
}

BallFilter::LinearRegressionResults BallFilter::calculateLineOfBestFit(
    boost::circular_buffer<BallDetection> ball_detections)
{
    if (ball_detections.size() < 2)
    {
        throw std::invalid_argument("At least 2 elements required for linear regression");
    }

    auto x_vs_y_regression = calculateLinearRegression(ball_detections);

    // Linear regression cannot fit a vertical line. To get around this, we fit two lines,
    // one with x and y swapped, so any vertical line becomes horizontal. Then we take the
    // line of the two that fit the best.
    boost::circular_buffer<BallDetection> swapped_ball_detections = ball_detections;
    for (auto &detection : swapped_ball_detections)
    {
        detection.position = Point(detection.position.y(), detection.position.x());
    }
    auto y_vs_x_regression = calculateLinearRegression(swapped_ball_detections);
    // Because we swapped the coordinates of the input, we have to swap the coordinates of
    // the output to get back to our expected coordinate space
    y_vs_x_regression.regression_line.swapXY();

    // We use the regression from above with the least error
    if (x_vs_y_regression.regression_error < y_vs_x_regression.regression_error)
    {
        return x_vs_y_regression;
    }
    else
    {
        return y_vs_x_regression;
    }
}

BallFilter::LinearRegressionResults BallFilter::calculateLinearRegression(
    boost::circular_buffer<BallDetection> ball_detections)
{
    if (ball_detections.size() < 2)
    {
        throw std::invalid_argument("At least 2 elements required for linear regression");
    }

    // Sort the detections in increasing order before processing. This places the oldest
    // detections (smallest timestamp) at the front of the buffer, and the most recent
    // detections (with the largest timestamp) at the end of the buffer
    std::sort(ball_detections.begin(), ball_detections.end());

    // Construct matrix A and vector b for linear regression. The first column of A
    // contains the bias variable, and the second column contains the x coordinates of the
    // ball. Vector b contains the y coordinates of the ball.
    Eigen::MatrixXf A(ball_detections.size(), 2);
    Eigen::VectorXf b(ball_detections.size());
    for (unsigned i = 0; i < ball_detections.size(); i++)
    {
        // This extra column of 1's is the bias variable, so that we can regress with a
        // y-intercept
        A(i, 0) = 1.0;
        A(i, 1) = static_cast<float>(ball_detections.at(i).position.x());

        b(i) = static_cast<float>(ball_detections.at(i).position.y());
    }

    // Perform linear regression to find the line of best fit through the ball positions.
    // This is solving the formula Ax = b, where x is the vector we want to solve for.
    Eigen::Vector2f regression_vector =
        A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    // How to calculate the error is from
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    double regression_error = std::numeric_limits<double>::max();

    if ((A * regression_vector - b).norm() == 0 && b.norm() == 0)
    {
        regression_error = 0;
    }
    if (b.norm() != 0)
    {
        regression_error =
            (A * regression_vector - b).norm() / (b.norm());  // norm() is L2 norm
    }

    // Find 2 points on the regression line that we solved for, and use this to construct
    // our own Line class
    Eigen::Vector2f p1_vec(1, 0);
    Point p1(0, p1_vec.dot(regression_vector));
    Eigen::Vector2f p2_vec(1, 1);
    Point p2(1, p2_vec.dot(regression_vector));
    Line regression_line = Line(p1, p2);

    LinearRegressionResults results({regression_line, regression_error});

    return results;
}

Point BallFilter::estimateBallPosition(
    boost::circular_buffer<BallDetection> ball_detections, const Line &regression_line)
{
    if (ball_detections.empty())
    {
        throw std::invalid_argument(
            "Non-empty buffer required to estimate ball position");
    }

    // Take the position of the most recent ball position and project it onto the line of
    // best fit. We do this because we assume the ball must be travelling along its
    // velocity vector (the line), and this allows us to return more stable position
    // values since the line of best fit is less likely to fluctuate compared to the raw
    // position of a ball detection
    BallDetection latest_ball_detection = ball_detections.front();
    return closestPoint(latest_ball_detection.position, regression_line);
}

std::optional<BallFilter::BallVelocityEstimate> BallFilter::estimateBallVelocity(
    boost::circular_buffer<BallDetection> ball_detections,
    const std::optional<Line> &ball_regression_line)
{
    // Sort the detections in increasing order before processing. This places the oldest
    // detections (smallest timestamp) at the front of the buffer, and the most recent
    // detections (with the largest timestamp) at the end of the buffer
    std::sort(ball_detections.begin(), ball_detections.end());

    std::vector<Vector> ball_velocities;
    std::vector<double> ball_velocity_magnitudes;
    for (unsigned i = 1; i < ball_detections.size(); i++)
    {
        for (unsigned j = i; j < ball_detections.size(); j++)
        {
            BallDetection previous_detection = ball_detections.at(i - 1);
            BallDetection current_detection  = ball_detections.at(j);

            Duration time_diff =
                current_detection.timestamp - previous_detection.timestamp;
            // Avoid division by 0. If we have adjacent detections with the same timestamp
            // the velocity cannot be calculated
            if (time_diff.toSeconds() == 0)
            {
                continue;
            }

            // Project the detection positions onto the regression line if it was provided
            Point current_position;
            Point previous_position;
            if (ball_regression_line)
            {
                current_position  = closestPoint(current_detection.position,
                                                ball_regression_line.value());
                previous_position = closestPoint(previous_detection.position,
                                                 ball_regression_line.value());
            }
            else
            {
                current_position  = current_detection.position;
                previous_position = previous_detection.position;
            }
            Vector velocity_vector    = current_position - previous_position;
            double velocity_magnitude = velocity_vector.length() / time_diff.toSeconds();
            Vector velocity           = velocity_vector.normalize(velocity_magnitude);

            ball_velocity_magnitudes.emplace_back(velocity_magnitude);
            ball_velocities.emplace_back(velocity);
        }
    }

    if (ball_velocities.empty() || ball_velocity_magnitudes.empty())
    {
        return std::nullopt;
    }

    double velocity_magnitude_sum = 0;
    for (const auto &velocity_magnitude : ball_velocity_magnitudes)
    {
        velocity_magnitude_sum += velocity_magnitude;
    }
    double average_velocity_magnitude =
        velocity_magnitude_sum / static_cast<double>(ball_velocity_magnitudes.size());
    double velocity_magnitude_max = *std::max_element(ball_velocity_magnitudes.begin(),
                                                      ball_velocity_magnitudes.end());
    double velocity_magnitude_min = *std::min_element(ball_velocity_magnitudes.begin(),
                                                      ball_velocity_magnitudes.end());
    double min_max_average = (velocity_magnitude_min + velocity_magnitude_max) / 2.0;

    Vector velocity_vector_sum = Vector(0, 0);
    for (const auto &velocity : ball_velocities)
    {
        velocity_vector_sum += velocity;
    }
    Vector average_velocity = velocity_vector_sum.normalize(average_velocity_magnitude);

    BallVelocityEstimate velocity_data(
        {average_velocity, average_velocity_magnitude, min_max_average});

    return velocity_data;
}
