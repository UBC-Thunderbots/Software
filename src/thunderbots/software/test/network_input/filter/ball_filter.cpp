/**
 * Tests for the Ball Filter
 */

#include "network_input/filter/ball_filter.h"
#include "test/test_util/test_util.h"
#include "geom/ray.h"
#include <random>

#include <gtest/gtest.h>
#include "geom/segment.h"
#include <limits>
#include <exception>
#include "../shared/constants.h"

class BallFilterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize the time
        current_timestamp = Timestamp::fromSeconds(123);
        field = ::Test::TestUtil::createSSLDivBField();
        ball_filter = BallFilter();
        time_step = Duration::fromSeconds(1.0 / 60.0);
        // Use a constant seed to results are deterministic
        random_generator.seed(1);
    }

    void testFilterAlongBallTrajectory(const Timestamp& start_time, const Ray& ball_trajectory, double ball_velocity_magnitude, double ball_position_variance,
                                    double time_step_variance, double expected_position_tolerance, Angle expected_velocity_angle_tolerance,
                                    double expected_velocity_magnitude_tolerance, unsigned int num_iterations, unsigned int num_steps_to_ignore) {
        Point ball_starting_position = ball_trajectory.getRayStart();
        Vector ball_velocity = ball_trajectory.toVector().norm(ball_velocity_magnitude);
        Duration max_ball_travel_duration = Duration::fromSeconds(std::numeric_limits<double>::max());

        testFilterHelper(start_time, ball_starting_position, ball_velocity,
                ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolerance,
                expected_velocity_magnitude_tolerance, num_iterations, max_ball_travel_duration, num_steps_to_ignore);
    }

    void testFilterAlongLineSegment(const Timestamp& start_time, const Segment& ball_path, double ball_velocity_magnitude, double ball_position_variance,
            double time_step_variance, double expected_position_tolerance, Angle expected_velocity_angle_tolerance,
            double expected_velocity_magnitude_tolerance, unsigned int num_steps_to_ignore) {
        Point ball_starting_position = ball_path.getSegStart();
        Vector ball_velocity = ball_path.toVector().norm(ball_velocity_magnitude);
        // Check for division by 0
        if(ball_velocity_magnitude == 0) {
            throw std::system_error();
        }
        // Calculate how many simulation steps to take, given the ball's velocity and the
        // time step in order for the ball to reach the end of the given segment.
        Duration max_ball_travel_duration = Duration::fromSeconds(ball_path.length() / ball_velocity_magnitude);
        int num_iterations = std::round(max_ball_travel_duration.getSeconds() / time_step.getSeconds());

        testFilterHelper(start_time, ball_starting_position, ball_velocity,
                         ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolerance,
                         expected_velocity_magnitude_tolerance, num_iterations, max_ball_travel_duration, num_steps_to_ignore);
    }

    void testFilterHelper(const Timestamp& start_time,
                          const Point& ball_starting_position,
                          const Vector& ball_velocity,
                          double ball_position_variance,
                          double time_step_variance,
                          double expected_position_tolerance,
                          const Angle& expected_velocity_angle_tolerance,
                          double expected_velocity_magnitude_tolerance,
                          unsigned int num_iterations,
                          const Duration& max_ball_travel_duration,
                          unsigned int num_steps_to_ignore = 0) {
        std::normal_distribution<double> position_noise_distribution(0, ball_position_variance);
        std::normal_distribution<double> time_step_noise_distribution(0, time_step_variance);

        for(int i = 0; i < num_iterations; i++) {
            // Generate the noise that will be added to the position and time step to
            // simulate imperfect data
            Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
            Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));

            // Calculate the current time and add noise
            // We make sure the applied noise doesn't cause the timestamp to be larger than expected on the last
            // iteration so the ball's final position is close to what's expected by the caller of this function
            Timestamp current_timestamp_with_noise = start_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
            this->current_timestamp = std::min(current_timestamp_with_noise, start_time + max_ball_travel_duration);

            // Take the time difference from the start time and calculate the ball's current position based on it's
            // velocity and the elapsed time
            Duration time_diff = current_timestamp - start_time;
            Point current_ball_position = ball_starting_position + ball_velocity.norm(ball_velocity.len() * time_diff.getSeconds());

            // Apply noise to the ball's position to simulate measurement noise
            Point ball_position_with_noise = current_ball_position + position_noise;

            // Create the detection that would have been seen by the vision system
            std::vector<SSLBallDetection> ball_detections = {
                    SSLBallDetection{ball_position_with_noise, current_timestamp}
            };

            // Get the filtered result given the new detection information
            auto filtered_ball = ball_filter.getFilteredData(ball_detections, field);

            if(i < num_steps_to_ignore) {
                continue;
            }

            ASSERT_TRUE(filtered_ball);
            double ball_position_difference = (filtered_ball->position() - current_ball_position).len();
            EXPECT_LT(ball_position_difference, expected_position_tolerance);
            // Only check the velocity once we have more than 1 data entry in the filter
            // since the filter can't return a realistic velocity with only a single detection
            if(i > 0) {
                // Check the direction of the velocity
                double velocity_orientation_difference = std::fabs(filtered_ball->velocity().orientation().minDiff(ball_velocity.orientation()).toDegrees());
                EXPECT_LE(velocity_orientation_difference, expected_velocity_angle_tolerance.toDegrees());
                // Check the magnitude of the velocity
                double velocity_magnitude_difference = std::fabs(filtered_ball->velocity().len() - ball_velocity.len());
                EXPECT_LE(velocity_magnitude_difference, expected_velocity_magnitude_tolerance);
            }

            // Make sure the timestamps are always increasing
            EXPECT_GE(filtered_ball->lastUpdateTimestamp(), current_timestamp);
        }
    }

    Field field = ::Test::TestUtil::createSSLDivBField();
    BallFilter ball_filter;
    Duration time_step;
    std::mt19937 random_generator;
    Timestamp current_timestamp;
};

TEST_F(BallFilterTest, ball_sitting_still_with_low_noise) {
    Ray ball_trajectory = Ray(Point(0, 0), Vector(0, 0));
    double ball_velocity_magnitude = 0;
    double ball_position_variance = 0.001;
    double time_step_variance = 0.001;
    double expected_position_tolerance = 0.005;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::threeQuarter();
    double expected_velocity_magnitude_tolerance = 0.075;
    int num_simulation_steps = 200;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongBallTrajectory(start_time, ball_trajectory, ball_velocity_magnitude, ball_position_variance,
            time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace,
            expected_velocity_magnitude_tolerance, num_simulation_steps, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_sitting_still_with_moderate_noise) {
    Ray ball_trajectory = Ray(Point(0, 0), Vector(0, 0));
    double ball_velocity_magnitude = 0;
    double ball_position_variance = 0.003;
    double time_step_variance = 0.003;
    double expected_position_tolerance = 0.016;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::threeQuarter();
    double expected_velocity_magnitude_tolerance = 1.0;
    int num_simulation_steps = 10;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongBallTrajectory(start_time, ball_trajectory, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_simulation_steps, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_no_noise_in_data) {
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude = 0.31;
    double ball_position_variance = 0;
    double time_step_variance = 0;
    double expected_position_tolerance = 0.001;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::ofDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongLineSegment(start_time, ball_path, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_small_noise_in_data) {
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude = 0.3;
    double ball_position_variance = 0.001;
    double time_step_variance = 0.001;
    double expected_position_tolerance = 0.005;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::ofDegrees(10);
    double expected_velocity_magnitude_tolerance = 0.1;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongLineSegment(start_time, ball_path, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_medium_noise_in_data) {
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude = 0.3;
    double ball_position_variance = 0.005;
    double time_step_variance = 0.001;
    double expected_position_tolerance = 0.022;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::ofDegrees(1);
    double expected_velocity_magnitude_tolerance = 0.08;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongLineSegment(start_time, ball_path, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_no_noise_in_data) {
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude = 6.2;
    double ball_position_variance = 0;
    double time_step_variance = 0;
    double expected_position_tolerance = 0.001;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::ofDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongLineSegment(start_time, ball_path, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_small_noise_in_data) {
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude = 5.72;
    double ball_position_variance = 0.001;
    double time_step_variance = 0.001;
    double expected_position_tolerance = 0.003;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::ofDegrees(1);
    double expected_velocity_magnitude_tolerance = 0.08;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongLineSegment(start_time, ball_path, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_medium_noise_in_data) {
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude = 5.04;
    double ball_position_variance = 0.005;
    double time_step_variance = 0.001;
    double expected_position_tolerance = 0.015;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace = Angle::ofDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.00;
    int num_steps_to_ignore = 5;
    Timestamp start_time = current_timestamp;

    testFilterAlongLineSegment(start_time, ball_path, ball_velocity_magnitude, ball_position_variance, time_step_variance, expected_position_tolerance, expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance, num_steps_to_ignore);
}
