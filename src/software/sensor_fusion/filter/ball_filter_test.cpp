#include "software/sensor_fusion/filter/ball_filter.h"

#include <gtest/gtest.h>

#include <exception>
#include <limits>
#include <random>

#include "shared/constants.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/world/field.h"

class BallFilterTest : public ::testing::Test
{
   protected:
    BallFilterTest()
        : field(Field::createSSLDivisionBField()),
          ball_filter(),
          current_timestamp(Timestamp::fromSeconds(123)),
          time_step(Duration::fromSeconds(1.0 / 60.0))
    {
    }

    void SetUp() override
    {
        // Use a constant seed to results are deterministic
        random_generator.seed(1);
    }

    /**
     * Simulates the ball moving in a certain direction with a specified amount of noise
     * in the measurements, and verifies that the filtered ball data is within the given
     * tolerances of the real ball.
     *
     * @param start_time The time at which the ball starts moving
     * @param ball_trajectory A ray indicating the ball's starting position and direction
     * of travel
     * @param ball_velocity_magnitude How fast the ball is moving
     * @param ball_position_variance The variance in the noise the ball's position will be
     * sampled with
     * @param time_step_variance The variance in the timestep the ball will be sampled
     * with
     * @param expected_position_tolerance How close to the real position the filtered ball
     * position should be
     * @param expected_velocity_angle_tolerance How close to the real velocity angle the
     * filtered ball velocity angle should be
     * @param expected_velocity_magnitude_tolerance How close to the real velocity
     * magnitude the filtered ball velocity magnitude should be
     * @param num_iterations For how many iterations the simulation should run
     * @param num_steps_to_ignore The number of iterations at the beginning of the
     * simulation to not check tolerances
     */
    void testFilterAlongBallTrajectory(
        const Timestamp& start_time, const Ray& ball_trajectory,
        double ball_velocity_magnitude, double ball_position_variance,
        double time_step_variance, double expected_position_tolerance,
        Angle expected_velocity_angle_tolerance,
        double expected_velocity_magnitude_tolerance, unsigned int num_iterations,
        unsigned int num_steps_to_ignore)
    {
        Point ball_starting_position = ball_trajectory.getStart();
        Vector ball_velocity =
            ball_trajectory.toUnitVector().normalize(ball_velocity_magnitude);
        Duration max_ball_travel_duration =
            Duration::fromSeconds(std::numeric_limits<double>::max());

        testFilterHelper(start_time, ball_starting_position, ball_velocity,
                         ball_position_variance, time_step_variance,
                         expected_position_tolerance, expected_velocity_angle_tolerance,
                         expected_velocity_magnitude_tolerance, num_iterations,
                         max_ball_travel_duration, num_steps_to_ignore);
    }

    /**
     * Simulates the ball moving in a line, starting at one point and ending at another
     * with a specified amount of noise in the measurements, and verifies the filtered
     * ball data is within the given tolerances of the real ball.
     *
     * @param start_time The time at which the ball starts moving
     * @param ball_path The segment along which the ball will travel
     * @param ball_velocity_magnitude How fast the ball is moving
     * @param ball_position_variance The variance in the noise the ball's position will be
     * sampled with
     * @param time_step_variance The variance in the timestep the ball will be sampled
     * with
     * @param expected_position_tolerance How close to the real position the filtered ball
     * position should be
     * @param expected_velocity_angle_tolerance How close to the real velocity angle the
     * filtered ball velocity angle should be
     * @param expected_velocity_magnitude_tolerance How close to the real velocity
     * magnitude the filtered ball velocity magnitude should be
     * @param num_steps_to_ignore The number of iterations at the beginning of the
     * simulation to not check tolerances
     */
    void testFilterAlongLineSegment(const Timestamp& start_time, const Segment& ball_path,
                                    double ball_velocity_magnitude,
                                    double ball_position_variance,
                                    double time_step_variance,
                                    double expected_position_tolerance,
                                    Angle expected_velocity_angle_tolerance,
                                    double expected_velocity_magnitude_tolerance,
                                    unsigned int num_steps_to_ignore)
    {
        Point ball_starting_position = ball_path.getStart();
        Vector ball_velocity = ball_path.toVector().normalize(ball_velocity_magnitude);
        // Check for division by 0
        if (ball_velocity_magnitude == 0)
        {
            throw std::invalid_argument(
                "ball_velocity_magnitude with value of 0 given, this will result in division by 0");
        }
        // Calculate how many simulation steps to take, given the ball's velocity and the
        // time step in order for the ball to reach the end of the given segment.
        Duration max_ball_travel_duration =
            Duration::fromSeconds(ball_path.length() / ball_velocity_magnitude);
        int num_iterations = static_cast<int>(
            std::round(max_ball_travel_duration.toSeconds() / time_step.toSeconds()));

        testFilterHelper(start_time, ball_starting_position, ball_velocity,
                         ball_position_variance, time_step_variance,
                         expected_position_tolerance, expected_velocity_angle_tolerance,
                         expected_velocity_magnitude_tolerance, num_iterations,
                         max_ball_travel_duration, num_steps_to_ignore);
    }

    /**
     * A helper function that simulates the ball moving in a straight line. The ball's
     * position will be sampled with a given amount of noise / variance, and the filtered
     * data will be checked to verify it is within the specified tolerances compared to
     * the real ball
     * @param start_time The time at which the ball starts moving
     * @param ball_starting_position The position the ball starts from
     * @param ball_velocity The velocity of the ball
     * @param ball_position_variance The variance in the noise the ball's position will be
     * sampled with
     * @param time_step_variance The variance in the timestep the ball will be sampled
     * with
     * @param expected_position_tolerance How close to the real position the filtered ball
     * position should be
     * @param expected_velocity_angle_tolerance How close to the real velocity angle the
     * filtered ball velocity angle should be
     * @param expected_velocity_magnitude_tolerance How close to the real velocity
     * magnitude the filtered ball velocity magnitude should be
     * @param num_iterations For how many iterations the simulation should run
     * @param max_ball_travel_duration The maximum duration for which the ball may travel
     * / be simulated
     * @param num_steps_to_ignore The number of iterations at the beginning of the
     * simulation to not check tolerances
     */
    void testFilterHelper(const Timestamp& start_time,
                          const Point& ball_starting_position,
                          const Vector& ball_velocity, double ball_position_variance,
                          double time_step_variance, double expected_position_tolerance,
                          const Angle& expected_velocity_angle_tolerance,
                          double expected_velocity_magnitude_tolerance,
                          unsigned int num_iterations,
                          const Duration& max_ball_travel_duration,
                          unsigned int num_steps_to_ignore = 0)
    {
        // Create the distributions we use to generate noise when sampling the ball
        std::normal_distribution<double> position_noise_distribution(
            0, ball_position_variance);
        std::normal_distribution<double> time_step_noise_distribution(0,
                                                                      time_step_variance);

        for (unsigned i = 0; i < num_iterations; i++)
        {
            // Generate the noise that will be added to the position and time step to
            // simulate imperfect data
            Vector position_noise(position_noise_distribution(random_generator),
                                  position_noise_distribution(random_generator));
            Duration time_step_noise =
                Duration::fromSeconds(time_step_noise_distribution(random_generator));

            // Calculate the current time and add noise
            // We make sure the applied noise doesn't cause the timestamp to be larger
            // than expected on the last iteration so the ball's final position is close
            // to what's expected by the caller of this function
            Timestamp current_timestamp_with_noise =
                start_time + Duration::fromSeconds(i * time_step.toSeconds() +
                                                   time_step_noise.toSeconds());
            this->current_timestamp = std::min(current_timestamp_with_noise,
                                               start_time + max_ball_travel_duration);

            // Take the time difference from the start time and calculate the ball's
            // current position based on it's velocity and the elapsed time
            Duration time_diff = current_timestamp - start_time;
            Point current_ball_position =
                ball_starting_position +
                ball_velocity.normalize(ball_velocity.length() * time_diff.toSeconds());

            // Apply noise to the ball's position to simulate measurement noise
            Point ball_position_with_noise = current_ball_position + position_noise;

            // Create the detection that would have been seen by the vision system
            std::vector<BallDetection> ball_detections = {
                BallDetection{ball_position_with_noise, BALL_DISTANCE_FROM_GROUND,
                              current_timestamp, 0.9}};

            // Get the filtered result given the new detection information
            auto filtered_ball =
                ball_filter.estimateBallState(ball_detections, field.fieldBoundary());
            if (i < num_steps_to_ignore)
            {
                continue;
            }

            ASSERT_TRUE(filtered_ball);
            double ball_position_difference =
                (filtered_ball->position() - current_ball_position).length();
            EXPECT_LT(ball_position_difference, expected_position_tolerance);
            // Only check the velocity once we have more than 1 data entry in the filter
            // since the filter can't return a realistic velocity with only a single
            // detection
            if (i > 0)
            {
                // Check the direction of the velocity
                double velocity_orientation_difference =
                    std::fabs(filtered_ball->velocity()
                                  .orientation()
                                  .minDiff(ball_velocity.orientation())
                                  .toDegrees());
                EXPECT_LE(velocity_orientation_difference,
                          expected_velocity_angle_tolerance.toDegrees());
                // Check the magnitude of the velocity
                double velocity_magnitude_difference = std::fabs(
                    filtered_ball->velocity().length() - ball_velocity.length());
                EXPECT_LE(velocity_magnitude_difference,
                          expected_velocity_magnitude_tolerance);
            }

            // Make sure the timestamps are always increasing
            EXPECT_GE(filtered_ball->timestamp(), current_timestamp);
        }
    }

    Field field;
    BallFilter ball_filter;
    Timestamp current_timestamp;
    Duration time_step;
    std::mt19937 random_generator;
    // For these tests, the ball is always on the ground. The filters
    // are not designed for filtering balls in the air
    static constexpr double BALL_DISTANCE_FROM_GROUND = 0.0;
};

TEST_F(BallFilterTest, ball_sitting_still_with_low_noise)
{
    Ray ball_trajectory                = Ray(Point(0, 0), Vector(0, 0));
    double ball_velocity_magnitude     = 0;
    double ball_position_variance      = 0.001;
    double time_step_variance          = 0.001;
    double expected_position_tolerance = 0.005;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace      = Angle::threeQuarter();
    double expected_velocity_magnitude_tolerance = 0.075;
    int num_simulation_steps                     = 200;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongBallTrajectory(
        start_time, ball_trajectory, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_simulation_steps, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_sitting_still_with_moderate_noise)
{
    Ray ball_trajectory                = Ray(Point(0, 0), Vector(0, 0));
    double ball_velocity_magnitude     = 0;
    double ball_position_variance      = 0.003;
    double time_step_variance          = 0.003;
    double expected_position_tolerance = 0.016;
    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    Angle expected_velocity_angle_tolernace      = Angle::threeQuarter();
    double expected_velocity_magnitude_tolerance = 1.0;
    int num_simulation_steps                     = 10;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongBallTrajectory(
        start_time, ball_trajectory, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_simulation_steps, num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_no_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 0.31;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_small_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 0.3;
    double ball_position_variance                = 0.001;
    double time_step_variance                    = 0.001;
    double expected_position_tolerance           = 0.004;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(5.5);
    double expected_velocity_magnitude_tolerance = 0.04;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_medium_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 0.3;
    double ball_position_variance                = 0.003;
    double time_step_variance                    = 0.001;
    double expected_position_tolerance           = 0.011;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(16);
    double expected_velocity_magnitude_tolerance = 0.11;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_no_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 6.2;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_small_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 5.72;
    double ball_position_variance                = 0.001;
    double time_step_variance                    = 0.001;
    double expected_position_tolerance           = 0.003;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.9);
    double expected_velocity_magnitude_tolerance = 0.07;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_medium_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 5.04;
    double ball_position_variance                = 0.003;
    double time_step_variance                    = 0.001;
    double expected_position_tolerance           = 0.008;
    Angle expected_velocity_angle_tolerance      = Angle::fromDegrees(3.0);
    double expected_velocity_magnitude_tolerance = 0.21;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolerance, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest,
       ball_moving_fast_in_a_straight_line_and_then_bouncing_with_no_noise_in_data)
{
    Segment ball_path = Segment(field.friendlyCornerNeg(), field.enemyCornerPos());
    double ball_velocity_magnitude               = 5.04;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.0001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);

    ball_path                   = Segment(field.enemyCornerPos(), field.enemyCornerNeg());
    ball_velocity_magnitude     = 4.8;
    expected_position_tolerance = 0.0001;
    expected_velocity_angle_tolernace     = Angle::fromDegrees(0.01);
    expected_velocity_magnitude_tolerance = 0.01;
    num_steps_to_ignore                   = 5;
    start_time                            = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest,
       ball_moving_fast_in_a_straight_line_and_then_bouncing_with_no_noise_in_data_2)
{
    Segment ball_path =
        Segment(field.friendlyCornerNeg(), field.friendlyHalf().posXPosYCorner());
    double ball_velocity_magnitude               = 5.04;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.0001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.1);
    double expected_velocity_magnitude_tolerance = 0.1;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);

    ball_path = Segment(field.friendlyHalf().posXPosYCorner(), field.enemyCornerNeg());
    ball_velocity_magnitude               = 4.8;
    expected_position_tolerance           = 0.0001;
    expected_velocity_angle_tolernace     = Angle::fromDegrees(0.1);
    expected_velocity_magnitude_tolerance = 0.1;
    num_steps_to_ignore                   = 5;
    start_time                            = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest,
       ball_moving_fast_in_a_straight_line_and_then_bouncing_with_no_noise_in_data_3)
{
    Segment ball_path                            = Segment(Point(-3, -1), Point(0, 0));
    double ball_velocity_magnitude               = 5.04;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.0001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.1);
    double expected_velocity_magnitude_tolerance = 0.1;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);

    ball_path                             = Segment(Point(0, 0), Point(3, 0));
    ball_velocity_magnitude               = 4.8;
    expected_position_tolerance           = 0.0001;
    expected_velocity_angle_tolernace     = Angle::fromDegrees(0.1);
    expected_velocity_magnitude_tolerance = 0.1;
    num_steps_to_ignore                   = 5;
    start_time                            = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_along_x_axis)
{
    Segment ball_path = Segment(field.friendlyGoalCenter(), field.enemyGoalCenter());
    double ball_velocity_magnitude               = 5;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}

TEST_F(BallFilterTest, ball_moving_along_y_axis)
{
    Segment ball_path                            = field.halfwayLine();
    double ball_velocity_magnitude               = 5;
    double ball_position_variance                = 0;
    double time_step_variance                    = 0;
    double expected_position_tolerance           = 0.001;
    Angle expected_velocity_angle_tolernace      = Angle::fromDegrees(0.01);
    double expected_velocity_magnitude_tolerance = 0.01;
    int num_steps_to_ignore                      = 5;
    Timestamp start_time                         = current_timestamp;

    testFilterAlongLineSegment(
        start_time, ball_path, ball_velocity_magnitude, ball_position_variance,
        time_step_variance, expected_position_tolerance,
        expected_velocity_angle_tolernace, expected_velocity_magnitude_tolerance,
        num_steps_to_ignore);
}
