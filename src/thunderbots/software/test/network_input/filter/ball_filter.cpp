/**
 * Tests for the Ball Filter
 */

#include "network_input/filter/ball_filter.h"
#include "test/test_util/test_util.h"
#include <random>

#include <gtest/gtest.h>
#include "geom/segment.h"
#include <limits>

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

    void testFilterAlongLineSegment(const Timestamp& start_time, const Segment& ball_path, double ball_velocity_magnitude, double ball_position_variance,
            double time_step_variance, double expected_position_tolerance, Angle expected_velocity_angle_tolerance,
            double expected_velocity_magnitude_tolerance, std::optional<int> num_iterations=std::nullopt) {
        // Calculate how many simulation steps to take, given the ball's velocity and the
        // time step in order for the ball to reach the end of the given segment. We
        // round up using the ceil so that we don't fall short of the end of the segment
        Duration duration_to_reach_end_of_segment;
        int num_steps = 0;
        if(num_iterations) {
            // Make the duration to reach the end of the segment very large so it is not
            // the limiting factor
            duration_to_reach_end_of_segment = Duration::fromSeconds(std::numeric_limits<int>::max());
            num_steps = *num_iterations;
        }else {
            duration_to_reach_end_of_segment = Duration::fromSeconds(ball_path.length() / ball_velocity_magnitude);
            num_steps = std::ceil(duration_to_reach_end_of_segment.getSeconds() / time_step.getSeconds());
        }

        Vector ball_velocity = ball_path.toVector().norm(ball_velocity_magnitude);

        std::normal_distribution<double> position_noise_distribution(0, ball_position_variance);
        std::normal_distribution<double> time_step_noise_distribution(0, time_step_variance);

        for(int i = 0; i < num_steps; i++) {
            // Generate the noise that will be added to the position and time step to
            // simulate imperfect data
            Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
            Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));

            // Calculate the "correct" ball position
            Timestamp current_timestamp = start_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
            current_timestamp = std::min(current_timestamp, start_time + duration_to_reach_end_of_segment);
            Duration time_diff = current_timestamp - start_time;
            Point current_ball_position = ball_path.getSegStart() + ball_velocity.norm(ball_velocity.len() * time_diff.getSeconds());

            // Apply noise to the position to simulate measurement noise
            Point ball_position_with_noise = current_ball_position + position_noise;

            // Create the detection that would have been seen by the vision system
            std::vector<SSLBallDetection> ball_detections = {
                    SSLBallDetection{ball_position_with_noise, current_timestamp}
            };

            // Get the filtered result given the new detection information
            auto filtered_ball = ball_filter.getFilteredData(ball_detections, field);

            if(!filtered_ball) {
                // TODO: You are here. Why is this failing on iteration 1????
                volatile int q = 0;
            }
            ASSERT_TRUE(filtered_ball);
            double dist = (filtered_ball->position() - current_ball_position).len();
            bool pclose = filtered_ball->position().isClose(current_ball_position, expected_position_tolerance);
            EXPECT_TRUE(pclose);
            // Only check the velocity once we have more than 1 data entry in the filter
            // since the filter can't return a realistic velocity with only a single detection
//            if(i > 0) {
//                double od = std::fabs(filtered_ball->velocity().orientation().minDiff(ball_velocity.orientation()).toDegrees());
//                EXPECT_LE(od, expected_velocity_angle_tolerance.toDegrees());
////                EXPECT_NEAR(filtered_ball->velocity().orientation().angleMod().toDegrees(), ball_velocity.orientation().angleMod().toDegrees(), expected_velocity_angle_tolerance.toDegrees());
//                double vd = std::fabs(filtered_ball->velocity().len() - ball_velocity.len());
//                EXPECT_NEAR(filtered_ball->velocity().len(), ball_velocity.len(), expected_velocity_magnitude_tolerance);
//            }
//
//            double td = (filtered_ball->lastUpdateTimestamp() - current_timestamp).getSeconds();
//            EXPECT_GE(filtered_ball->lastUpdateTimestamp(), current_timestamp);
        }
    }

//    Timestamp starting_time;
    Field field = ::Test::TestUtil::createSSLDivBField();
    BallFilter ball_filter;
    Duration time_step;
    std::mt19937 random_generator;
    Timestamp current_timestamp;
};




TEST_F(BallFilterTest, ball_sitting_still_with_low_noise) {
    Segment ball_path = Segment(Point(0, 0), Point(0, 0));
    double ball_velocity_magnitude = 0;

    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
    testFilterAlongLineSegment(current_timestamp, ball_path, ball_velocity_magnitude, 0.001, 0.001, 0.005, Angle::ofDegrees(359), 0.075, 200);
}

TEST_F(BallFilterTest, ball_sitting_still_with_moderate_noise) {
    Segment ball_path = Segment(Point(0, 0), Point(0, 0));
    double ball_velocity_magnitude = 0;

    // When the ball is sitting still, the velocity could be any direction so we do
    // not use a strict tolerance for this test. We only really care about the
    // magnitude of the velocity to make sure it's small enough
//    testFilterAlongLineSegment(current_timestamp, ball_path, ball_velocity_magnitude, 0.008, 0.01, 0.016, Angle::ofDegrees(359), 0.1, 10);
    testFilterAlongLineSegment(current_timestamp, ball_path, ball_velocity_magnitude, 0.008, 0.01, 0.016, Angle::ofDegrees(359), 0.1, 200);
}
//    Point ball_starting_position = Point(-2, -1);
//    Vector velocity = Vector(0, 0);
//
//    // Variance of 8mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.008);
//    // Time noise of 10ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.01);
//
//    for(int i = 0; i < 200; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        EXPECT_TRUE(ball->velocity().isClose(velocity, 0.01));
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_no_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(5);
//
//    // Variance of 0mm
//    std::normal_distribution<double> position_noise_distribution(0, 0);
//    // Time noise of 0ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << ball->velocity().len() << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_TRUE(ball->velocity().isClose(velocity, 0.01));
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_small_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(5);
//
//    // Variance of 1mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.001);
//    // Time noise of 1ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.001);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << (ball->velocity() - velocity).len() << ", " << (ball->velocity().orientation() - velocity.orientation()).toDegrees() << ", " << std::fabs(ball->velocity().len() - velocity.len()) << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_NEAR(ball->velocity().orientation().toDegrees(), velocity.orientation().toDegrees(), 0.9);
//            EXPECT_NEAR(ball->velocity().len(), velocity.len(), 0.1);
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_moderate_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(5);
//
//    // Variance of 1mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.008);
//    // Time noise of 1ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.01);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << (ball->velocity() - velocity).len() << ", " << (ball->velocity().orientation() - velocity.orientation()).toDegrees() << ", " << std::fabs(ball->velocity().len() - velocity.len()) << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_NEAR(ball->velocity().orientation().toDegrees(), velocity.orientation().toDegrees(), 0.9);
//            EXPECT_NEAR(ball->velocity().len(), velocity.len(), 0.1);
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_small_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(0.4);
//
//    // Variance of 1mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.001);
//    // Time noise of 1ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.001);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << (ball->velocity() - velocity).len() << ", " << (ball->velocity().orientation() - velocity.orientation()).toDegrees() << ", " << std::fabs(ball->velocity().len() - velocity.len()) << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_NEAR(ball->velocity().orientation().toDegrees(), velocity.orientation().toDegrees(), 0.9);
//            EXPECT_NEAR(ball->velocity().len(), velocity.len(), 0.1);
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_slow_in_a_straight_line_with_moderate_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(0.4);
//
//    // Variance of 1mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.008);
//    // Time noise of 1ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.01);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << (ball->velocity() - velocity).len() << ", " << (ball->velocity().orientation() - velocity.orientation()).toDegrees() << ", " << std::fabs(ball->velocity().len() - velocity.len()) << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_NEAR(ball->velocity().orientation().toDegrees(), velocity.orientation().toDegrees(), 0.9);
//            EXPECT_NEAR(ball->velocity().len(), velocity.len(), 0.1);
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_a_sharp_bounce_and_small_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(5);
//
//    // Variance of 1mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.008);
//    // Time noise of 1ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.01);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << (ball->velocity() - velocity).len() << ", " << (ball->velocity().orientation() - velocity.orientation()).toDegrees() << ", " << std::fabs(ball->velocity().len() - velocity.len()) << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_NEAR(ball->velocity().orientation().toDegrees(), velocity.orientation().toDegrees(), 0.9);
//            EXPECT_NEAR(ball->velocity().len(), velocity.len(), 0.1);
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
//TEST_F(BallFilterTest, ball_moving_fast_in_a_straight_line_with_moderate_noise_in_data) {
//    // The ball is moving from one corner of the field to the other at 5m/s
//    Point ball_starting_position = field.friendlyCornerNeg();
//    Vector velocity = (field.enemyCornerPos() - field.friendlyCornerNeg()).norm(5);
//
//    // Variance of 1mm
//    std::normal_distribution<double> position_noise_distribution(0, 0.008);
//    // Time noise of 1ms
//    std::normal_distribution<double> time_step_noise_distribution(0, 0.01);
//
//    for(int i = 0; i < 100; i++) {
//        // Generate the noise that will be added to the position and time step to simulate
//        // imperfect data
//        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
//        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
//
//        // Calculate the "correct" ball position
//        Timestamp current_timestamp = starting_time + Duration::fromSeconds(i * time_step.getSeconds() + time_step_noise.getSeconds());
//        Duration time_diff = current_timestamp - starting_time;
//        Point current_ball_position = ball_starting_position + velocity.norm(velocity.len() * time_diff.getSeconds());
//
//        // Apply noise to the position
//        Point ball_position_with_noise = current_ball_position + position_noise;
//
//        std::vector<SSLBallDetection> ball_detections = {
//                SSLBallDetection{ball_position_with_noise, current_timestamp}
//        };
//        auto ball = ball_filter.getFilteredData(ball_detections, field);
//
//        std::cout << ball_position_with_noise << ", " << (ball_position_with_noise - current_ball_position).len()  << ", " << ball->velocity() << ", " << (ball->velocity() - velocity).len() << ", " << (ball->velocity().orientation() - velocity.orientation()).toDegrees() << ", " << std::fabs(ball->velocity().len() - velocity.len()) << std::endl;
//
//        ASSERT_TRUE(ball);
//        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.005));
//        // Only check the velocity once we have more than 1 data entry in the filter
//        // since the filter can't return a realistic velocity with only a single detection
//        if(i > 0) {
//            EXPECT_NEAR(ball->velocity().orientation().toDegrees(), velocity.orientation().toDegrees(), 0.9);
//            EXPECT_NEAR(ball->velocity().len(), velocity.len(), 0.1);
//        }
//        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
//    }
//}
//
////TEST_F(BallFilterTest, ball_moving_in_a_straight_line_with_no_noise_in_data) {
////    double ball_velocity_magnitude = 4.0;
////    Point ball_starting_position = Point(-2, -1);
////    std::normal_distribution<double> position_noise_distribution(0, 0);
////    std::normal_distribution<double> time_step_noise_distribution(0, 0);
////
////    Vector velocity = Vector(1, 1).norm(ball_velocity_magnitude);
////    Vector position_increment = velocity.norm(velocity.len() * time_step.getSeconds());
////
////    Point current_ball_position = ball_starting_position;
////    Timestamp current_timestamp = starting_time;
////
////    for(int i = 0; i < 100; i++) {
////        // Generate the noise that will be added to the position and time step to simulate
////        // imperfect data
////        Point position_noise(position_noise_distribution(random_generator), position_noise_distribution(random_generator));
////        Duration time_step_noise = Duration::fromSeconds(time_step_noise_distribution(random_generator));
////
////        // Update the "correct" state of the ball
////        current_ball_position = current_ball_position + position_increment;
////        current_timestamp = current_timestamp + Duration::fromSeconds(time_step.getSeconds());
////
////        // Apply noise to the current state
////        Point ball_position_with_noise = current_ball_position + position_noise;
////        Timestamp current_timestamp_with_noise = current_timestamp + time_step_noise;
////
////        std::vector<SSLBallDetection> ball_detections = {
////                SSLBallDetection{ball_position_with_noise, current_timestamp_with_noise}
////        };
////        auto ball = ball_filter.getFilteredData(ball_detections, field);
////
////        ASSERT_TRUE(ball);
////        EXPECT_TRUE(ball->position().isClose(current_ball_position, 0.01));
////        EXPECT_TRUE(ball->velocity().isClose(velocity, 0.01));
////        EXPECT_EQ(ball->lastUpdateTimestamp(), current_timestamp);
////    }
////}
////

