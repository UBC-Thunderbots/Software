#include "software/world/ball.h"

#include <gtest/gtest.h>

#include "proto/message_translation/tbots_protobuf.h"
#include "shared/constants.h"


class BallTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        // An arbitrary fixed point in time.
        // We use this fixed point in time to make the tests deterministic.
        current_time = Timestamp::fromSeconds(123);
        one_hundred_fifty_milliseconds_future =
            current_time + Duration::fromMilliseconds(150);
        half_second_future = current_time + Duration::fromMilliseconds(500);
        one_second_future  = current_time + Duration::fromSeconds(1);
        one_second_past    = current_time - Duration::fromSeconds(1);
    }

    Timestamp current_time;
    Timestamp half_second_future;
    Timestamp one_second_future;
    Timestamp one_hundred_fifty_milliseconds_future;
    Timestamp one_second_past;
};

TEST_F(BallTest, construct_with_no_params)
{
    Ball ball = Ball(Point(), Vector(), current_time);

    EXPECT_EQ(Point(), ball.position());
    EXPECT_EQ(Vector(), ball.velocity());
    EXPECT_EQ(current_time, ball.timestamp());
}

TEST_F(BallTest, construct_with_params)
{
    Ball ball = Ball(Point(1, 2.3), Vector(-0.04, 0.0), current_time);

    EXPECT_EQ(Point(1, 2.3), ball.position());
    EXPECT_EQ(Vector(-0.04, 0.0), ball.velocity());
    EXPECT_EQ(current_time, ball.timestamp());
}

TEST_F(BallTest, construct_with_initial_state)
{
    Ball ball = Ball(BallState(Point(1, 2.3), Vector(-0.04, 0.0)), current_time);

    EXPECT_EQ(Point(1, 2.3), ball.position());
    EXPECT_EQ(Vector(-0.04, 0.0), ball.velocity());
    EXPECT_EQ(current_time, ball.timestamp());
}

TEST_F(BallTest, construct_with_protobuf)
{
    Ball original_ball(Point(1.0, 1.0), Vector(2.0, 2.0), Timestamp::fromSeconds(3.0));
    std::unique_ptr<TbotsProto::Ball> ball_proto = createBallProto(original_ball);
    Ball proto_converted_ball(*ball_proto);

    EXPECT_EQ(original_ball, proto_converted_ball);
}

TEST_F(BallTest, update_state_with_all_params)
{
    Ball ball = Ball(Point(), Vector(), current_time);

    ball.updateState(BallState(Point(-4.23, 1.07), Vector(1, 2)), one_second_future);

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(1, 2), one_second_future), ball);
}

TEST_F(BallTest, update_state_with_new_position_old_velocity)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ball.updateState(BallState(Point(0.01, -99.8), ball.velocity()), current_time);

    EXPECT_EQ(Ball(Point(0.01, -99.8), Vector(1, 2), current_time), ball);
}

TEST_F(BallTest, update_state_with_new_velocity_old_position)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ball.updateState(BallState(ball.position(), Vector(-0.0, -9.433)), current_time);

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(-0.0, -9.433), current_time), ball);
}

TEST_F(BallTest, update_state_with_new_ball)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    BallState ball_updated_state(Point(), Vector(-4.89, 3.1));

    ball.updateState(ball_updated_state, current_time);

    EXPECT_EQ(ball_updated_state, ball.currentState());
    EXPECT_EQ(current_time, ball.timestamp());
}

TEST_F(BallTest, update_state_with_past_timestamp)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ASSERT_THROW(
        ball.updateState(BallState(Point(-4.23, 1.07), Vector(1, 2)), one_second_past),
        std::invalid_argument);
}

TEST_F(BallTest, get_position_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(Point(3, 7), ball.position());
}

TEST_F(BallTest, get_velocity_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(Vector(-4.5, -0.12), ball.velocity());
}

TEST_F(BallTest, equality_operator_compare_ball_with_itself)
{
    Ball ball_0 = Ball(Point(), Vector(), current_time);

    Ball ball_1 = Ball(Point(2, -3), Vector(0, 1), one_hundred_fifty_milliseconds_future);

    EXPECT_EQ(ball_0, ball_0);
    EXPECT_EQ(ball_1, ball_1);
}

TEST_F(BallTest, equality_operator_balls_with_different_positions)
{
    Ball ball_0 = Ball(Point(0.01, -0.0), Vector(), current_time);

    Ball ball_1 = Ball(Point(2, -3), Vector(), current_time);

    EXPECT_NE(ball_0, ball_1);
}

TEST_F(BallTest, equality_operator_balls_with_different_velocities)
{
    Ball ball_0 = Ball(Point(2, -3), Vector(1, 2), current_time);

    Ball ball_1 = Ball(Point(2, -3), Vector(-1, 4.5), current_time);

    EXPECT_NE(ball_0, ball_1);
}

TEST_F(BallTest, equality_operator_balls_with_different_timestamps)
{
    Ball ball_0 = Ball(Point(2, -3), Vector(1, 2), current_time);

    Ball ball_1 = Ball(Point(2, -3), Vector(1, 2), one_second_future);

    EXPECT_EQ(ball_0, ball_1);
}

TEST_F(BallTest, acceleration)
{
    Ball ball_0 = Ball(Point(2, -3), Vector(1, 2), current_time);
    EXPECT_EQ(Vector(0, 0), ball_0.acceleration());
    Ball ball_1 = Ball(Point(2, -3), Vector(1, 2), current_time, Vector(-1, 2));
    EXPECT_EQ(Vector(-1, 2), ball_1.acceleration());
}

TEST_F(BallTest, estimate_future_state_no_acceleration)
{
    Ball ball = Ball(Point(2, -3), Vector(1, 2), current_time);
    BallState expected_future_ball_state(Point(3, -1), Vector(1, 2));
    EXPECT_EQ(expected_future_ball_state,
              ball.estimateFutureState(Duration::fromSeconds(1.0)));
}

TEST_F(BallTest, estimate_future_state_friction)
{
    Ball ball = Ball(Point(2, -3), Vector(1, 2), current_time, Vector(-0.5, -1));
    BallState expected_future_ball_state(Point(2.75, -1.5), Vector(.5, 1));
    EXPECT_EQ(expected_future_ball_state,
              ball.estimateFutureState(Duration::fromSeconds(1.0)));
}

TEST(HasBallBeenKickedTest, ball_over_speed_threshold_and_no_direction_difference)
{
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Vector expected_direction(4, 4);

    EXPECT_TRUE(ball.hasBallBeenKicked(expected_direction.orientation()));
}

TEST(HasBallBeenKickedTest, ball_under_speed_threshold_and_no_direction_difference)
{
    Ball ball({5, 2}, {0, 0.2}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(90);

    EXPECT_FALSE(ball.hasBallBeenKicked(expected_direction));
}

TEST(HasBallBeenKickedTest,
     ball_under_optional_speed_threshold_and_small_direction_difference)
{
    Ball ball({0, 0}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromRadians(0);

    EXPECT_FALSE(ball.hasBallBeenKicked(expected_direction, 5));
}

TEST(HasBallBeenKickedTest,
     ball_over_speed_threshold_and_small_negative_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(-15);

    EXPECT_TRUE(ball.hasBallBeenKicked(expected_direction));
}

TEST(HasBallBeenKickedTest,
     ball_over_speed_threshold_and_small_positive_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(15);

    EXPECT_TRUE(ball.hasBallBeenKicked(expected_direction));
}

TEST(HasBallBeenKickedTest, ball_over_speed_threshold_and_large_direction_difference)
{
    Ball ball({-5, 2}, {-3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromRadians(0);

    EXPECT_FALSE(ball.hasBallBeenKicked(expected_direction));
}

TEST(HasBallBeenKickedTest, ball_under_speed_threshold_and_large_direction_difference)
{
    Ball ball({2, 2}, {0.2, 0.2}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(90);

    EXPECT_FALSE(ball.hasBallBeenKicked(expected_direction));
}
