#include "software/world/ball.h"

#include <gtest/gtest.h>

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
    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());
}

TEST_F(BallTest, construct_with_params)
{
    Ball ball = Ball(Point(1, 2.3), Vector(-0.04, 0.0), current_time);

    EXPECT_EQ(Point(1, 2.3), ball.position());
    EXPECT_EQ(Vector(-0.04, 0.0), ball.velocity());
    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());
}

TEST_F(BallTest, construct_with_initial_state)
{
    Ball ball =
        Ball(TimestampedBallState(Point(1, 2.3), Vector(-0.04, 0.0), current_time));

    EXPECT_EQ(Point(1, 2.3), ball.position());
    EXPECT_EQ(Vector(-0.04, 0.0), ball.velocity());
    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());
}

TEST_F(BallTest, update_state_with_all_params)
{
    Ball ball = Ball(Point(), Vector(), current_time);

    ball.updateState(
        TimestampedBallState(Point(-4.23, 1.07), Vector(1, 2), one_second_future));

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(1, 2), one_second_future), ball);
}

TEST_F(BallTest, update_state_with_new_position_old_velocity)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ball.updateState(
        TimestampedBallState(Point(0.01, -99.8), ball.velocity(), current_time));

    EXPECT_EQ(Ball(Point(0.01, -99.8), Vector(1, 2), current_time), ball);
}

TEST_F(BallTest, update_state_with_new_velocity_old_position)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ball.updateState(
        TimestampedBallState(ball.position(), Vector(-0.0, -9.433), current_time));

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(-0.0, -9.433), current_time), ball);
}

TEST_F(BallTest, update_state_with_new_ball)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    TimestampedBallState ball_updated_state =
        TimestampedBallState(Point(), Vector(-4.89, 3.1), current_time);

    ball.updateState(ball_updated_state);

    EXPECT_EQ(ball_updated_state, ball.currentState());
}

TEST_F(BallTest, update_state_with_past_timestamp)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ASSERT_THROW(ball.updateState(TimestampedBallState(Point(-4.23, 1.07), Vector(1, 2),
                                                       one_second_past)),
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

TEST_F(BallTest, get_position_history)
{
    std::vector prevPositions = {Point(-1.3, 3), Point(-1.2, 3), Point(3, 1.2)};

    Ball ball = Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time);
    ball.updateState(
        TimestampedBallState(Point(-1.2, 3), Vector(2.2, -0.05), half_second_future));
    ball.updateState(
        TimestampedBallState(Point(-1.3, 3), Vector(2.3, -0.05), half_second_future));

    BallHistory previous_states = ball.getPreviousStates();
    std::vector<Point> previous_positions{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_positions.push_back(previous_states.at(i).state().position());
    }
    EXPECT_EQ(prevPositions, previous_positions);
}

TEST_F(BallTest, get_velocity_history)
{
    std::vector prevVelocities = {Vector(2.3, -0.05), Vector(2.2, -0.05), Vector(-3, 1)};

    Ball ball = Ball(Point(3, 1.2), Vector(-3, 1), current_time);
    ball.updateState(
        TimestampedBallState(Point(-1.2, 3), Vector(2.2, -0.05), half_second_future));
    ball.updateState(
        TimestampedBallState(Point(-1.3, 3), Vector(2.3, -0.05), half_second_future));

    BallHistory previous_states = ball.getPreviousStates();
    std::vector<Vector> previous_velocities{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_velocities.push_back(previous_states.at(i).state().velocity());
    }
    EXPECT_EQ(prevVelocities, previous_velocities);
}

TEST_F(BallTest, get_timestamp_history)
{
    std::vector prevAngularVelocities = {half_second_future, half_second_future,
                                         current_time};

    Ball ball = Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time);
    ball.updateState(
        TimestampedBallState(Point(-1.2, 3), Vector(2.2, -0.05), half_second_future));
    ball.updateState(
        TimestampedBallState(Point(-1.3, 3), Vector(2.3, -0.05), half_second_future));

    BallHistory previous_states = ball.getPreviousStates();
    std::vector<Timestamp> previous_timestamps{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_timestamps.push_back(previous_states.at(i).timestamp());
    }
    EXPECT_EQ(prevAngularVelocities, previous_timestamps);
}

TEST_F(BallTest, initialize_ball_with_history_size_0)
{
    EXPECT_THROW(Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time, 0),
                 std::invalid_argument);
}
