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

TEST_F(BallTest, update_state_to_predicted_state_with_future_timestamp)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    ball.updateStateToPredictedState(one_second_future);

    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    EXPECT_EQ(Point(-1.5, 6.88), ball.position());
    EXPECT_LT((Vector(-4.0717, -0.1086) - ball.velocity()).length(), EPSILON);
    EXPECT_EQ(one_second_future, ball.lastUpdateTimestamp());
}


TEST_F(BallTest, update_state_to_predicted_state_with_past_timestamp)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    ASSERT_THROW(ball.updateStateToPredictedState(one_second_past),
                 std::invalid_argument);
}

TEST_F(BallTest, get_position_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(Point(3, 7), ball.position());
}

TEST_F(BallTest, get_position_at_future_time_with_positive_ball_velocity)
{
    Ball ball = Ball(Point(), Vector(1, 2), current_time);

    EXPECT_EQ(Point(0.15, 0.3),
              ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(150)));
    EXPECT_EQ(Point(1, 2),
              ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(2, 4),
              ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(2000)));
}

TEST_F(BallTest, get_position_at_future_time_with_negative_ball_velocity)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(Point(2.325, 6.982),
              ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(150)));
    EXPECT_EQ(Point(-1.5, 6.88),
              ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(-6, 6.76),
              ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(2000)));
}

TEST_F(BallTest, get_position_at_past_time)
{
    Ball ball = Ball(Point(3, 7), Vector(1, 0.12), current_time);

    ASSERT_THROW(ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(-200)),
                 std::invalid_argument);
    ASSERT_THROW(ball.estimatePositionAtFutureTime(Duration::fromMilliseconds(-2000)),
                 std::invalid_argument);
}

TEST_F(BallTest, get_velocity_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(Vector(-4.5, -0.12), ball.velocity());
}

TEST_F(BallTest, get_velocity_at_future_time_with_positive_ball_velocity)
{
    Ball ball = Ball(Point(), Vector(1, 2), current_time);

    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    EXPECT_LT((Vector(0.9851, 1.9702) -
               ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(150)))
                  .length(),
              EPSILON);
    EXPECT_LT((Vector(0.9048, 1.8097) -
               ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(1000)))
                  .length(),
              EPSILON);
    EXPECT_LT((Vector(0.8187, 1.6375) -
               ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(2000)))
                  .length(),
              EPSILON);
}

TEST_F(BallTest, get_velocity_at_future_time_with_negative_ball_velocity)
{
    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_LT((Vector(-4.4330, -0.1182) -
               ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(150)))
                  .length(),
              EPSILON);
    EXPECT_LT((Vector(-4.0717, -0.1086) -
               ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(1000)))
                  .length(),
              EPSILON);
    EXPECT_LT((Vector(-3.6843, -0.0982) -
               ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(2000)))
                  .length(),
              EPSILON);
}

TEST_F(BallTest, get_velocity_at_past_time)
{
    Ball ball = Ball(Point(), Vector(1, 2), current_time);

    ASSERT_THROW(ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(-150)),
                 std::invalid_argument);
    ASSERT_THROW(ball.estimateVelocityAtFutureTime(Duration::fromMilliseconds(-1000)),
                 std::invalid_argument);
}

TEST_F(BallTest, get_last_update_timestamp)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());

    ball.updateStateToPredictedState(half_second_future);

    EXPECT_EQ(half_second_future, ball.lastUpdateTimestamp());
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

    boost::circular_buffer<TimestampedBallState> previous_states =
        ball.getPreviousStates();
    std::vector<Point> previous_positions{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_positions.push_back(previous_states.at(i).ballState().position());
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

    boost::circular_buffer<TimestampedBallState> previous_states =
        ball.getPreviousStates();
    std::vector<Vector> previous_velocities{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_velocities.push_back(previous_states.at(i).ballState().velocity());
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

    boost::circular_buffer<TimestampedBallState> previous_states =
        ball.getPreviousStates();
    std::vector<Timestamp> previous_timestamps{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_timestamps.push_back(previous_states.at(i).timestamp());
    }
    EXPECT_EQ(prevAngularVelocities, previous_timestamps);
}

TEST_F(BallTest, get_timestamp_index_fetches_first_index)
{
    std::vector prevAngularVelocities = {half_second_future, half_second_future,
                                         current_time};

    Ball ball = Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time);
    ball.updateState(
        TimestampedBallState(Point(-1.2, 3), Vector(2.2, -0.05), half_second_future));
    ball.updateState(
        TimestampedBallState(Point(-1.3, 3), Vector(2.3, -0.05), one_second_future));

    EXPECT_EQ(0, ball.getHistoryIndexFromTimestamp(one_second_future));
}

TEST_F(BallTest, get_timestamp_index_fetches_last_index)
{
    std::vector prevAngularVelocities = {half_second_future, half_second_future,
                                         current_time};

    Ball ball = Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time);
    ball.updateState(
        TimestampedBallState(Point(-1.2, 3), Vector(2.2, -0.05), half_second_future));
    ball.updateState(
        TimestampedBallState(Point(-1.3, 3), Vector(2.3, -0.05), one_second_future));

    EXPECT_EQ(2, ball.getHistoryIndexFromTimestamp(current_time));
}

TEST_F(BallTest, get_timestamp_index_no_matching_timestamp)
{
    std::vector prevAngularVelocities = {half_second_future, half_second_future,
                                         current_time};

    Ball ball = Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time);
    ball.updateState(
        TimestampedBallState(Point(-1.2, 3), Vector(2.2, -0.05), half_second_future));
    ball.updateState(
        TimestampedBallState(Point(-1.3, 3), Vector(2.3, -0.05), one_second_future));

    Timestamp no_matching_time =
        half_second_future +
        Duration::fromMilliseconds(POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS + 1);

    EXPECT_EQ(std::nullopt, ball.getHistoryIndexFromTimestamp(no_matching_time));
}

TEST_F(BallTest, initialize_ball_with_history_size_0)
{
    EXPECT_THROW(Ball(Point(3, 1.2), Vector(2.2, -0.05), current_time, 0),
                 std::invalid_argument);
}
