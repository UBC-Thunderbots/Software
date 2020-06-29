#include "software/ai/evaluation/robot.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"


TEST(RobotEvaluationTest, orientation_in_threshold_facing_0_target_45_threshold_60)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(0);
    Point target      = Point(1, 1);
    Angle threshold   = Angle::fromDegrees(60);

    EXPECT_TRUE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                             target, threshold));
}


TEST(RobotEvaluationTest, orientation_not_in_threshold_facing_0_target_45_threshold_30)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(0);
    Point target      = Point(1, 1);
    Angle threshold   = Angle::fromDegrees(30);

    EXPECT_FALSE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                              target, threshold));
}

TEST(RobotEvaluationTest, orientation_not_in_threshold_facing_0_target_45_threshold_45)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(0);
    Point target      = Point(1, 1);
    Angle threshold   = Angle::fromDegrees(45);

    EXPECT_FALSE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                              target, threshold));
}

TEST(RobotEvaluationTest, orientation_in_threshold_facing_0_target_135_threshold_150)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(0);
    Point target      = Point(-1, 1);
    Angle threshold   = Angle::fromDegrees(150);

    EXPECT_TRUE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                             target, threshold));
}

TEST(RobotEvaluationTest, orientation_not_in_threshold_facing_0_target_135_threshold_90)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(0);
    Point target      = Point(-1, 1);
    Angle threshold   = Angle::fromDegrees(90);

    EXPECT_FALSE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                              target, threshold));
}

TEST(RobotEvaluationTest,
     orientation_in_threshold_facing_150_target_minus_150_threshold_90)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(150);
    Point target      = Point(-2, -1);
    Angle threshold   = Angle::fromDegrees(90);

    EXPECT_TRUE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                             target, threshold));
}

TEST(RobotEvaluationTest,
     orientation_not_in_threshold_facing_150_target_minus_150_threshold_30)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::fromDegrees(150);
    Point target      = Point(-2, -1);
    Angle threshold   = Angle::fromDegrees(30);

    EXPECT_FALSE(robotOrientationWithinAngleThresholdOfTarget(position, orientation,
                                                              target, threshold));
}

TEST(RobotEvaluationTest, has_possession_directly_in_front_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, has_possession_directly_in_front_of_robot_at_future_timestamp)
{
    Point ball_position  = Point(0.1, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(1);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    // we don't have data for future timestamps, should return nullopt
    EXPECT_FALSE(robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates(),
                                    Timestamp::fromSeconds(2))
                     .has_value());
}

TEST(RobotEvaluationTest, has_possession_ball_to_side_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::half(), AngularVelocity::zero(),
                        timestamp);
    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(*result);
}

TEST(RobotEvaluationTest, has_possession_robot_moving_ball_in_dribbler)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(1);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(1, 1), Angle::zero(),
                        AngularVelocity::zero(), timestamp);

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, has_possession_ball_far_away_from_robot)
{
    Point ball_position  = Point(-1, -2);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(*result);
}

TEST(RobotEvaluationTest, has_possession_ball_slightly_off_center_but_still_on_dribbler)
{
    Point ball_position  = Point(0.07, 0.005);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, has_possession_robot_on_angle_with_ball_in_dribbler)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, possession_robot_timestamp_too_far_past)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Ball ball = Ball(ball_position, ball_velocity, Timestamp::fromSeconds(1000));

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates(),
                                     Timestamp::fromSeconds(1000));
    EXPECT_FALSE(result.has_value());
}

TEST(RobotEvaluationTest, possession_ball_timestamp_too_far_past)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Ball ball            = Ball(ball_position, ball_velocity, Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(1000));

    auto result = robotHasPossession(ball.getPreviousStates(), robot.getPreviousStates(),
                                     Timestamp::fromSeconds(1000));
    EXPECT_FALSE(result.has_value());
}


TEST(RobotEvaluationTest, pass_with_stationary_ball)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(*result);
}

TEST(RobotEvaluationTest, pass_with_ball_direct_fast)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(5, 5);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(2.035, 2.06), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, pass_with_ball_direct_fast_at_future_timestamp)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(5, 5);
    Timestamp timestamp  = Timestamp::fromSeconds(1);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(2.035, 2.06), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, pass_with_ball_direct_slow)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(0.1, 0.1);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(2.035, 2.06), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(*result);
}

TEST(RobotEvaluationTest, pass_with_ball_direct_wrong_way)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(-5, -5);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(2.035, 2.06), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(*result);
}

TEST(RobotEvaluationTest, pass_with_ball_slightly_off)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(4, 4.5);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(2.035, 2.06), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
}

TEST(RobotEvaluationTest, pass_ball_robot_timestamp_too_far_past)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(4, 4.5);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);
    Field field          = Field::createSSLDivisionBField();
    World world(field, ball, Team(Duration::fromSeconds(10)),
                Team(Duration::fromSeconds(10)));

    Robot robot = Robot(0, Point(2.035, 2.06), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);
    world.updateFriendlyTeamState(Team({robot}, Duration::fromSeconds(10)));

    auto result =
        robotBeingPassedTo(world.ball().getPreviousStates(), robot.getPreviousStates(),
                           Timestamp::fromSeconds(1000));
    EXPECT_FALSE(result.has_value());
}

class RobotEvaluationFindStateTest : public ::testing::Test
{
   protected:
    RobotEvaluationFindStateTest()
        : current_time(Timestamp::fromSeconds(123)),
          half_second_future(current_time + Duration::fromMilliseconds(500)),
          one_second_future(current_time + Duration::fromSeconds(1)),
          one_second_past(current_time - Duration::fromSeconds(1)),
          robot_state_current_time(
              TimestampedRobotState(Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                                    AngularVelocity::fromDegrees(25), current_time)),
          robot_state_half_second_future(TimestampedRobotState(
              Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
              AngularVelocity::fromRadians(1.1), half_second_future)),
          robot_state_one_second_future(TimestampedRobotState(
              Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
              AngularVelocity::fromRadians(1.2), one_second_future)),
          robot_states(3),
          ball_state_current_time(
              TimestampedBallState(Point(3, 1.2), Vector(2.2, -0.05), current_time)),
          ball_state_half_second_future(TimestampedBallState(
              Point(-1.2, 3), Vector(2.2, -0.05), half_second_future)),
          ball_state_one_second_future(TimestampedBallState(
              Point(-1.3, 3), Vector(2.3, -0.05), one_second_future)),
          ball_states(3)
    {
        robot_states.push_front(robot_state_current_time);
        robot_states.push_front(robot_state_half_second_future);
        robot_states.push_front(robot_state_one_second_future);

        ball_states.push_front(ball_state_current_time);
        ball_states.push_front(ball_state_half_second_future);
        ball_states.push_front(ball_state_one_second_future);
    }

    Timestamp current_time;
    Timestamp half_second_future;
    Timestamp one_second_future;
    Timestamp one_second_past;

    TimestampedRobotState robot_state_current_time;
    TimestampedRobotState robot_state_half_second_future;
    TimestampedRobotState robot_state_one_second_future;
    RobotHistory robot_states;

    TimestampedBallState ball_state_current_time;
    TimestampedBallState ball_state_half_second_future;
    TimestampedBallState ball_state_one_second_future;
    BallHistory ball_states;
};

TEST_F(RobotEvaluationFindStateTest, find_robot_state_fetches_one_second_future)
{
    EXPECT_EQ(robot_state_one_second_future,
              findState<TimestampedRobotState>(robot_states, one_second_future));
}

TEST_F(RobotEvaluationFindStateTest, find_robot_state_fetches_current_time)
{
    EXPECT_EQ(robot_state_current_time,
              findState<TimestampedRobotState>(robot_states, current_time));
}

TEST_F(RobotEvaluationFindStateTest, find_robot_state_no_matching_time)
{
    Timestamp no_matching_time =
        half_second_future +
        Duration::fromMilliseconds(POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS + 1.0);
    EXPECT_EQ(std::nullopt,
              findState<TimestampedRobotState>(robot_states, no_matching_time));
}

TEST_F(RobotEvaluationFindStateTest, find_ball_state_fetches_one_second_future)
{
    EXPECT_EQ(ball_state_one_second_future,
              findState<TimestampedBallState>(ball_states, one_second_future));
}

TEST_F(RobotEvaluationFindStateTest, find_ball_state_fetches_current_time)
{
    EXPECT_EQ(ball_state_current_time,
              findState<TimestampedBallState>(ball_states, current_time));
}

TEST_F(RobotEvaluationFindStateTest, find_ball_state_no_matching_time)
{
    Timestamp no_matching_time =
        half_second_future +
        Duration::fromMilliseconds(POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS + 1.0);
    EXPECT_EQ(std::nullopt,
              findState<TimestampedBallState>(ball_states, no_matching_time));
}
