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

    auto result = robotHasPossession(ball, robot);
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
    EXPECT_FALSE(robotHasPossession(ball, robot, Timestamp::fromSeconds(2)).has_value());
}

TEST(RobotEvaluationTest, has_possession_ball_to_side_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::half(), AngularVelocity::zero(),
                        timestamp);
    auto result = robotHasPossession(ball, robot);
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

    auto result = robotHasPossession(ball, robot);
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

    auto result = robotHasPossession(ball, robot);
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

    auto result = robotHasPossession(ball, robot);
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

    auto result = robotHasPossession(ball, robot);
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

    auto result = robotHasPossession(ball, robot, Timestamp::fromSeconds(1000));
    EXPECT_FALSE(result.has_value());
}

TEST(RobotEvaluationTest, possession_ball_timestamp_too_far_past)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Ball ball            = Ball(ball_position, ball_velocity, Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(1000));

    auto result = robotHasPossession(ball, robot, Timestamp::fromSeconds(1000));
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

    auto result = robotBeingPassedTo(world, robot);
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

    auto result = robotBeingPassedTo(world, robot);
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

    auto result = robotBeingPassedTo(world, robot);
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

    auto result = robotBeingPassedTo(world, robot);
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

    auto result = robotBeingPassedTo(world, robot);
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

    auto result = robotBeingPassedTo(world, robot);
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

    auto result = robotBeingPassedTo(world, robot, Timestamp::fromSeconds(1000));
    EXPECT_FALSE(result.has_value());
}
