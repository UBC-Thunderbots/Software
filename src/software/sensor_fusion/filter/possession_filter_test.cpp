#include "software/sensor_fusion/filter/possession_filter.h"

#include <gtest/gtest.h>

#include <iostream>

TEST(PossessionFilterTest, get_possession_distance_directly_in_front_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_TRUE(
        getPossessionDistance(ball.position(), robot.position(), robot.orientation()));
}

TEST(PossessionFilterTest, get_possession_distance_ball_to_side_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::half(), AngularVelocity::zero(),
                        timestamp);
    EXPECT_FALSE(
        getPossessionDistance(ball.position(), robot.position(), robot.orientation()));
}

TEST(PossessionFilterTest, get_possession_distance_robot_moving_ball_in_dribbler)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(1);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(1, 1), Angle::zero(),
                        AngularVelocity::zero(), timestamp);

    EXPECT_TRUE(
        getPossessionDistance(ball.position(), robot.position(), robot.orientation()));
}

TEST(PossessionFilterTest, get_possession_distance_ball_far_away_from_robot)
{
    Point ball_position  = Point(-1, -2);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_FALSE(
        getPossessionDistance(ball.position(), robot.position(), robot.orientation()));
}

TEST(PossessionFilterTest,
     get_possession_distance_ball_slightly_off_center_but_still_on_dribbler)
{
    Point ball_position  = Point(0.07, 0.005);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_TRUE(
        getPossessionDistance(ball.position(), robot.position(), robot.orientation()));
}

TEST(PossessionFilterTest, get_possession_distance_robot_on_angle_with_ball_in_dribbler)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);

    EXPECT_TRUE(
        getPossessionDistance(ball.position(), robot.position(), robot.orientation()));
}

TEST(PossessionFilterTest, get_robots_with_possession_robot_in_control)
{
    Ball ball({-0.93, 3}, {0, 10}, Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot0, robot1, robot2});
    EXPECT_EQ(0, getRobotWithPossession(ball, friendly_team));
    EXPECT_FALSE(getRobotWithPossession(ball, enemy_team));

    enemy_team.updateRobots({robot0, robot1, robot2});
    EXPECT_EQ(0, getRobotWithPossession(ball, friendly_team));
    EXPECT_EQ(0, getRobotWithPossession(ball, enemy_team));
}

TEST(PossessionEvaluationTest, get_robots_with_possession_no_posession)
{
    Ball ball({-2, 3}, {0, 10}, Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    friendly_team.updateRobots({robot0, robot1, robot2});
    enemy_team.updateRobots({robot0, robot1, robot2});

    EXPECT_FALSE(getRobotWithPossession(ball, friendly_team));
    EXPECT_FALSE(getRobotWithPossession(ball, enemy_team));
}

TEST(PossessionEvaluationTest, get_robots_with_possession_breakbeam)
{
    Ball ball({-2, 3}, {0, 10}, Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    EXPECT_FALSE(getRobotWithPossession(ball, friendly_team, {1, 2}));
    EXPECT_FALSE(getRobotWithPossession(ball, enemy_team));
}
