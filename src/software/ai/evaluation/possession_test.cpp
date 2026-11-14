#include "software/ai/evaluation/possession.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/test_util/test_util.h"


TEST(PossessionEvaluationTest, get_team_baller_with_empty_team)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    auto baller = getRobotWithEffectiveBallPossession(team, ball, field);
    EXPECT_FALSE(baller);
}

TEST(PossessionEvaluationTest, get_team_baller_robots_and_ball_stationary)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    auto baller = getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot0);
}

TEST(PossessionEvaluationTest, get_team_baller_robot_already_has_ball)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball({-2 + DIST_TO_FRONT_OF_ROBOT_METERS, 1}, {0, 0}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    auto baller = getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot1);
}

TEST(PossessionEvaluationTest, get_team_baller_ball_moving_towards_robot)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball({-2, 4}, {0, -2}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(-0, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    // The ball is closer to robot0, but is moving towards robot1 so we expect robot1
    // to be the baller
    auto baller = getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot1);
}

TEST(PossessionEvaluationTest, get_team_baller_robot_chasing_ball)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 3}, {0, -0.5}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(-0.0, 4), Vector(0.0, -1.0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, -2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    // robot0 is chasing the ball and is close enough to catching it we expect it to be
    // the baller
    auto baller = getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot0);
}

TEST(PossessionEvaluationTest, get_team_baller_ball_moving_extremely_fast_out_of_field)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {0, 10}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    // The ball is moving too fast to be caught by any robot within the field, so we
    // expect robot1 to be the baller since it's the closest at this time.
    auto baller = getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot1);
}

TEST(PossessionEvaluationTest, equal_intercept_times_keep_the_first_robot)
{
    const auto field = Field::createSSLDivisionBField();
    Team team(Duration::fromSeconds(1));
    team.updateRobots({Robot(4, Point(-1, 0), Vector(), Angle::zero(),
                             AngularVelocity::zero(), Timestamp()),
                       Robot(2, Point(1, 0), Vector(), Angle::zero(),
                             AngularVelocity::zero(), Timestamp())});
    const auto result = getRobotWithEffectiveBallPossession(
        team, Ball(Point(), Vector(), Timestamp()), field);
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, team.getAllRobots().front());
}

TEST(PossessionEvaluationTest, single_robot_is_still_evaluated)
{
    const auto field = Field::createSSLDivisionBField();
    Team team(Duration::fromSeconds(1));
    const Robot robot(3, Point(1, 0), Vector(0.2, 0), Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(1));
    team.updateRobots({robot});
    for (const auto& velocity : {Vector(), Vector(2, 0)})
    {
        const auto result = getRobotWithEffectiveBallPossession(
            team, Ball(Point(), velocity, Timestamp()), field);
        ASSERT_TRUE(result);
        EXPECT_EQ(*result, robot);
    }
}

TEST(PossessionEvaluationTest, unreachable_stationary_ball_falls_back_to_nearest_robot)
{
    const auto field = Field::createSSLDivisionBField();
    auto constants   = robot_constants::createRobotConstants();
    constants.robot_trajectory_max_speed_m_per_s = 0;
    Team team(Duration::fromSeconds(1));
    team.updateRobots(
        {Robot(0, Point(3, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
               Timestamp(), false, {}, constants),
         Robot(1, Point(1, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
               Timestamp(), false, {}, constants)});
    const auto result = getRobotWithEffectiveBallPossession(
        team, Ball(Point(), Vector(), Timestamp()), field);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->id(), 1);
}

TEST(PossessionEvaluationTest, previous_baller_is_kept_unless_clearly_beaten)
{
    const auto field = Field::createSSLDivisionBField();
    Team team(Duration::fromSeconds(1));
    const Robot farther(4, Point(-1, 0), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp());
    const Robot closer(2, Point(0.9, 0), Vector(), Angle::zero(),
                       AngularVelocity::zero(), Timestamp());
    team.updateRobots({farther, closer});
    const Ball ball{Point(), Vector(), Timestamp()};

    // Without a previous baller, or with no margin, the closer robot wins
    EXPECT_EQ(*getRobotWithEffectiveBallPossession(team, ball, field), closer);
    EXPECT_EQ(*getRobotWithEffectiveBallPossession(team, ball, field, 4), closer);

    // A margin larger than the time difference keeps the previous baller
    EXPECT_EQ(*getRobotWithEffectiveBallPossession(team, ball, field, 4,
                                                   Duration::fromSeconds(1)),
              farther);
    EXPECT_EQ(*getRobotWithEffectiveBallPossession(team, ball, field, 2,
                                                   Duration::fromSeconds(1)),
              closer);

    // A margin smaller than the time difference does not
    EXPECT_EQ(*getRobotWithEffectiveBallPossession(team, ball, field, 4,
                                                   Duration::fromSeconds(0.001)),
              closer);

    // A previous baller that is no longer on the team is ignored
    EXPECT_EQ(*getRobotWithEffectiveBallPossession(team, ball, field, 9,
                                                   Duration::fromSeconds(1)),
              closer);
}

TEST(PossessionEvaluationTest, previous_baller_cannot_keep_an_unreachable_ball)
{
    const auto field = Field::createSSLDivisionBField();
    Team team(Duration::fromSeconds(1));
    const Robot a(1, Point(-1, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp());
    const Robot b(2, Point(1, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp());
    team.updateRobots({a, b});
    // The ball is outside the field, so nobody can intercept it: fall back to nearest
    const Ball ball{Point(0, 20), Vector(), Timestamp()};
    const auto result = getRobotWithEffectiveBallPossession(
        team, ball, field, 1, Duration::fromSeconds(5));
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, team.getNearestRobot(ball.position()));
}
