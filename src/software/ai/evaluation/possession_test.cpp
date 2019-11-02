/**
 * This file contains the unit tests for evaluation functions
 * in baller.cpp
 */

#include "software/ai/evaluation/possession.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/test_util/test_util.h"
#include "software/util/parameter/dynamic_parameters.h"


TEST(PossessionEvaluationTest, get_team_baller_with_empty_team)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    auto baller = Evaluation::getRobotWithEffectiveBallPossession(team, ball, field);
    EXPECT_FALSE(baller);
}

TEST(PossessionEvaluationTest, get_team_baller_robots_and_ball_stationary)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    auto baller = Evaluation::getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot0);
}

TEST(PossessionEvaluationTest, get_team_baller_robot_already_has_ball)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({-2 + DIST_TO_FRONT_OF_ROBOT_METERS, 1}, {0, 0}, Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromSeconds(1));

    Robot robot0 = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});

    auto baller = Evaluation::getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot1);
}

TEST(PossessionEvaluationTest, get_team_baller_ball_moving_towards_robot)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
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
    auto baller = Evaluation::getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot1);
}

TEST(PossessionEvaluationTest, get_team_baller_robot_chasing_ball)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
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
    auto baller = Evaluation::getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot0);
}

TEST(PossessionEvaluationTest, get_team_baller_ball_moving_extremely_fast_out_of_field)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
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
    auto baller = Evaluation::getRobotWithEffectiveBallPossession(team, ball, field);

    EXPECT_TRUE(baller);
    EXPECT_EQ(*baller, robot1);
}

TEST(PossessionEvaluationTest, team_has_possession_robot_in_control)
{
    Ball ball({-0.93, 3}, {0, 10}, Timestamp::fromSeconds(0));
    Team team   = Team(Duration::fromSeconds(1));
    Field field = ::Test::TestUtil::createSSLDivBField();
    World world(field, ball, team, Team(Duration::fromSeconds(1)));

    Robot robot0 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});
    world.updateFriendlyTeamState(team);

    EXPECT_TRUE(Evaluation::teamHasPossession(world, team));
}

TEST(PossessionEvaluationTest, team_does_not_have_posession)
{
    Ball ball({-2, 3}, {0, 10}, Timestamp::fromSeconds(0));
    Team team   = Team(Duration::fromSeconds(1));
    Field field = ::Test::TestUtil::createSSLDivBField();
    World world(field, ball, team, Team(Duration::fromSeconds(1)));

    Robot robot0 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    team.updateRobots({robot0, robot1, robot2});
    world.updateFriendlyTeamState(team);


    EXPECT_FALSE(Evaluation::teamHasPossession(world, team));
}

TEST(PossessionEvaluationTest, team_had_possession_half_second_ago)
{
    Team team = Team(Duration::fromSeconds(1));
    Ball ball({-0.9, 3}, {0, 10}, Timestamp::fromSeconds(2));
    Field field = ::Test::TestUtil::createSSLDivBField();
    World world(field, ball, team, Team(Duration::fromSeconds(1)));

    Robot robot0 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(2));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(2));
    Robot robot2 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(2));

    ball.updateState(
        {0.1, 2.5}, {0, 10},
        Timestamp::fromSeconds(2 + Util::DynamicParameters->getEvaluationConfig()
                                       ->getPossessionConfig()
                                       ->PossessionBufferTimeSeconds()
                                       ->value()));
    world.updateBallState(ball);

    robot0.updateState(
        Point(-2, 3), Vector(), Angle::zero(), AngularVelocity::zero(),
        Timestamp::fromSeconds(2 + Util::DynamicParameters->getEvaluationConfig()
                                       ->getPossessionConfig()
                                       ->PossessionBufferTimeSeconds()
                                       ->value()));
    robot1.updateState(
        Point(-2, 0), Vector(), Angle::quarter(), AngularVelocity::zero(),
        Timestamp::fromSeconds(2 + Util::DynamicParameters->getEvaluationConfig()
                                       ->getPossessionConfig()
                                       ->PossessionBufferTimeSeconds()
                                       ->value()));
    robot2.updateState(
        Point(1.5, 2.3), Vector(), Angle::zero(), AngularVelocity::zero(),
        Timestamp::fromSeconds(2 + Util::DynamicParameters->getEvaluationConfig()
                                       ->getPossessionConfig()
                                       ->PossessionBufferTimeSeconds()
                                       ->value()));

    team.updateRobots({robot0, robot1, robot2});
    world.updateFriendlyTeamState(team);

    EXPECT_TRUE(Evaluation::teamHasPossession(world, team));
}

TEST(PossessionEvaluationTest, team_had_possession_more_than_three_seconds_ago)
{
    Team team = Team(Duration::fromSeconds(1));
    Ball ball({-0.93, 3}, {0, 10}, Timestamp::fromSeconds(0));
    Field field = ::Test::TestUtil::createSSLDivBField();
    World world(field, ball, team, Team(Duration::fromSeconds(1)));

    Robot robot0 = Robot(0, Point(-1, 3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot1 = Robot(1, Point(-2, 0), Vector(), Angle::quarter(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot robot2 = Robot(2, Point(1.5, 2.3), Vector(), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ball.updateState({-0.93, 3}, {0, 10}, Timestamp::fromSeconds(4));
    world.updateBallState(ball);

    robot0.updateState(Point(-2, 3), Vector(), Angle::zero(), AngularVelocity::zero(),
                       Timestamp::fromSeconds(4));
    robot1.updateState(Point(-2, 0), Vector(), Angle::quarter(), AngularVelocity::zero(),
                       Timestamp::fromSeconds(4));
    robot2.updateState(Point(1.5, 2.3), Vector(), Angle::zero(), AngularVelocity::zero(),
                       Timestamp::fromSeconds(4));


    team.updateRobots({robot0, robot1, robot2});
    world.updateFriendlyTeamState(team);

    EXPECT_FALSE(Evaluation::teamHasPossession(world, team));
}
