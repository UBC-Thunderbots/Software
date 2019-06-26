/**
 * This file contains the unit tests for evaluation functions
 * in calc_best_shot.cpp
 */

#include "ai/hl/stp/evaluation/calc_best_shot.h"

#include <gtest/gtest.h>

#include "geom/util.h"
#include "shared/constants.h"
#include "test/test_util/test_util.h"

TEST(CalcBestShotTest, calc_best_shot_on_enemy_goal_with_no_obstacles)
{
    World world          = ::Test::TestUtil::createBlankTestingWorld();
    Team team            = Team(Duration::fromSeconds(1));
    Robot shooting_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    auto result = Evaluation::calcBestShotOnEnemyGoal(world.field(), world.friendlyTeam(),
                                                      world.enemyTeam(), shooting_robot);

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(result->first.isClose(world.field().enemyGoal(), 0.05));
    EXPECT_NEAR(result->second.toDegrees(), 12, 5);
}

TEST(CalcBestShotTest, calc_best_shot_on_friendly_goal_with_no_obstacles)
{
    World world          = ::Test::TestUtil::createBlankTestingWorld();
    Team team            = Team(Duration::fromSeconds(1));
    Robot shooting_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    auto result = Evaluation::calcBestShotOnFriendlyGoal(
        world.field(), world.friendlyTeam(), world.enemyTeam(), shooting_robot);

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(result->first.isClose(world.field().friendlyGoal(), 0.05));
    EXPECT_NEAR(result->second.toDegrees(), 12, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_enemy_goal_with_obstacles_and_no_obstacles_being_ignored)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::Test::TestUtil::setEnemyRobotPositions(
        world, {world.field().enemyGoal(), Point(2.5, 0.7), Point(-1, -1)},
        Timestamp::fromSeconds(0));

    auto result = Evaluation::calcBestShotOnEnemyGoal(world.field(), world.friendlyTeam(),
                                                      world.enemyTeam(), shooting_robot);

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(result->first.isClose(Point(world.field().enemyGoal().x(), -0.3), 0.05));
    EXPECT_NEAR(result->second.toDegrees(), 6, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_friendly_goal_with_obstacles_and_no_obstacles_being_ignored)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(-1, world.field().friendlyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateEnemyTeamState(team);

    world = ::Test::TestUtil::setFriendlyRobotPositions(
        world, {world.field().friendlyGoal(), Point(-2.5, -0.7), Point(1, 1)},
        Timestamp::fromSeconds(0));

    auto result = Evaluation::calcBestShotOnFriendlyGoal(
        world.field(), world.friendlyTeam(), world.enemyTeam(), shooting_robot);

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(
        result->first.isClose(Point(world.field().friendlyGoal().x(), -0.3), 0.05));
    EXPECT_NEAR(result->second.toDegrees(), 6, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_enemy_goal_with_obstacles_and_some_obstacles_being_ignored)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_blocking_robot =
        Robot(1, Point(1.3, world.field().enemyGoalpostNeg().y() - 0.05), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::Test::TestUtil::setEnemyRobotPositions(
        world, {world.field().enemyGoal(), Point(2.5, 0.7), Point(-1, -1)},
        Timestamp::fromSeconds(0));

    auto result = Evaluation::calcBestShotOnEnemyGoal(world.field(), world.friendlyTeam(),
                                                      world.enemyTeam(), shooting_robot);

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(result->first.isClose(Point(world.field().enemyGoal().x(), -0.3), 0.05));
    EXPECT_NEAR(result->second.toDegrees(), 6, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_friendly_goal_with_obstacles_and_some_obstacles_being_ignored)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(-1, world.field().friendlyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_blocking_robot =
        Robot(1, Point(-1.3, world.field().enemyGoalpostNeg().y() - 0.05), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateEnemyTeamState(team);

    world = ::Test::TestUtil::setFriendlyRobotPositions(
        world, {world.field().friendlyGoal(), Point(-2.5, -0.7), Point(1, 1)},
        Timestamp::fromSeconds(0));

    auto result = Evaluation::calcBestShotOnFriendlyGoal(
        world.field(), world.friendlyTeam(), world.enemyTeam(), shooting_robot);

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(
        result->first.isClose(Point(world.field().friendlyGoal().x(), -0.3), 0.05));
    EXPECT_NEAR(result->second.toDegrees(), 6, 5);
}

TEST(CalcBestShotTest, calc_best_shot_on_enemy_goal_with_all_shots_blocked_by_obstacles)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::Test::TestUtil::setEnemyRobotPositions(
        world, {shooting_robot.position() + Vector(ROBOT_MAX_RADIUS_METERS * 2, 0)},
        Timestamp::fromSeconds(0));

    auto result = Evaluation::calcBestShotOnEnemyGoal(world.field(), world.friendlyTeam(),
                                                      world.enemyTeam(), shooting_robot);

    // We should not be able to find a shot
    ASSERT_FALSE(result);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_friendly_goal_with_all_shots_blocked_by_obstacles)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(-1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::Test::TestUtil::setEnemyRobotPositions(
        world, {shooting_robot.position() - Vector(ROBOT_MAX_RADIUS_METERS * 2, 0)},
        Timestamp::fromSeconds(0));

    auto result = Evaluation::calcBestShotOnFriendlyGoal(
        world.field(), world.friendlyTeam(), world.enemyTeam(), shooting_robot);

    // We should not be able to find a shot
    ASSERT_FALSE(result);
}

TEST(CalcBestShotTest, calc_open_enemy_net_percentage_with_unblocked_net)
{
    World world                  = ::Test::TestUtil::createBlankTestingWorld();
    Field field                  = ::Test::TestUtil::createSSLDivBField();
    Point shot_origin            = world.field().enemyGoal() - Vector(0.5, 0);
    std::pair<Point, Angle> shot = {world.field().enemyGoal(), Angle::ofDegrees(90)};

    auto result = Evaluation::calcShotOpenEnemyNetPercentage(field, shot_origin, shot);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 1.0, 0.01);
}

TEST(CalcBestShotTest, calc_open_enemy_net_percentage_with_partially_blocked_net)
{
    World world                  = ::Test::TestUtil::createBlankTestingWorld();
    Field field                  = ::Test::TestUtil::createSSLDivBField();
    Point shot_origin            = world.field().enemyGoal() - Vector(0.5, 0);
    std::pair<Point, Angle> shot = {world.field().enemyGoal() + Vector(0, 0.25),
                                    Angle::ofDegrees(45)};

    auto result = Evaluation::calcShotOpenEnemyNetPercentage(field, shot_origin, shot);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.5, 0.01);
}

TEST(CalcBestShotTest, calc_open_enemy_net_percentage_with_fully_blocked_net)
{
    World world                  = ::Test::TestUtil::createBlankTestingWorld();
    Field field                  = ::Test::TestUtil::createSSLDivBField();
    Point shot_origin            = world.field().enemyGoal() - Vector(0.5, 0);
    std::pair<Point, Angle> shot = {world.field().enemyGoal(), Angle::zero()};

    auto result = Evaluation::calcShotOpenFriendlyNetPercentage(field, shot_origin, shot);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.0, 0.01);
}

TEST(CalcBestShotTest, calc_open_friendly_net_percentage_with_unblocked_net)
{
    World world                  = ::Test::TestUtil::createBlankTestingWorld();
    Field field                  = ::Test::TestUtil::createSSLDivBField();
    Point shot_origin            = world.field().friendlyGoal() + Vector(0.5, 0);
    std::pair<Point, Angle> shot = {world.field().enemyGoal(), Angle::ofDegrees(90)};

    auto result = Evaluation::calcShotOpenFriendlyNetPercentage(field, shot_origin, shot);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 1.0, 0.01);
}

TEST(CalcBestShotTest, calc_open_friendly_net_percentage_with_partially_blocked_net)
{
    World world                  = ::Test::TestUtil::createBlankTestingWorld();
    Field field                  = ::Test::TestUtil::createSSLDivBField();
    Point shot_origin            = world.field().friendlyGoal() + Vector(0.5, 0);
    std::pair<Point, Angle> shot = {world.field().enemyGoal() + Vector(0, 0.25),
                                    Angle::ofDegrees(45)};

    auto result = Evaluation::calcShotOpenFriendlyNetPercentage(field, shot_origin, shot);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.5, 0.01);
}

TEST(CalcBestShotTest, calc_open_friendly_net_percentage_with_fully_blocked_net)
{
    World world                  = ::Test::TestUtil::createBlankTestingWorld();
    Field field                  = ::Test::TestUtil::createSSLDivBField();
    Point shot_origin            = world.field().enemyGoal() + Vector(0.5, 0);
    std::pair<Point, Angle> shot = {world.field().enemyGoal(), Angle::zero()};

    auto result = Evaluation::calcShotOpenEnemyNetPercentage(field, shot_origin, shot);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.0, 0.01);
}
