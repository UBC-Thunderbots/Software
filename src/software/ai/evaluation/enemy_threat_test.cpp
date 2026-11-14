#include "software/ai/evaluation/enemy_threat.h"

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <random>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/geom/algorithms/intersects.h"
#include "software/test_util/test_util.h"

TEST(FindAllPasserReceiverPairsTest, robot_passing_to_itself)
{
    // Self-passes aren't filtered out, but callers never put a robot in both lists
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    auto result = findAllReceiverPasserPairs({friendly_robot_0}, {friendly_robot_0},
                                             {friendly_robot_0});

    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> expected_result = {
        std::make_pair(friendly_robot_0, std::vector<Robot>{friendly_robot_0})};

    EXPECT_EQ(result, expected_result);
}

TEST(FindAllPasserReceiverPairsTest, one_passer_one_receiver_with_no_obstacles)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(-5, -5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    std::vector<Robot> all_robots{friendly_robot_0, friendly_robot_1};

    auto result =
        findAllReceiverPasserPairs({friendly_robot_0}, {friendly_robot_1}, all_robots);

    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> expected_result = {
        std::make_pair(friendly_robot_1, std::vector<Robot>{friendly_robot_0})};

    EXPECT_EQ(result, expected_result);
}

TEST(FindAllPasserReceiverPairsTest,
     two_passers_one_receiver_with_one_pass_blocked_by_obstacles)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(3, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_2 = Robot(2, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // Blocks the pass from robot 0 to robot 2
    Robot enemy_robot_0 = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Robot> all_robots{friendly_robot_0, friendly_robot_1, friendly_robot_2,
                                  enemy_robot_0};

    auto result = findAllReceiverPasserPairs({friendly_robot_0, friendly_robot_1},
                                             {friendly_robot_2}, all_robots);

    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> expected_result = {
        std::make_pair(friendly_robot_2, std::vector<Robot>{friendly_robot_1})};

    EXPECT_EQ(result, expected_result);
}

TEST(FindAllPasserReceiverPairsTest, all_passes_blocked)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // Blocks the pass from robot 0 to robot 1
    Robot enemy_robot_0 = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Robot> all_robots{friendly_robot_0, friendly_robot_1, enemy_robot_0};

    auto result =
        findAllReceiverPasserPairs({friendly_robot_0}, {friendly_robot_1}, all_robots);

    EXPECT_TRUE(result.empty());
}

TEST(FindAllPasserReceiverPairsTest, receiver_with_multiple_passers)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(3, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_2 = Robot(2, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Robot> all_robots{friendly_robot_0, friendly_robot_1, friendly_robot_2};

    auto result = findAllReceiverPasserPairs({friendly_robot_0, friendly_robot_1},
                                             {friendly_robot_2}, all_robots);

    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> expected_result = {
        std::make_pair(friendly_robot_2,
                       std::vector<Robot>{friendly_robot_0, friendly_robot_1})};

    EXPECT_EQ(result, expected_result);
}

TEST(GetNumPassesToRobotTest, robot_passing_to_itself)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(-5, -5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});

    Team enemy_team = Team(Duration::fromSeconds(1));

    auto result = getNumPassesToRobot(friendly_robot_0, friendly_robot_0, friendly_team,
                                      enemy_team);

    // A valid result should have been found
    ASSERT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(0, num_passes);
    ASSERT_FALSE(passer);
}

TEST(GetNumPassesToRobotTest, one_simple_pass_to_robot_with_no_obstacles)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(-5, -5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});

    Team enemy_team = Team(Duration::fromSeconds(1));

    // Robot 0 should be able to pass to robot 1 in a single pass
    auto result = getNumPassesToRobot(friendly_robot_0, friendly_robot_1, friendly_team,
                                      enemy_team);

    // A valid result should have been found
    ASSERT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(1, num_passes);
    ASSERT_TRUE(passer);
    EXPECT_EQ(passer.value(), friendly_robot_0);
}

// TODO: Re-enable as part of https://github.com/UBC-Thunderbots/Software/issues/642
// TEST(GetNumPassesToRobotTest, two_passes_around_a_single_obstacle)
//{
//    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_1 = Robot(1, Point(3, 1.5), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_2 = Robot(2, Point(5, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team friendly_team     = Team(Duration::fromSeconds(1));
//    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1, friendly_robot_2});
//
//    Robot enemy_robot_0 = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(),
//                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team enemy_team     = Team(Duration::fromSeconds(1));
//    enemy_team.updateRobots({enemy_robot_0});
//
//    // The enemy robot is blocking the pass from robot 0 to robot 2, so we expect an
//    // intermediate pass via robot 1
//    auto result = getNumPassesToRobot(friendly_robot_0, friendly_robot_2,
//                                                  friendly_team, enemy_team);
//
//    // A valid result should have been found
//    EXPECT_TRUE(result);
//
//    int num_passes              = result.value().first;
//    std::optional<Robot> passer = result.value().second;
//
//    EXPECT_EQ(2, num_passes);
//    EXPECT_TRUE(passer);
//    EXPECT_EQ(passer.value(), friendly_robot_1);
//}

// TODO: Re-enable as part of https://github.com/UBC-Thunderbots/Software/issues/642
// TEST(GetNumPassesToRobotTest, multiple_friendly_robots_and_blocking_enemies)
//{
//    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_1 = Robot(1, Point(1, 1.5), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_2 = Robot(2, Point(3.5, -2), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_3 = Robot(3, Point(5, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team friendly_team     = Team(Duration::fromSeconds(1));
//    friendly_team.updateRobots(
//        {friendly_robot_0, friendly_robot_1, friendly_robot_2, friendly_robot_3});
//
//    // Blocks the pass between robot 0 and robot 3
//    Robot enemy_robot_0 = Robot(0, Point(4, 0), Vector(0, 0), Angle::zero(),
//                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    // Blocks the pass between robot 1 and 3
//    Robot enemy_robot_1 = Robot(1, Point(1.25, 1.4), Vector(0, 0), Angle::zero(),
//                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    // Blocks the pass between robot 0 and 2
//    Robot enemy_robot_2 = Robot(2, Point(1.75, -1), Vector(0, 0), Angle::zero(),
//                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team enemy_team     = Team(Duration::fromSeconds(1));
//    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1, enemy_robot_2});
//
//    // The only way for robot 3 to get the ball is to receive a pass from
//    // robot 0 -> robot 1 -> robot 2 -> robot 3
//    auto result = getNumPassesToRobot(friendly_robot_0, friendly_robot_3,
//                                                  friendly_team, enemy_team);
//
//    // A valid result should have been found
//    ASSERT_TRUE(result);
//
//    int num_passes              = result.value().first;
//    std::optional<Robot> passer = result.value().second;
//
//    EXPECT_EQ(3, num_passes);
//    ASSERT_TRUE(passer);
//    EXPECT_EQ(passer.value(), friendly_robot_2);
//}


// TODO: Re-enable as part of https://github.com/UBC-Thunderbots/Software/issues/642
// TEST(GetNumPassesToRobotTest, all_passes_blocked)
//{
//    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_1 = Robot(1, Point(5, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team friendly_team     = Team(Duration::fromSeconds(1));
//    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
//
//    Robot enemy_robot_0 = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(),
//                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team enemy_team     = Team(Duration::fromSeconds(1));
//    enemy_team.updateRobots({enemy_robot_0});
//
//    auto result = getNumPassesToRobot(friendly_robot_0, friendly_robot_1,
//                                                  friendly_team, enemy_team);
//
//    // We don't expect any pass info to be returned
//    EXPECT_FALSE(result);
//}

// TEST(GetNumPassesToRobotTest, final_receiver_can_receive_passes_from_multiple_robots)
//{
//    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_1 = Robot(1, Point(1, 1.5), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_2 = Robot(2, Point(3.5, -2), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Robot friendly_robot_3 = Robot(3, Point(5, 0), Vector(0, 0), Angle::zero(),
//                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team friendly_team     = Team(Duration::fromSeconds(1));
//    friendly_team.updateRobots(
//        {friendly_robot_0, friendly_robot_1, friendly_robot_2, friendly_robot_3});
//
//    // Blocks the pass between robot 0 and robot 3
//    Robot enemy_robot_0 = Robot(0, Point(4, 0), Vector(0, 0), Angle::zero(),
//                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
//    Team enemy_team     = Team(Duration::fromSeconds(1));
//    enemy_team.updateRobots({enemy_robot_0});
//
//    // robot 3 can receive the ball from either:
//    // robot 0 -> robot 1 -> robot 3
//    // or
//    // robot 0 -> robot 2 -> robot 3
//    // Robot 2 is closer to robot 3 so we expect it to be the most likely passer
//    auto result = getNumPassesToRobot(friendly_robot_0, friendly_robot_3,
//                                                  friendly_team, enemy_team);
//
//    // A valid result should have been found
//    EXPECT_TRUE(result);
//
//    int num_passes              = result.value().first;
//    std::optional<Robot> passer = result.value().second;
//
//    EXPECT_EQ(2, num_passes);
//    EXPECT_TRUE(passer);
//    EXPECT_EQ(passer.value(), friendly_robot_2);
//}

TEST(SortEnemyThreatsTest, only_one_robot_has_possession)
{
    // The exact state of the robots don't matter for these tests.
    // Only the data in the struct matters
    Robot robot1 = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot2 = Robot(1, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    auto threat1 = EnemyThreat{
        robot1, true,        Angle::fromDegrees(50), Angle::fromDegrees(20), Point(-4, 0),
        0,      std::nullopt};

    auto threat2 = EnemyThreat{
        robot2, false, Angle::fromDegrees(60), Angle::fromDegrees(30), Point(-4, 0),
        1,      robot1};

    // Despite robot2 having better shooting and scoring opportunity, robot1 has the ball
    // so should be more threatening
    std::vector<EnemyThreat> expected_result = {threat1, threat2};

    std::vector<EnemyThreat> threats = {threat2, threat1};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, expected_result);
}

TEST(SortEnemyThreatsTest, multiple_robots_have_possession_simultaneously)
{
    // The exact state of the robots don't matter for these tests.
    // Only the data in the struct matters
    Robot robot1 = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot2 = Robot(1, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    auto threat1 = EnemyThreat{
        robot1, true,        Angle::fromDegrees(50), Angle::fromDegrees(20), Point(-4, 0),
        0,      std::nullopt};

    auto threat2 = EnemyThreat{
        robot2, true,  Angle::fromDegrees(60), Angle::fromDegrees(30), Point(-4, 0),
        1,      robot1};

    // Both robots have possession but robot2 has a better shot on the friendly goal, so
    // it should be more threatening
    std::vector<EnemyThreat> expected_result = {threat2, threat1};

    std::vector<EnemyThreat> threats = {threat1, threat2};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, expected_result);
}

TEST(SortEnemyThreatsTest,
     neither_robot_has_possession_but_take_a_different_number_of_passes_to_be_reached)
{
    // The exact state of the robots don't matter for these tests.
    // Only the data in the struct matters
    Robot robot1 = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot2 = Robot(1, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    auto threat1 = EnemyThreat{
        robot1, false,       Angle::fromDegrees(50), Angle::fromDegrees(20), Point(-4, 0),
        1,      std::nullopt};

    auto threat2 = EnemyThreat{
        robot2, false, Angle::fromDegrees(60), Angle::fromDegrees(30), Point(-4, 0),
        2,      robot1};

    // robot1 can be reached in fewer passes, so it should be more threatening
    std::vector<EnemyThreat> expected_result = {threat1, threat2};

    std::vector<EnemyThreat> threats = {threat2, threat1};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, expected_result);
}

TEST(SortEnemyThreatsTest,
     neither_robot_has_possession_but_can_be_reached_in_the_same_number_of_passes)
{
    // The exact state of the robots don't matter for these tests.
    // Only the data in the struct matters
    Robot robot1 = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot2 = Robot(1, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    auto threat1 = EnemyThreat{
        robot1,       false, Angle::fromDegrees(50), Angle::fromDegrees(20),
        Point(-4, 0), 2,
        std::nullopt  // The passer doesn't matter since it doesn't affect the threat
                      // It's only for the use of whatever code uses these threat
                      // evaluations
    };

    auto threat2 = EnemyThreat{
        robot2,       false, Angle::fromDegrees(60), Angle::fromDegrees(30),
        Point(-4, 0), 2,
        std::nullopt  // The passer doesn't matter since it doesn't affect the threat
                      // It's only for the use of whatever code uses these threat
                      // evaluations
    };

    // Robot 2 has a better view of the goal so it's more threatening
    std::vector<EnemyThreat> expected_result = {threat2, threat1};

    std::vector<EnemyThreat> threats = {threat1, threat2};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, expected_result);
}

TEST(SortEnemyThreatsTest, robot_with_possession_but_no_shot_is_still_the_biggest_threat)
{
    // The exact state of the robots don't matter for these tests.
    // Only the data in the struct matters
    Robot robot1 = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot2 = Robot(1, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    auto threat1 = EnemyThreat{robot1,       true,         Angle::fromDegrees(25),
                               std::nullopt, std::nullopt, 0,
                               std::nullopt};

    auto threat2 = EnemyThreat{
        robot2, false, Angle::fromDegrees(19), Angle::fromDegrees(11), Point(-4, 0),
        1,      robot1};

    // Possession outranks everything else, so robot1 is more threatening even with no
    // shot at all while robot2 has a clear one
    std::vector<EnemyThreat> expected_result = {threat1, threat2};

    std::vector<EnemyThreat> threats = {threat2, threat1};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, expected_result);
}

TEST(SortEnemyThreatsTest, both_robots_have_possession_but_only_one_has_a_shot)
{
    // The exact state of the robots don't matter for these tests.
    // Only the data in the struct matters
    Robot robot1 = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Robot robot2 = Robot(1, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    auto threat1 = EnemyThreat{robot1,       true,         Angle::fromDegrees(50),
                               std::nullopt, std::nullopt, 0,
                               std::nullopt};

    auto threat2 = EnemyThreat{
        robot2, true,        Angle::fromDegrees(30), Angle::fromDegrees(20), Point(-4, 0),
        0,      std::nullopt};

    // Both have the ball, so they are ranked by their shot. An empty best_shot_angle
    // sorts below any real angle, making robot1 the less threatening of the two
    std::vector<EnemyThreat> expected_result = {threat2, threat1};

    std::vector<EnemyThreat> threats = {threat1, threat2};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, expected_result);
}

TEST(EnemyThreatTest, no_enemies_on_field)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    ::TestUtil::setBallPosition(world,
                                Point(world->field().friendlyGoalCenter()) +
                                    Vector(2 - ROBOT_MAX_RADIUS_METERS, 0),
                                Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    // Make sure we got the correct number of results
    EXPECT_EQ(result.size(), 0);
}


TEST(EnemyThreatTest, single_enemy_in_front_of_net_with_ball_and_no_obstacles)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot enemy_robot_0 =
        Robot(0, Point(world->field().friendlyGoalCenter()) + Vector(2, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_0});
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(world,
                                Point(world->field().friendlyGoalCenter()) +
                                    Vector(2 - DIST_TO_FRONT_OF_ROBOT_METERS, 0),
                                Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    // Make sure we got the correct number of results
    EXPECT_EQ(result.size(), 1);

    auto threat = result.at(0);
    EXPECT_EQ(threat.robot, enemy_robot_0);
    EXPECT_TRUE(threat.has_ball);
    EXPECT_NEAR(threat.goal_angle.toDegrees(), 30, 5);
    ASSERT_TRUE(threat.best_shot_angle);
    EXPECT_NEAR(threat.best_shot_angle->toDegrees(), 30, 5);
    ASSERT_TRUE(threat.best_shot_target);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        *threat.best_shot_target, world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat.num_passes_to_get_possession, 0);
    ASSERT_FALSE(threat.passer);
}

TEST(EnemyThreatTest, enemy_goalie_included_and_excluded)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot enemy_goalie =
        Robot(0, world->field().enemyGoalCenter(), Vector(0, 0), Angle::half(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(2, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_goalie, enemy_robot_1});
    enemy_team.assignGoalie(0);
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(world,
                                Point(world->field().friendlyGoalCenter()) +
                                    Vector(2 - DIST_TO_FRONT_OF_ROBOT_METERS, 0),
                                Timestamp::fromSeconds(0));

    auto threats_without_goalie = getAllEnemyThreats(
        world->field(), world->friendlyTeam(), world->enemyTeam(), world->ball(), false);
    ASSERT_EQ(threats_without_goalie.size(), 1);
    EXPECT_EQ(threats_without_goalie.at(0).robot, enemy_robot_1);

    auto threats_with_goalie = getAllEnemyThreats(
        world->field(), world->friendlyTeam(), world->enemyTeam(), world->ball(), true);
    ASSERT_EQ(threats_with_goalie.size(), 2);
    // Enemy robot 1 has the ball, so it is the most threatening of the two
    EXPECT_EQ(threats_with_goalie.at(0).robot, enemy_robot_1);
    EXPECT_EQ(threats_with_goalie.at(1).robot, enemy_goalie);
}

TEST(EnemyThreatTest, two_enemies_with_the_ball_carrier_screened)
{
    // Two friendly robots screen the enemy holding the ball, leaving it almost no shot,
    // while the other enemy is unmarked with a much better view of the net

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(2.3, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(1.9, 1.5), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_1, enemy_robot_2});
    world->updateEnemyTeamState(enemy_team);

    Robot friendly_goalie =
        Robot(0, world->field().friendlyGoalCenter() + Vector(0.2, 0), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(1.6, 0.15), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(1.6, -0.15), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_goalie, friendly_robot_1, friendly_robot_2});
    world->updateFriendlyTeamState(friendly_team);

    ::TestUtil::setBallPosition(
        world, enemy_robot_1.position() + Vector(-DIST_TO_FRONT_OF_ROBOT_METERS, 0),
        Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    ASSERT_EQ(result.size(), 2);

    // Possession is currently weighted above shot quality, so the screened enemy is
    // ranked first even though the unmarked enemy has the better shot
    EXPECT_EQ(result.at(0).robot, enemy_robot_1);
    EXPECT_TRUE(result.at(0).has_ball);
    EXPECT_EQ(result.at(1).robot, enemy_robot_2);
    EXPECT_FALSE(result.at(1).has_ball);

    ASSERT_TRUE(result.at(0).best_shot_angle);
    ASSERT_TRUE(result.at(1).best_shot_angle);
    EXPECT_LT(result.at(0).best_shot_angle->toDegrees(),
              result.at(1).best_shot_angle->toDegrees());
}

TEST(EnemyThreatTest, three_enemies_vs_one_friendly)
{
    // This test evaluates the enemy threat for a 3-vs-1 scenario
    //
    //                                    enemy robot 2
    //
    //
    //      enemy robot 1
    //          ball
    //
    //                         friendly robot
    //
    //
    //                                                    enemy robot 3
    //
    //
    //                       | friendly net |
    //                       ----------------
    //
    // This tests threat cases where passes between enemy robots are possible, and where
    // different robots have significantly different views of the goal.
    //
    // Enemy robot 1 is the most threatening because it has the ball and has a good view
    // of the goal. Enemy robot 2 is the second most threatening because it also has a
    // good view of the goal, and can receive the ball quickly via a pass from enemy 1.
    // Finally, enemy robot 3 is the least threatening because it doesn't have a great
    // angle on the goal because it's off to the side

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    // Robots are positioned relative to the friendly goal
    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(1.25, 1.5), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(2, -1), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_3 =
        Robot(3, world->field().friendlyGoalCenter() + Vector(0.4, -2), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_1, enemy_robot_2, enemy_robot_3});
    world->updateEnemyTeamState(enemy_team);

    Robot friendly_robot =
        Robot(0, world->field().friendlyGoalCenter() + Vector(1, 0.5), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_robot});
    world->updateFriendlyTeamState(friendly_team);

    // Put the ball right in front of enemy 1
    ::TestUtil::setBallPosition(
        world, enemy_robot_1.position() + Vector(-DIST_TO_FRONT_OF_ROBOT_METERS, 0),
        Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    // Make sure we got the correct number of results
    EXPECT_EQ(result.size(), 3);

    auto threat_0 = result.at(0);
    EXPECT_EQ(threat_0.robot, enemy_robot_1);
    EXPECT_TRUE(threat_0.has_ball);
    EXPECT_NEAR(threat_0.goal_angle.toDegrees(), 15, 5);
    ASSERT_TRUE(threat_0.best_shot_angle);
    EXPECT_NEAR(threat_0.best_shot_angle->toDegrees(), 15, 5);
    ASSERT_TRUE(threat_0.best_shot_target);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        *threat_0.best_shot_target, world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat_0.num_passes_to_get_possession, 0);
    ASSERT_FALSE(threat_0.passer);

    auto threat_1 = result.at(1);
    EXPECT_EQ(threat_1.robot, enemy_robot_2);
    EXPECT_FALSE(threat_1.has_ball);
    EXPECT_NEAR(threat_1.goal_angle.toDegrees(), 20, 5);
    ASSERT_TRUE(threat_1.best_shot_angle);
    EXPECT_NEAR(threat_1.best_shot_angle->toDegrees(), 20, 5);
    ASSERT_TRUE(threat_1.best_shot_target);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        *threat_1.best_shot_target, world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat_1.num_passes_to_get_possession, 1);
    ASSERT_TRUE(threat_1.passer);
    EXPECT_EQ(threat_1.passer, enemy_robot_1);

    auto threat_2 = result.at(2);
    EXPECT_EQ(threat_2.robot, enemy_robot_3);
    EXPECT_FALSE(threat_2.has_ball);
    EXPECT_NEAR(threat_2.goal_angle.toDegrees(), 5, 5);
    ASSERT_TRUE(threat_2.best_shot_angle);
    EXPECT_NEAR(threat_2.best_shot_angle->toDegrees(), 5, 5);
    ASSERT_TRUE(threat_2.best_shot_target);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        *threat_2.best_shot_target, world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat_2.num_passes_to_get_possession, 1);
    ASSERT_TRUE(threat_2.passer);
    EXPECT_EQ(threat_2.passer, enemy_robot_1);
}

TEST(EnemyThreatTest, four_enemies_with_the_pass_to_the_best_position_blocked)
{
    // Enemy robot 3 is parked right in front of our net and has by far the best view of
    // it, but enemy robot 4 blocks the only direct pass to it, so it takes two passes to
    // reach. Enemies that can be reached sooner are considered more threatening.
    //
    //                                 enemy robot 2
    //
    //                                                       enemy robot 1
    //      | friendly net |    enemy robot 3   enemy robot 4    ball
    //      ----------------

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(3.0, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(2.0, 1.5), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_3 =
        Robot(3, world->field().friendlyGoalCenter() + Vector(0.8, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_4 =
        Robot(4, world->field().friendlyGoalCenter() + Vector(2.0, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_1, enemy_robot_2, enemy_robot_3, enemy_robot_4});
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(
        world, enemy_robot_1.position() + Vector(-DIST_TO_FRONT_OF_ROBOT_METERS, 0),
        Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    ASSERT_EQ(result.size(), 4);

    EXPECT_EQ(result.at(0).robot, enemy_robot_1);
    EXPECT_TRUE(result.at(0).has_ball);
    EXPECT_EQ(result.at(1).robot, enemy_robot_4);
    EXPECT_EQ(result.at(2).robot, enemy_robot_2);
    EXPECT_EQ(result.at(3).robot, enemy_robot_3);

    // Ranked last despite having the best view of our net
    EXPECT_GT(result.at(3).goal_angle, result.at(1).goal_angle);
    EXPECT_EQ(result.at(1).num_passes_to_get_possession, 1);
    EXPECT_EQ(result.at(2).num_passes_to_get_possession, 1);
    EXPECT_EQ(result.at(3).num_passes_to_get_possession, 2);
    ASSERT_TRUE(result.at(3).passer);
    EXPECT_EQ(result.at(3).passer, enemy_robot_4);
}

TEST(EnemyThreatTest, three_enemies_one_pass_away_with_different_goal_angles)
{
    // Enemy robots 2 and 3 are both one pass away, so the tie is broken by their view
    // of the net. Enemy robot 2 is head on, enemy robot 3 is off to the side.
    //                                                     enemy robot 3
    //
    //                                                            enemy robot 1
    //      | friendly net |            enemy robot 2                  ball
    //      ----------------

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(3.0, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(1.5, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_3 =
        Robot(3, world->field().friendlyGoalCenter() + Vector(1.5, 2.5), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_1, enemy_robot_2, enemy_robot_3});
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(
        world, enemy_robot_1.position() + Vector(-DIST_TO_FRONT_OF_ROBOT_METERS, 0),
        Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    ASSERT_EQ(result.size(), 3);

    EXPECT_EQ(result.at(0).robot, enemy_robot_1);
    EXPECT_TRUE(result.at(0).has_ball);

    EXPECT_EQ(result.at(1).robot, enemy_robot_2);
    EXPECT_EQ(result.at(2).robot, enemy_robot_3);
    EXPECT_EQ(result.at(1).num_passes_to_get_possession, 1);
    EXPECT_EQ(result.at(2).num_passes_to_get_possession, 1);
    EXPECT_GT(result.at(1).goal_angle, result.at(2).goal_angle);
}

TEST(EnemyThreatTest, pass_data_is_copied_into_the_right_threat)
{
    // getAllEnemyThreats looks each enemy up in one shared pass search rather than
    // searching per enemy, so check every threat ends up with its own pass count and
    // passer. This cannot catch a bug in the search itself, since getNumPassesToRobot
    // is implemented by the same search
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(3.0, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(2.0, 1.5), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_3 =
        Robot(3, world->field().friendlyGoalCenter() + Vector(0.8, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_4 =
        Robot(4, world->field().friendlyGoalCenter() + Vector(2.0, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_1, enemy_robot_2, enemy_robot_3, enemy_robot_4});
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(
        world, enemy_robot_1.position() + Vector(-DIST_TO_FRONT_OF_ROBOT_METERS, 0),
        Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    ASSERT_EQ(result.size(), 4);
    ASSERT_TRUE(result.at(0).has_ball);
    Robot ball_carrier = result.at(0).robot;

    for (const auto& threat : result)
    {
        auto pass_data = getNumPassesToRobot(ball_carrier, threat.robot,
                                             world->enemyTeam(), world->friendlyTeam());

        ASSERT_TRUE(pass_data);
        EXPECT_EQ(threat.num_passes_to_get_possession, pass_data->first);
        EXPECT_EQ(threat.passer, pass_data->second);
    }
}

TEST(EnemyThreatTest, three_enemies_with_one_behind_the_goal_line)
{
    // Enemy robot 3 is behind the goal line, just outside the near post. It's closer to
    // both goalposts than enemy robot 2, so it would get the wider goal angle and come
    // out on top even though it can't shoot from there
    //
    //                    enemy robot 1 + ball
    //
    //                    enemy robot 2
    //
    //                  | friendly net |
    //                  ----------------
    //             enemy robot 3

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    Robot enemy_robot_1 =
        Robot(1, world->field().friendlyGoalCenter() + Vector(3.5, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_2 =
        Robot(2, world->field().friendlyGoalCenter() + Vector(1.5, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_3 =
        Robot(3, world->field().friendlyGoalCenter() + Vector(-0.15, 0.6), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_1, enemy_robot_2, enemy_robot_3});
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(
        world, enemy_robot_1.position() + Vector(-DIST_TO_FRONT_OF_ROBOT_METERS, 0),
        Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    ASSERT_EQ(result.size(), 3);

    EXPECT_EQ(result.at(0).robot, enemy_robot_1);
    EXPECT_TRUE(result.at(0).has_ball);

    // Both enemies can be reached in a single pass, so the tie is broken by their view
    // of the net
    EXPECT_EQ(result.at(1).robot, enemy_robot_2);
    EXPECT_EQ(result.at(2).robot, enemy_robot_3);
    EXPECT_EQ(result.at(1).num_passes_to_get_possession, 1);
    EXPECT_EQ(result.at(2).num_passes_to_get_possession, 1);

    EXPECT_EQ(result.at(2).goal_angle, Angle::zero());
    EXPECT_GT(result.at(1).goal_angle, Angle::zero());
}

TEST(EnemyThreatTest, enemies_in_and_behind_the_goal_mouth_have_no_view_of_the_net)
{
    // A robot needs to be at least its own radius ahead of the goal line before it has
    // a net to shoot at. Inside that, on the line, or behind it, the angle is zero
    //
    // Field always puts the friendly goal at negative x, so only that orientation can
    // be tested here

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    Robot clear_of_the_mouth =
        Robot(1, world->field().friendlyGoalCenter() + Vector(1.0, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot in_the_mouth = Robot(
        2, world->field().friendlyGoalCenter() + Vector(ROBOT_MAX_RADIUS_METERS / 2, 0),
        Vector(0, 0), Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot on_the_line =
        Robot(3, world->field().friendlyGoalCenter(), Vector(0, 0), Angle::half(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot behind_the_line =
        Robot(4, world->field().friendlyGoalCenter() + Vector(-0.05, 0), Vector(0, 0),
              Angle::half(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots(
        {clear_of_the_mouth, in_the_mouth, on_the_line, behind_the_line});
    world->updateEnemyTeamState(enemy_team);

    ::TestUtil::setBallPosition(world, world->field().enemyGoalCenter(),
                                Timestamp::fromSeconds(0));

    auto result = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                     world->enemyTeam(), world->ball(), false);

    ASSERT_EQ(result.size(), 4);

    std::map<RobotId, Angle> goal_angle_by_id;
    for (const auto& threat : result)
    {
        goal_angle_by_id[threat.robot.id()] = threat.goal_angle;
    }

    EXPECT_GT(goal_angle_by_id.at(clear_of_the_mouth.id()), Angle::zero());
    EXPECT_EQ(goal_angle_by_id.at(in_the_mouth.id()), Angle::zero());
    EXPECT_EQ(goal_angle_by_id.at(on_the_line.id()), Angle::zero());
    EXPECT_EQ(goal_angle_by_id.at(behind_the_line.id()), Angle::zero());
}

namespace
{
auto referencePassSearch(const std::vector<Robot>& passers,
                         const std::vector<Robot>& receivers,
                         const std::vector<Robot>& robots)
{
    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> result;
    for (const auto& passer : passers)
        for (const auto& receiver : receivers)
        {
            auto obstacles = robots;
            obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), passer),
                            obstacles.end());
            obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), receiver),
                            obstacles.end());
            if (!std::any_of(obstacles.begin(), obstacles.end(),
                             [passer, receiver](const Robot& obstacle)
                             {
                                 return intersects(
                                     Circle(obstacle.position(), ROBOT_MAX_RADIUS_METERS),
                                     Segment(passer.position(), receiver.position()));
                             }))
            {
                if (result.count(receiver) > 0)
                    result.at(receiver).emplace_back(passer);
                else
                    result.insert(std::make_pair(receiver, std::vector<Robot>{passer}));
            }
        }
    return result;
}

std::vector<std::vector<Robot>> passSearchLayouts()
{
    std::mt19937 random(6157);
    std::uniform_real_distribution<double> position(-4, 4);
    std::vector<std::vector<Robot>> layouts;
    for (int layout = 0; layout < 100; ++layout)
    {
        std::vector<Robot> robots;
        for (unsigned int id = 0; id < 12; ++id)
            robots.emplace_back(id, Point(position(random), position(random)), Vector(),
                                Angle::zero(), AngularVelocity::zero(), Timestamp());
        layouts.push_back(robots);
    }
    for (double offset : {0.0, ROBOT_MAX_RADIUS_METERS - 1e-9, ROBOT_MAX_RADIUS_METERS,
                          ROBOT_MAX_RADIUS_METERS + 1e-9})
    {
        std::vector<Robot> robots;
        for (const auto& point :
             {Point(0, 0), Point(2, 0), Point(1, offset), Point(0, offset)})
            robots.emplace_back(static_cast<unsigned int>(robots.size()), point, Vector(),
                                Angle::zero(), AngularVelocity::zero(), Timestamp());
        layouts.push_back(robots);
    }
    return layouts;
}
}  // namespace

TEST(FindAllPasserReceiverPairsTest, matches_previous_algorithm)
{
    EXPECT_TRUE(findAllReceiverPasserPairs({}, {}, {}).empty());
    for (auto robots : passSearchLayouts())
    {
        // Include an equal duplicate and a robot with the same ID but different state.
        robots.push_back(robots.front());
        robots.emplace_back(robots.front().id(), Point(0.4, -0.2), Vector(),
                            Angle::zero(), AngularVelocity::zero(), Timestamp());
        auto receivers = robots;
        std::reverse(receivers.begin(), receivers.end());
        EXPECT_EQ(findAllReceiverPasserPairs(robots, receivers, robots),
                  referencePassSearch(robots, receivers, robots));
    }
}

TEST(FindAllPasserReceiverPairsTest, DISABLED_benchmark)
{
    const auto layouts = passSearchLayouts();
    size_t consumed    = 0;
    auto measure       = [&](auto search)
    {
        auto start = std::chrono::steady_clock::now();
        for (int batch = 0; batch < 20; ++batch)
            for (const auto& robots : layouts)
                for (const auto& [receiver, passers] : search(robots, robots, robots))
                {
                    consumed += receiver.id();
                    for (const auto& passer : passers)
                        consumed += passer.id() + 1;
                }
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - start)
            .count();
    };
    measure(referencePassSearch);
    measure(findAllReceiverPasserPairs);
    for (int repeat = 0; repeat < 5; ++repeat)
        std::cout << "old_ms=" << measure(referencePassSearch)
                  << " new_ms=" << measure(findAllReceiverPasserPairs) << std::endl;
    std::cout << "consumed=" << consumed << std::endl;
}

TEST(EnemyThreatTest, shot_eligibility_boundaries_preserve_possession)
{
    auto world = TestUtil::createBlankTestingWorld();
    for (double offset :
         {-0.1, 0.0, ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS + 1e-9})
    {
        Point position = world->field().friendlyGoalCenter() + Vector(offset, 0);
        TestUtil::setEnemyRobotPositions(world, {position}, Timestamp());
        TestUtil::setBallPosition(world, position + Vector(0.09, 0), Timestamp());
        auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                          world->enemyTeam(), world->ball(), false);
        ASSERT_EQ(threats.size(), 1);
        EXPECT_TRUE(threats[0].has_ball);
        EXPECT_EQ(threats[0].num_passes_to_get_possession, 0);
        if (offset <= ROBOT_MAX_RADIUS_METERS)
        {
            EXPECT_EQ(threats[0].goal_angle, Angle::zero());
            EXPECT_FALSE(threats[0].best_shot_angle);
            EXPECT_FALSE(threats[0].best_shot_target);
        }
        else
            EXPECT_GT(threats[0].goal_angle, Angle::zero());
    }
}

TEST(EnemyThreatTest, DISABLED_evaluation_benchmark)
{
    const auto field   = Field::createSSLDivisionBField();
    const auto layouts = passSearchLayouts();
    size_t consumed    = 0;
    for (unsigned int team_size : {6u, 12u})
        for (double ball_speed : {0.0, 2.0})
        {
            std::vector<World> worlds;
            for (size_t i = 0; i < 16; ++i)
            {
                Team friendly(Duration::fromSeconds(1)), enemy(Duration::fromSeconds(1));
                std::vector<Robot> enemies(layouts[i].begin(),
                                           layouts[i].begin() + team_size);
                friendly.updateRobots(layouts[i + 16]);
                enemy.updateRobots(enemies);
                worlds.emplace_back(
                    field, Ball(Point(0.2, -0.3), Vector(ball_speed, 0), Timestamp()),
                    friendly, enemy);
            }
            auto evaluate = [&](const World& world)
            {
                auto threats = getAllEnemyThreats(field, world.friendlyTeam(),
                                                  world.enemyTeam(), world.ball(), false);
                for (const auto& threat : threats)
                {
                    consumed += threat.robot.id() +
                                static_cast<size_t>(threat.num_passes_to_get_possession);
                    if (threat.best_shot_target)
                        consumed += 1;
                }
            };
            auto measure = [&](auto operation)
            {
                auto start = std::chrono::steady_clock::now();
                for (int batch = 0; batch < 10; ++batch)
                    for (const auto& world : worlds)
                        operation(world);
                return std::chrono::duration<double, std::micro>(
                           std::chrono::steady_clock::now() - start)
                           .count() /
                       160;
            };
            measure(evaluate);
            for (int repeat = 0; repeat < 5; ++repeat)
                std::cout
                    << "evaluation_us size=" << team_size << " speed=" << ball_speed
                    << " total=" << measure(evaluate) << " possession="
                    << measure(
                           [&](const World& world)
                           {
                               auto robot = getRobotWithEffectiveBallPossession(
                                   world.enemyTeam(), world.ball(), field);
                               if (robot)
                                   consumed += robot->id();
                           })
                    << " shots="
                    << measure(
                           [&](const World& world)
                           {
                               for (const auto& robot : world.enemyTeam().getAllRobots())
                               {
                                   if (!canShootOnFriendlyGoal(field, robot.position()))
                                       continue;
                                   auto shot = calcBestShotOnGoal(
                                       field, world.friendlyTeam(), world.enemyTeam(),
                                       robot.position(), TeamType::FRIENDLY, {robot});
                                   if (shot)
                                       consumed += 1;
                               }
                           })
                    << std::endl;
        }
    std::cout << "evaluation_consumed=" << consumed << std::endl;
}

TEST(EnemyThreatTest, pass_counts_and_nearest_passers_match_independent_graph_search)
{
    auto layouts = passSearchLayouts();
    // A blocked direct pass with two equally close, two-pass predecessors.
    std::vector<Robot> diamond;
    for (const auto& position :
         {Point(-2, 0), Point(0, -1), Point(0, 1), Point(2, 0), Point(-1, 0)})
        diamond.emplace_back(static_cast<RobotId>(diamond.size()), position, Vector(),
                             Angle::zero(), AngularVelocity::zero(), Timestamp());
    layouts.push_back(diamond);
    unsigned int unreachable = 0, multiple_passes = 0;
    for (const auto& robots : layouts)
    {
        const auto pairs = referencePassSearch(robots, robots, robots);
        std::vector<std::vector<bool>> edges(robots.size(),
                                             std::vector<bool>(robots.size(), false));
        for (size_t p = 0; p < robots.size(); ++p)
            for (size_t r = 0; r < robots.size(); ++r)
            {
                const auto receiver = pairs.find(robots[r]);
                if (receiver != pairs.end())
                    edges[p][r] =
                        std::find(receiver->second.begin(), receiver->second.end(),
                                  robots[p]) != receiver->second.end();
            }
        Team team(Duration::fromSeconds(1)), other(Duration::fromSeconds(1));
        auto reversed = robots;
        std::reverse(reversed.begin(), reversed.end());
        team.updateRobots(reversed);
        for (size_t source = 0; source < robots.size(); ++source)
        {
            std::vector<int> passes(robots.size(), -1);
            std::vector<size_t> queue{source};
            passes[source] = 0;
            for (size_t head = 0; head < queue.size(); ++head)
                for (size_t receiver = 0; receiver < robots.size(); ++receiver)
                    if (edges[queue[head]][receiver] && passes[receiver] < 0)
                    {
                        passes[receiver] = passes[queue[head]] + 1;
                        queue.push_back(receiver);
                    }
            for (size_t receiver = 0; receiver < robots.size(); ++receiver)
            {
                auto actual =
                    getNumPassesToRobot(robots[source], robots[receiver], team, other);
                ASSERT_EQ(actual.has_value(), passes[receiver] >= 0);
                if (!actual)
                {
                    ++unreachable;
                    continue;
                }
                EXPECT_EQ(actual->first, passes[receiver]);
                std::optional<Robot> nearest;
                if (passes[receiver] > 0)
                    for (size_t p = 0; p < robots.size(); ++p)
                        if (edges[p][receiver] && passes[p] == passes[receiver] - 1 &&
                            (!nearest ||
                             (robots[p].position() - robots[receiver].position())
                                     .length() <
                                 (nearest->position() - robots[receiver].position())
                                     .length()))
                            nearest = robots[p];
                EXPECT_EQ(actual->second, nearest);
                if (passes[receiver] > 1)
                    ++multiple_passes;
            }
        }
    }
    EXPECT_GT(unreachable, 0);
    EXPECT_GT(multiple_passes, 0);
    Team team(Duration::fromSeconds(1)), other(Duration::fromSeconds(1));
    team.updateRobots(diamond);
    const auto tie = getNumPassesToRobot(diamond[0], diamond[3], team, other);
    ASSERT_TRUE(tie);
    EXPECT_EQ(tie->first, 2);
    ASSERT_TRUE(tie->second);
    EXPECT_EQ(tie->second->id(), 1);
}

namespace
{
EnemyThreat makeThreat(RobotId id, bool has_ball, double goal_angle_degrees,
                       int num_passes)
{
    Robot robot(id, Point(-1, id), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    return EnemyThreat{robot,
                       has_ball,
                       Angle::fromDegrees(goal_angle_degrees),
                       Angle::fromDegrees(goal_angle_degrees),
                       Point(-4.5, 0),
                       num_passes,
                       std::nullopt};
}
}  // namespace

TEST(SortEnemyThreatsTest, threats_equal_in_every_criterion_have_a_deterministic_order)
{
    auto threat_low_id  = makeThreat(2, false, 30, 1);
    auto threat_high_id = makeThreat(5, false, 30, 1);

    std::vector<EnemyThreat> threats = {threat_high_id, threat_low_id};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{threat_low_id, threat_high_id}));

    threats = {threat_low_id, threat_high_id};
    sortThreatsInDecreasingOrder(threats);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{threat_low_id, threat_high_id}));
}

TEST(StabilizePrimaryThreatTest, previous_primary_stays_when_tied_within_margin)
{
    auto leader   = makeThreat(0, false, 30, 1);
    auto previous = makeThreat(1, false, 28, 1);
    std::vector<EnemyThreat> threats = {leader, previous};

    stabilizePrimaryThreat(threats, 1, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{previous, leader}));
}

TEST(StabilizePrimaryThreatTest, previous_primary_promoted_from_the_back_keeps_the_rest)
{
    auto leader   = makeThreat(0, false, 30, 1);
    auto middle   = makeThreat(1, false, 29, 1);
    auto previous = makeThreat(2, false, 28, 1);
    std::vector<EnemyThreat> threats = {leader, middle, previous};

    stabilizePrimaryThreat(threats, 2, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{previous, leader, middle}));
}

TEST(StabilizePrimaryThreatTest, previous_primary_loses_spot_beyond_margin)
{
    auto leader   = makeThreat(0, false, 30, 1);
    auto previous = makeThreat(1, false, 20, 1);
    std::vector<EnemyThreat> threats = {leader, previous};

    stabilizePrimaryThreat(threats, 1, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));
}

TEST(StabilizePrimaryThreatTest, never_overrides_a_different_pass_distance)
{
    auto leader   = makeThreat(0, false, 30, 0);
    auto previous = makeThreat(1, false, 30, 1);
    std::vector<EnemyThreat> threats = {leader, previous};

    stabilizePrimaryThreat(threats, 1, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));
}

TEST(StabilizePrimaryThreatTest, never_overrides_ball_possession)
{
    auto leader   = makeThreat(0, true, 30, 0);
    auto previous = makeThreat(1, false, 30, 0);
    std::vector<EnemyThreat> threats = {leader, previous};

    stabilizePrimaryThreat(threats, 1, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));
}

TEST(StabilizePrimaryThreatTest, no_previous_or_missing_previous_changes_nothing)
{
    auto leader = makeThreat(0, false, 30, 1);
    auto other  = makeThreat(1, false, 30, 1);
    std::vector<EnemyThreat> threats = {leader, other};

    stabilizePrimaryThreat(threats, std::nullopt, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, other}));

    stabilizePrimaryThreat(threats, 7, Angle::fromDegrees(5));
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, other}));

    std::vector<EnemyThreat> empty;
    stabilizePrimaryThreat(empty, 0, Angle::fromDegrees(5));
    EXPECT_TRUE(empty.empty());
}

TEST(HoldPrimaryThreatTest, contesting_previous_primary_is_held_then_released)
{
    auto previous = makeThreat(0, false, 20, 0);
    auto leader   = makeThreat(1, false, 30, 0);
    PrimaryThreatHold state;
    std::optional<RobotId> primary = 0;

    // The first update after the previous primary was chosen: it has held 0 updates
    state.updates_held = 0;
    for (int i = 0; i < 3; i++)
    {
        std::vector<EnemyThreat> threats = {leader, previous};
        holdPrimaryThreat(threats, primary, state, 3);
        EXPECT_EQ(threats.front().robot.id(), 0) << i;
    }
    std::vector<EnemyThreat> threats = {leader, previous};
    holdPrimaryThreat(threats, primary, state, 3);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));
    EXPECT_EQ(state.updates_held, 0);
}

TEST(HoldPrimaryThreatTest, change_is_blocked_for_the_hold_after_a_switch)
{
    auto a = makeThreat(0, false, 30, 0);
    auto b = makeThreat(1, false, 20, 0);
    PrimaryThreatHold state;
    state.updates_held = 5;

    std::vector<EnemyThreat> threats = {b, a};
    holdPrimaryThreat(threats, 0, state, 5);  // held long enough: b takes over
    EXPECT_EQ(threats.front().robot.id(), 1);
    threats = {a, b};
    holdPrimaryThreat(threats, 1, state, 5);  // a wants it back right away: held
    EXPECT_EQ(threats.front().robot.id(), 1);
}

TEST(HoldPrimaryThreatTest, never_holds_a_primary_that_no_longer_contests_the_ball)
{
    auto previous = makeThreat(0, false, 30, 2);
    auto leader   = makeThreat(1, false, 20, 0);
    PrimaryThreatHold state;
    std::vector<EnemyThreat> threats = {leader, previous};
    holdPrimaryThreat(threats, 0, state, 50);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));
}

TEST(HoldPrimaryThreatTest, a_new_ball_carrier_takes_over_immediately)
{
    auto previous = makeThreat(0, false, 30, 0);
    auto leader   = makeThreat(1, true, 20, 0);
    PrimaryThreatHold state;
    std::vector<EnemyThreat> threats = {leader, previous};
    holdPrimaryThreat(threats, 0, state, 50);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));
}

TEST(HoldPrimaryThreatTest, disabled_or_missing_previous_changes_nothing)
{
    auto previous = makeThreat(0, false, 30, 0);
    auto leader   = makeThreat(1, false, 20, 0);
    PrimaryThreatHold state;
    std::vector<EnemyThreat> threats = {leader, previous};
    holdPrimaryThreat(threats, 0, state, 0);
    holdPrimaryThreat(threats, std::nullopt, state, 5);
    holdPrimaryThreat(threats, 9, state, 5);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{leader, previous}));

    std::vector<EnemyThreat> empty;
    holdPrimaryThreat(empty, 0, state, 5);
    EXPECT_TRUE(empty.empty());
}

TEST(HoldPrimaryThreatTest, a_previous_primary_one_pass_away_still_contests_the_ball)
{
    auto previous = makeThreat(0, false, 20, 1);
    auto leader   = makeThreat(1, false, 30, 0);
    PrimaryThreatHold state;
    state.updates_held = 0;
    std::vector<EnemyThreat> threats = {leader, previous};
    holdPrimaryThreat(threats, 0, state, 4);
    EXPECT_EQ(threats, (std::vector<EnemyThreat>{previous, leader}));
    EXPECT_EQ(state.updates_held, 1);
}

TEST(EnemyThreatTest, previous_baller_with_margin_keeps_the_primary_threat)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(world, {Point(-1, 0), Point(0.9, 0)},
                                       Timestamp::fromSeconds(0));
    const auto get = [&](std::optional<RobotId> previous, double margin_s,
                         std::optional<RobotId>* baller)
    {
        return getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                  world->enemyTeam(), world->ball(), true, previous,
                                  Duration::fromSeconds(margin_s), baller);
    };

    // The default arguments behave exactly like no previous baller and no margin
    const auto defaults = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                             world->enemyTeam(), world->ball(), true);
    std::optional<RobotId> baller;
    EXPECT_EQ(get(std::nullopt, 0, &baller), defaults);
    EXPECT_EQ(baller, 1);
    EXPECT_EQ(defaults.front().robot.id(), 1);

    // A previous baller without a margin does not change anything
    EXPECT_EQ(get(0, 0, &baller), defaults);
    EXPECT_EQ(baller, 1);

    // With a margin the farther previous baller stays the primary threat
    const auto kept = get(0, 1.0, &baller);
    EXPECT_EQ(baller, 0);
    EXPECT_EQ(kept.front().robot.id(), 0);
    EXPECT_EQ(kept.front().num_passes_to_get_possession, 0);

    // A previous baller that is not on the enemy team is ignored
    EXPECT_EQ(get(7, 1.0, &baller), defaults);
    EXPECT_EQ(baller, 1);
}

TEST(HoldPrimaryThreatTest, a_hold_of_one_update_blocks_exactly_one_update)
{
    auto a = makeThreat(0, false, 30, 0);
    auto b = makeThreat(1, false, 20, 0);
    PrimaryThreatHold state;
    state.updates_held = 1;

    std::vector<EnemyThreat> threats = {b, a};
    holdPrimaryThreat(threats, 0, state, 1);  // b takes over
    EXPECT_EQ(threats.front().robot.id(), 1);
    threats = {a, b};
    holdPrimaryThreat(threats, 1, state, 1);  // a is held once
    EXPECT_EQ(threats.front().robot.id(), 1);
    threats = {a, b};
    holdPrimaryThreat(threats, 1, state, 1);  // then takes over
    EXPECT_EQ(threats.front().robot.id(), 0);
}

TEST(HoldPrimaryThreatTest, an_unchanged_primary_saturates_the_hold_counter)
{
    auto a = makeThreat(0, false, 30, 0);
    auto b = makeThreat(1, false, 20, 0);
    PrimaryThreatHold state;
    for (int i = 0; i < 100; i++)
    {
        std::vector<EnemyThreat> threats = {a, b};
        holdPrimaryThreat(threats, 0, state, 5);
    }
    EXPECT_EQ(state.updates_held, 5);

    // So the next real change is not held
    std::vector<EnemyThreat> threats = {b, a};
    holdPrimaryThreat(threats, 0, state, 5);
    EXPECT_EQ(threats.front().robot.id(), 1);
}

TEST(HoldPrimaryThreatTest, a_previous_primary_two_passes_away_does_not_contest_the_ball)
{
    auto previous = makeThreat(0, false, 30, 2);
    auto leader   = makeThreat(1, false, 20, 1);
    PrimaryThreatHold state;
    std::vector<EnemyThreat> threats = {leader, previous};
    holdPrimaryThreat(threats, 0, state, 5);
    EXPECT_EQ(threats.front().robot.id(), 1);
}
