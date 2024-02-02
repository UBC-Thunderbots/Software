#include "software/ai/evaluation/enemy_threat.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/test_util/test_util.h"

TEST(FindAllPasserReceiverPairsTest, robot_passing_to_itself)
{
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

    // Despite robot2 having better shooting and scoring opporunity, robot1 has the ball
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
    EXPECT_TRUE(TestUtil::equalWithinTolerance(*threat.best_shot_target,
                                               world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat.num_passes_to_get_possession, 0);
    ASSERT_FALSE(threat.passer);
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
    // Finally, enemy robot 3 is the least threatening because it would take 2 passes to
    // get the ball, and doesn't have a great angle on the goal because it's off to
    // the side

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
    EXPECT_TRUE(TestUtil::equalWithinTolerance(*threat_0.best_shot_target,
                                               world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat_0.num_passes_to_get_possession, 0);
    ASSERT_FALSE(threat_0.passer);

    auto threat_1 = result.at(1);
    EXPECT_EQ(threat_1.robot, enemy_robot_2);
    EXPECT_FALSE(threat_1.has_ball);
    EXPECT_NEAR(threat_1.goal_angle.toDegrees(), 20, 5);
    ASSERT_TRUE(threat_1.best_shot_angle);
    EXPECT_NEAR(threat_1.best_shot_angle->toDegrees(), 20, 5);
    ASSERT_TRUE(threat_1.best_shot_target);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(*threat_1.best_shot_target,
                                               world->field().friendlyGoalCenter(), 0.05));
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
    EXPECT_TRUE(TestUtil::equalWithinTolerance(*threat_2.best_shot_target,
                                               world->field().friendlyGoalCenter(), 0.05));
    EXPECT_EQ(threat_2.num_passes_to_get_possession, 1);
    ASSERT_TRUE(threat_2.passer);
    EXPECT_EQ(threat_2.passer, enemy_robot_1);
}
