#include "ai/hl/stp/evaluation/enemy_threat.h"

#include <gtest/gtest.h>

#include "test/test_util/test_util.h"

TEST(FindAllPasserReceiverPairsTest, robot_passing_to_itself)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    auto result = Evaluation::findAllReceiverPasserPairs(
        {friendly_robot_0}, {friendly_robot_0}, {friendly_robot_0});

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

    auto result = Evaluation::findAllReceiverPasserPairs({friendly_robot_0},
                                                         {friendly_robot_1}, all_robots);

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

    auto result = Evaluation::findAllReceiverPasserPairs(
        {friendly_robot_0, friendly_robot_1}, {friendly_robot_2}, all_robots);

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

    auto result = Evaluation::findAllReceiverPasserPairs({friendly_robot_0},
                                                         {friendly_robot_1}, all_robots);

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

    auto result = Evaluation::findAllReceiverPasserPairs(
        {friendly_robot_0, friendly_robot_1}, {friendly_robot_2}, all_robots);

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

    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_0,
                                                  friendly_team, enemy_team);

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(0, num_passes);
    EXPECT_FALSE(passer);
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
    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_1,
                                                  friendly_team, enemy_team);

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(1, num_passes);
    EXPECT_TRUE(passer);
    EXPECT_EQ(passer.value(), friendly_robot_0);
}

TEST(GetNumPassesToRobotTest, two_passes_around_a_single_obstacle)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(3, 1.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_2 = Robot(2, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1, friendly_robot_2});

    Robot enemy_robot_0 = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team     = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_0});

    // The enemy robot is blocking the pass from robot 0 to robot 2, so we expect an
    // intermediate pass via robot 1
    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_2,
                                                  friendly_team, enemy_team);

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(2, num_passes);
    EXPECT_TRUE(passer);
    EXPECT_EQ(passer.value(), friendly_robot_1);
}

TEST(GetNumPassesToRobotTest, multiple_friendly_robots_and_blocking_enemies)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(1, 1.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_2 = Robot(2, Point(3.5, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_3 = Robot(3, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots(
        {friendly_robot_0, friendly_robot_1, friendly_robot_2, friendly_robot_3});

    // Blocks the pass between robot 0 and robot 3
    Robot enemy_robot_0 = Robot(0, Point(4, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Blocks the pass between robot 1 and 3
    Robot enemy_robot_1 = Robot(1, Point(1.25, 1.4), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Blocks the pass between robot 0 and 2
    Robot enemy_robot_2 = Robot(2, Point(1.75, -1), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team     = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1, enemy_robot_2});

    // The only way for robot 3 to get the ball is to receive a pass from
    // robot 0 -> robot 1 -> robot 2 -> robot 3
    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_3,
                                                  friendly_team, enemy_team);

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(3, num_passes);
    EXPECT_TRUE(passer);
    EXPECT_EQ(passer.value(), friendly_robot_2);
}

TEST(GetNumPassesToRobotTest, all_passes_blocked)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});

    Robot enemy_robot_0 = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team     = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_0});

    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_1,
                                                  friendly_team, enemy_team);

    // We don't expect any pass info to be returned
    EXPECT_FALSE(result);
}

TEST(GetNumPassesToRobotTest, final_receiver_can_receive_passes_from_multiple_robots)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(1, 1.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_2 = Robot(2, Point(3.5, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_3 = Robot(3, Point(5, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots(
        {friendly_robot_0, friendly_robot_1, friendly_robot_2, friendly_robot_3});

    // Blocks the pass between robot 0 and robot 3
    Robot enemy_robot_0 = Robot(0, Point(4, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy_team     = Team(Duration::fromSeconds(1));
    enemy_team.updateRobots({enemy_robot_0});

    // robot 3 can receive the ball from either:
    // robot 0 -> robot 1 -> robot 3
    // or
    // robot 0 -> robot 2 -> robot 3
    // Robot 2 is closer to robot 3 so we expect it to be the most likely passer
    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_3,
                                                  friendly_team, enemy_team);

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    EXPECT_EQ(2, num_passes);
    EXPECT_TRUE(passer);
    EXPECT_EQ(passer.value(), friendly_robot_2);
}
