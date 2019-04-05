#include "ai/hl/stp/evaluation/enemy_threat.h"

#include <gtest/gtest.h>

#include "test/test_util/test_util.h"

TEST(GetNumPassesToRobotTest, robot_passing_to_itself)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot_1 = Robot(1, Point(-5, -5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team     = Team(Duration::fromSeconds(1));
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});

    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_0,
                                                  friendly_team, {});

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

    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_1,
                                                  friendly_team, {});

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    // Robot 0 should be able to pass to robot 1 in a single pass
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

    // The enemy robot is blocking the pass from robot 0 to robot 2, so we expect an
    // intermediate pass via robot 1
    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_2,
                                                  friendly_team, {enemy_robot_0});

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    // Robot 0 should be able to pass to robot 1 in a single pass
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

    // The only way for robot 3 to get the ball is to receive a pass from
    // robot 0 -> robot 1 -> robot 2 -> robot 3
    auto result =
        Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_3, friendly_team,
                                        {enemy_robot_0, enemy_robot_1, enemy_robot_2});

    // A valid result should have been found
    EXPECT_TRUE(result);

    int num_passes              = result.value().first;
    std::optional<Robot> passer = result.value().second;

    // Robot 0 should be able to pass to robot 1 in a single pass
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

    auto result = Evaluation::getNumPassesToRobot(friendly_robot_0, friendly_robot_1,
                                                  friendly_team, {enemy_robot_0});

    // We don't expect any pass info to be returned
    EXPECT_FALSE(result);
}
