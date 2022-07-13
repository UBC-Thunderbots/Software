
#include "software/ai/evaluation/find_open_areas.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(FindOpenAreasTest, find_best_chip_with_enemy)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));
    Field field        = Field::createSSLDivisionBField();
    Ball ball = Ball(field.friendlyGoalCenter(), Vector(0, 0), Timestamp::fromSeconds(1));

    Robot friendly_goalie =
        Robot(0, field.friendlyGoalCenter(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot = Robot(1, Point(0.9, 0.9), Vector(0, 0), Angle::zero(),
                              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    friendly_team.updateRobots(std::vector<Robot>({friendly_goalie}));
    enemy_team.updateRobots(std::vector<Robot>({enemy_robot}));

    World world = World(field, ball, friendly_team, enemy_team);

    Rectangle clear_area = Rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Circle> chip_targets = findGoodChipTargets(world, clear_area);

    ASSERT_NE(0, chip_targets.size());
    // assume circles are correctly placed, see tests in find_open_circles_test.cpp
}

TEST(FindOpenAreasTest, find_best_chip_without_enemies)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));
    Field field        = Field::createSSLDivisionBField();
    Ball ball          = Ball(Point(0, 0), Vector(-1, 0), Timestamp::fromSeconds(1));

    Robot friendly_goalie =
        Robot(0, field.friendlyGoalCenter(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    friendly_team.updateRobots(std::vector<Robot>({friendly_goalie}));

    World world = World(field, ball, friendly_team, enemy_team);

    Rectangle clear_area = Rectangle(Point(1, -2), Point(2, 2));

    std::vector<Circle> chip_targets = findGoodChipTargets(world, clear_area);

    ASSERT_EQ(0, chip_targets.size());
}

TEST(FindOpenAreasTest, find_best_chip_with_no_good_options)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));
    Field field        = Field::createSSLDivisionBField();
    Ball ball          = Ball(Point(0, 0), Vector(-1, 0), Timestamp::fromSeconds(1));

    World world = World(field, ball, friendly_team, enemy_team);

    Robot friendly_goalie =
        Robot(0, field.friendlyGoalCenter(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Robot> enemy_robots;
    std::vector<Point> enemy_robot_positions = {Point(-2, -1), Point(3, -2),
                                                Point(-1.1, -1.1)};

    for (unsigned int i = 0; i < enemy_robot_positions.size(); ++i)
    {
        Point point       = enemy_robot_positions[i];
        Robot enemy_robot = Robot(i, point, Vector(0, 0), Angle::zero(),
                                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
        enemy_robots.emplace_back(enemy_robot);
    }

    friendly_team.updateRobots(std::vector<Robot>({friendly_goalie}));
    enemy_team.updateRobots(enemy_robots);

    Rectangle clear_area = Rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Circle> chip_targets = findGoodChipTargets(world, clear_area);

    ASSERT_EQ(0, chip_targets.size());
}
