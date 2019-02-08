//
// Created by roark on 07/02/19.
//

#include "ai/hl/stp/evaluation/deflect_off_enemy_target.h"

#include <gtest/gtest.h>

#include "geom/util.h"


TEST(DeflectOffEnemyTargetTest, deflect_off_enemy_target_test)
{
    auto test_ball = new Ball(Point(-1, 0), Vector(0, 0), Timestamp::fromMilliseconds(0));
    auto test_field    = new Field(9.0f, 6.0f, 1.0f, 2.5f, 1.0f, 1.4f, 1.0f);
    auto friendly_team = new Team(Duration::fromMilliseconds(0));
    auto enemy_team    = new Team(Duration::fromMilliseconds(0));

    Robot *test_robot =
        new Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromMilliseconds(0));

    Robot *enemy_robot =
        new Robot(0, Point(-3, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromMilliseconds(0));

    std::vector<Robot> friendly_robots;
    friendly_robots.emplace_back(*test_robot);
    friendly_team->updateRobots(friendly_robots);

    std::vector<Robot> enemy_robots;
    enemy_robots.emplace_back(*enemy_robot);
    enemy_team->updateRobots(enemy_robots);

    auto *test_world = new World(*test_field, *test_ball, *friendly_team, *enemy_team);
    using namespace Evaluation;

    Point p = Evaluation::deflect_off_enemy_target(*test_world);

    EXPECT_EQ(p, Point(-3, -0.1125));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
