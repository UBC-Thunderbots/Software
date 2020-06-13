#include "software/ai/evaluation/deflect_off_enemy_target.h"

#include <gtest/gtest.h>

#include "software/geom/util.h"
#include "software/test_util/test_util.h"


TEST(DeflectOffEnemyTargetTest, deflect_off_enemy_target_test)
{
    Robot friendly_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromMilliseconds(0));
    Robot enemy_robot    = Robot(0, Point(-3, 0), Vector(0, 0), Angle::zero(),
                              AngularVelocity::zero(), Timestamp::fromMilliseconds(0));
    std::vector<Robot> friendly_robots;
    std::vector<Robot> enemy_robots;
    friendly_robots.emplace_back(friendly_robot);
    enemy_robots.emplace_back(enemy_robot);

    World test_world = TestUtil::createBlankTestingWorld();
    test_world.mutableFriendlyTeam().updateRobots(friendly_robots);
    test_world.mutableEnemyTeam().updateRobots(enemy_robots);
    TestUtil::setBallPosition(test_world, Point(-1, 0), Timestamp::fromMilliseconds(0));

    Point p = deflect_off_enemy_target(test_world);

    EXPECT_EQ(p, Point(-3, -0.0675));
}
