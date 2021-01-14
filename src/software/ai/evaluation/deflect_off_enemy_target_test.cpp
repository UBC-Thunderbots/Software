#include "software/ai/evaluation/deflect_off_enemy_target.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"


TEST(DeflectOffEnemyTargetTest, deflectOffEnemyTarget_test)
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
    test_world.updateFriendlyTeamState(Team(friendly_robots));
    test_world.updateEnemyTeamState(Team(enemy_robots));
    TestUtil::setBallPosition(test_world, Point(-1, 0), Timestamp::fromMilliseconds(0));

    Point p = deflectOffEnemyTarget(test_world);

    EXPECT_EQ(p, Point(-3, -0.0675));
}
