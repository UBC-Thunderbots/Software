#include "software/ai/evaluation/keep_away.h"

#include <gtest/gtest.h>

#include "software/ai/passing/cost_function.h"
#include "software/test_util/test_util.h"
#include "software/world/team.h"

TEST(KeepAwayCostTest, test_keep_away_cost_no_enemies)
{
    Team enemy_team;
    Pass pass(Point(0, 0), Point(1, 1), 5);
    auto result = ratePasserPointForKeepAway(pass, enemy_team);
    EXPECT_GT(result, 0.99);
}

TEST(KeepAwayCostTest, test_keep_away_cost_interception)
{
    Team enemy_team;
    enemy_team = TestUtil::setRobotPositionsHelper(enemy_team, {Point(0, 0)},
                                                   Timestamp::fromSeconds(0));

    Pass bad_pass(Point(1, 0), Point(-1, 0), 5);
    auto lower_result = ratePasserPointForKeepAway(bad_pass, enemy_team);

    Pass good_pass(Point(-1, 1), Point(-1, 0), 5);
    auto higher_result = ratePasserPointForKeepAway(good_pass, enemy_team);

    EXPECT_GT(higher_result, lower_result);
}

TEST(KeepAwayCostTest, test_keep_away_cost_proximity)
{
    Team enemy_team;
    enemy_team = TestUtil::setRobotPositionsHelper(enemy_team, {Point(0, 0)},
                                                   Timestamp::fromSeconds(0));

    Pass bad_pass(Point(0.25, 0), Point(1, 0), 5);
    auto lower_result = ratePasserPointForKeepAway(bad_pass, enemy_team);

    Pass good_pass(Point(2, 0), Point(1, 0), 5);
    auto higher_result = ratePasserPointForKeepAway(good_pass, enemy_team);

    EXPECT_GT(higher_result, lower_result);
}

TEST(FindKeepAwayPointTest, test_keep_away_no_enemies)
{
    auto world = TestUtil::createBlankTestingWorld();
    Pass pass(Point(0, 0), Point(1, 1), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(world, pass);
    // gradient should be 0 everywhere
    EXPECT_EQ(keep_away_pt, Point(0, 0));
}

TEST(FindKeepAwayPointTest, test_keep_away_point_interception)
{
    auto world          = TestUtil::createBlankTestingWorld();
    auto new_enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), {Point(0, 0)}, Timestamp::fromSeconds(0));
    world.updateEnemyTeamState(new_enemy_team);

    Point ball_point(-1, 0.1);
    world.updateBall(Ball(ball_point, Vector(), Timestamp()));
    Pass pass(ball_point, Point(1, 0.1), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(world, pass);
    Pass new_pass(keep_away_pt, pass.receiverPoint(), pass.speed());
    EXPECT_GT(
        calculateInterceptRisk(world.enemyTeam(), pass, Duration::fromSeconds(0)),
        calculateInterceptRisk(world.enemyTeam(), new_pass, Duration::fromSeconds(0)));
}

TEST(FindKeepAwayPointTest, test_keep_away_point_proximity)
{
    auto world          = TestUtil::createBlankTestingWorld();
    auto new_enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), {Point(0, 0)}, Timestamp::fromSeconds(0));
    world.updateEnemyTeamState(new_enemy_team);

    Point ball_point(-0.25, 0.1);
    world.updateBall(Ball(ball_point, Vector(), Timestamp()));
    Pass pass(ball_point, Point(1, 0.1), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(world, pass);

    // copypasted from software/ai/evaluation/keep_away.cpp
    static constexpr double PASSER_ENEMY_PROXIMITY_IMPORTANCE = 1.5;

    EXPECT_GT(
        // this function is extensively tested as part of the pass cost functions, so
        // I'm fine with it being used to pass/fail a test here even though it is
        // used as part of the keep away cost function
        calculateProximityRisk(ball_point, world.enemyTeam(),
                               PASSER_ENEMY_PROXIMITY_IMPORTANCE),
        calculateProximityRisk(keep_away_pt, world.enemyTeam(),
                               PASSER_ENEMY_PROXIMITY_IMPORTANCE));
}

TEST(FindKeepAwayPointTest, test_keep_away_point_field_lines)
{
    auto world           = TestUtil::createBlankTestingWorld();
    auto top_left_corner = world.field().fieldLines().negXPosYCorner();
    Pass pass(top_left_corner, Point(0, 0), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(world, pass);
    auto field_center = world.field().fieldLines().centre();
    // the keep away point should be closer to the field center
    EXPECT_LT((keep_away_pt - field_center).length(),
              (top_left_corner - field_center).length());
}

TEST(FindKeepAwayPointTest, test_keep_away_point_field_lines_2)
{
    auto world           = TestUtil::createBlankTestingWorld();
    auto top_left_corner = world.field().fieldLines().negXPosYCorner();
    auto top_mid_point   = Point(0, top_left_corner.y());

    Pass pass(top_mid_point, Point(0, 0), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(world, pass);
    auto field_center = world.field().fieldLines().centre();
    // the keep away point should be closer to the field center
    EXPECT_LT((keep_away_pt - field_center).length(),
              (top_mid_point - field_center).length());
}
