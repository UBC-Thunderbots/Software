#include "software/ai/evaluation/keep_away.h"

#include <gtest/gtest.h>

#include "software/ai/passing/cost_function.h"
#include "software/test_util/test_util.h"
#include "software/world/team.h"

class KeepAwayTest : public testing::Test
{
   protected:
    KeepAwayTest()
        : world(TestUtil::createBlankTestingWorld()),
          field_bounds(world->field().fieldBoundary())
    {
        passing_config.set_pass_delay_sec(0.0);
    }

    TbotsProto::PassingConfig passing_config;
    std::shared_ptr<World> world;
    Rectangle field_bounds;
};
TEST_F(KeepAwayTest, test_keep_away_cost_no_enemies)
{
    Team enemy_team;
    Pass pass(Point(0, 0), Point(1, 1), 5);
    auto result = rateKeepAwayPosition(pass.passerPoint(), *world, pass, field_bounds,
                                       passing_config);
    EXPECT_GT(result, 0.99);
}

TEST_F(KeepAwayTest, test_keep_away_cost_interception)
{
    Point ball_position(-1, 0);
    Point enemy_robot_position(0, 0);
    TestUtil::setEnemyRobotPositions(world, {enemy_robot_position},
                                     Timestamp::fromSeconds(0));
    TestUtil::setBallPosition(world, ball_position, Timestamp::fromSeconds(0));

    // Keep away points are rated based on points around the ball that increase the best
    // pass rating
    Pass best_pass(ball_position, Point(1, 0), 5);

    // Pass from new keep away point is still blocked by enemy
    Point keep_away_point_bad(-0.5, 0);
    auto lower_result = rateKeepAwayPosition(keep_away_point_bad, *world, best_pass,
                                             field_bounds, passing_config);

    // Pass from new keep away point is NOT blocked by enemy
    Point keep_away_point_good(-1, 0.5);
    auto higher_result = rateKeepAwayPosition(keep_away_point_good, *world, best_pass,
                                              field_bounds, passing_config);

    EXPECT_GT(higher_result, lower_result);
}

TEST_F(KeepAwayTest, test_keep_away_cost_proximity)
{
    TestUtil::setEnemyRobotPositions(world, {Point(0, 0)}, Timestamp::fromSeconds(0));
    TestUtil::setBallPosition(world, Point(1, 0), Timestamp::fromSeconds(0));

    Pass pass(Point(1, 0), Point(3, 0), 5);
    Point keep_away_point_closer_to_enemy(0.3, 0);
    auto lower_result = rateKeepAwayPosition(keep_away_point_closer_to_enemy, *world,
                                             pass, field_bounds, passing_config);

    Point keep_away_point_farther_from_enemy(0.6, 0);
    auto higher_result = rateKeepAwayPosition(keep_away_point_farther_from_enemy, *world,
                                              pass, field_bounds, passing_config);

    EXPECT_GT(higher_result, lower_result);
}

TEST_F(KeepAwayTest, test_keep_away_no_enemies)
{
    auto world = TestUtil::createBlankTestingWorld();
    Pass pass(Point(0, 0), Point(1, 1), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(*world, pass, passing_config);
    // gradient should be 0 everywhere
    EXPECT_EQ(keep_away_pt, Point(0, 0));
}

TEST_F(KeepAwayTest, test_keep_away_point_interception)
{
    auto world          = TestUtil::createBlankTestingWorld();
    auto new_enemy_team = TestUtil::setRobotPositionsHelper(
        world->enemyTeam(), {Point(0, 0)}, Timestamp::fromSeconds(0));
    world->updateEnemyTeamState(new_enemy_team);

    Point ball_point(-1, 0.1);
    world->updateBall(Ball(ball_point, Vector(), Timestamp()));
    Pass pass(ball_point, Point(1, 0.1), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(*world, pass, passing_config);
    Pass new_pass(keep_away_pt, pass.receiverPoint(), pass.speed());
    EXPECT_GT(calculateInterceptRisk(world->enemyTeam(), pass, passing_config),
              calculateInterceptRisk(world->enemyTeam(), new_pass, passing_config));
}

TEST_F(KeepAwayTest, test_keep_away_point_proximity)
{
    auto world          = TestUtil::createBlankTestingWorld();
    auto new_enemy_team = TestUtil::setRobotPositionsHelper(
        world->enemyTeam(), {Point(0, 0)}, Timestamp::fromSeconds(0));
    world->updateEnemyTeamState(new_enemy_team);

    Point ball_point(-0.25, 0.1);
    world->updateBall(Ball(ball_point, Vector(), Timestamp()));
    Pass pass(ball_point, Point(1, 0.1), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(*world, pass, passing_config);

    EXPECT_GT(
        // this function is extensively tested as part of the pass cost functions, so
        // I'm fine with it being used to pass/fail a test here even though it is
        // used as part of the keep away cost function
        calculateProximityRisk(ball_point, world->enemyTeam(), passing_config),
        calculateProximityRisk(keep_away_pt, world->enemyTeam(), passing_config));
}

TEST_F(KeepAwayTest, test_keep_away_point_field_lines)
{
    auto world           = TestUtil::createBlankTestingWorld();
    auto top_left_corner = world->field().fieldLines().negXPosYCorner();
    Pass pass(top_left_corner, Point(0, 0), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(*world, pass, passing_config);
    auto field_center = world->field().fieldLines().centre();
    // the keep away point should be closer to the field center
    EXPECT_LT((keep_away_pt - field_center).length(),
              (top_left_corner - field_center).length());
}

TEST_F(KeepAwayTest, test_keep_away_point_field_lines_2)
{
    auto world           = TestUtil::createBlankTestingWorld();
    auto top_left_corner = world->field().fieldLines().negXPosYCorner();
    auto top_mid_point   = Point(0, top_left_corner.y());

    Pass pass(top_mid_point, Point(0, 0), 5);
    auto keep_away_pt = findKeepAwayTargetPoint(*world, pass, passing_config);
    auto field_center = world->field().fieldLines().centre();
    // the keep away point should be closer to the field center
    EXPECT_LT((keep_away_pt - field_center).length(),
              (top_mid_point - field_center).length());
}
