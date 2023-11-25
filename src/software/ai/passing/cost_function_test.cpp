#include "software/ai/passing/cost_function.h"

#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/math/math_functions.h"
#include "software/test_util/test_util.h"

class PassingEvaluationTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        entire_field =
            std::make_shared<Rectangle>(Field::createSSLDivisionBField().fieldLines());
        passing_config.set_min_pass_speed_m_per_s(3.5);
        passing_config.set_max_pass_speed_m_per_s(5.5);
        avg_desired_pass_speed = 3.9;
    }

    double avg_desired_pass_speed;

    std::shared_ptr<Rectangle> entire_field;
    TbotsProto::PassingConfig passing_config;
};

// This test is disabled to speed up CI, it can be enabled by removing "DISABLED_" from
// the test name
TEST_F(PassingEvaluationTest, DISABLED_ratePass_speed_test)
{
    // This test does not assert anything. Rather, It can be used to gauge how
    // fast ratePass is running, and can be profiled in order to find areas
    // of improvement for ratePass

    const int num_passes_to_gen = 1000;

    World world = ::TestUtil::createBlankTestingWorld();

    world.updateEnemyTeamState(Team(
        {
            Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(2, {0, 1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(3, {1.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(4, {0, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(5, {2.5, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(6, {3, -3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
        },
        Duration::fromSeconds(10)));
    world.updateFriendlyTeamState(Team(
        {
            Robot(0, {-0.2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(1, {-1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(2, {0, 1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(3, {-1.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(4, {0, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(5, {-2.5, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(6, {-3, -3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
        },
        Duration::fromSeconds(10)));

    std::uniform_real_distribution x_distribution(-world.field().xLength() / 2,
                                                  world.field().xLength() / 2);
    std::uniform_real_distribution y_distribution(-world.field().yLength() / 2,
                                                  world.field().yLength() / 2);

    std::uniform_real_distribution speed_distribution(
        passing_config.min_pass_speed_m_per_s(), passing_config.max_pass_speed_m_per_s());

    std::vector<Pass> passes;

    std::mt19937 random_num_gen;
    for (int i = 0; i < num_passes_to_gen; i++)
    {
        Point passer_point(x_distribution(random_num_gen),
                           y_distribution(random_num_gen));
        Point receiver_point(x_distribution(random_num_gen),
                             y_distribution(random_num_gen));
        double pass_speed = speed_distribution(random_num_gen);

        Pass p(passer_point, receiver_point, pass_speed);
        passes.emplace_back(p);
    }

    auto start_time = std::chrono::system_clock::now();
    for (auto pass : passes)
    {
        ratePass(world, pass, *entire_field, passing_config);
    }

    double duration_ms = ::TestUtil::millisecondsSince(start_time);
    double avg_ms      = duration_ms / static_cast<double>(num_passes_to_gen);

    // At the time of this test's creation (PR #695), ratePass ran at an average 0.105ms
    // in debug on an i7
    std::cout << "Took " << duration_ms << "ms to run, average time of " << avg_ms << "ms"
              << std::endl;
}

TEST_F(PassingEvaluationTest, ratePass_enemy_directly_on_pass_trajectory)
{
    // A pass from halfway up the +y side of the field to the origin.
    // There is an enemy defender right on the pass trajectory
    Pass pass({2, 2}, {0, 0}, passing_config.max_pass_speed_m_per_s() - 0.2);

    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(1, {-0.1, -0.1}, {0, 0}, pass.receiverOrientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {1, 1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_GE(pass_rating, 0.0);
    EXPECT_LE(pass_rating, 0.11);
}

TEST_F(PassingEvaluationTest, ratePass_one_friendly_marked_and_one_friendly_free)
{
    // A pass from halfway up the +y side of the field to the origin.
    // There is a defender closely marking one friendly robot on the field, but the
    // friendly robot at the origin is free.
    Pass pass({2, 2}, {0, 0}, avg_desired_pass_speed);

    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {3, -0.8}, {0, 0}, pass.receiverOrientation(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {-0.1, -0.1}, {0, 0}, pass.receiverOrientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {3.2, -0.8}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_GE(pass_rating, 0.65);
    EXPECT_LE(pass_rating, 0.9);
}

TEST_F(PassingEvaluationTest, ratePass_only_friendly_marked)
{
    // A pass from the +y side of the field to the -y side of the field, roughly 1/2 way
    // up the enemy half of the field. There is a defender closely marking the only
    // friendly robot on the field.
    Pass pass({2, 2}, {1, -1}, passing_config.max_pass_speed_m_per_s() - 0.2);

    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {1, -0.8}, {0, 0}, pass.receiverOrientation(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {1.5, -0.8}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_GE(pass_rating, 0.0);
    EXPECT_LE(pass_rating, 0.02);
}

TEST_F(PassingEvaluationTest, ratePass_cross_over_enemy_goal_defender_somewhat_near_pass)
{
    // A pass from the +y side of the field to the -y side of the field,
    // roughly 1/2 way up the enemy half of the field. There is a defender somewhat close
    // to the pass, but not close enough to get there in time.
    Pass pass({2, 2}, {1, -1}, avg_desired_pass_speed);

    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {1, -0.8}, {0, 0}, pass.receiverOrientation(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {4, 0.5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_GE(pass_rating, 0.5);
    EXPECT_LE(pass_rating, 1.0);
}

TEST_F(PassingEvaluationTest, ratePass_cross_over_enemy_net_goalie_in_net)
{
    // A pass from the +y side of the field to the -y side of the field,
    // roughly 1/2 way up the enemy half of the field, with a goalie in the net, but off
    // to the positive side. We also pass as soon as we can
    Pass pass({2, 2}, {1, -1}, avg_desired_pass_speed);

    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {1, -0.8}, {0, 0}, pass.receiverOrientation(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, world.field().enemyGoalCenter() + Vector(0, 0.5), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);

    EXPECT_GE(pass_rating, 0.68);
    EXPECT_LE(pass_rating, 0.9);
}

TEST_F(PassingEvaluationTest, ratePass_cross_over_enemy_net)
{
    // A pass from the +y side of the field to the -y side of the field,
    // roughly 1/2 way up the enemy half of the field
    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {2, -1.8}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 2}, {2, -2}, avg_desired_pass_speed);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);

    EXPECT_LE(0.8, pass_rating);
    EXPECT_GE(1.0, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_corner_kick_to_center_no_enemies)
{
    // A pass from the positive enemy corner to a robot at the center of the field with
    // no enemies
    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass(world.field().enemyCornerPos(), {0, 0}, avg_desired_pass_speed);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_LE(0.95, pass_rating);
    EXPECT_GE(1.0, pass_rating);
}

// TODO (#1988) renable this test once ratePass is fixed. Currently, since the
// ratePassShootScore is probably 0, ratePass is ~0, so this test fails.
TEST_F(PassingEvaluationTest,
       DISABLED_ratePass_corner_kick_to_marked_robot_at_field_center)
{
    // A corner kick from the +x, +y corner of the field to a robot on the +x axis part
    // way up the enemy half of the field. The receiver friendly is marked by an enemy,
    // but it has enough space that it should be able to break away from it's marker in
    // time to make space to receive the pass and one-time shoot it into the net.

    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(
        {// Robot doing corner kick
         Robot(0, world.field().enemyCornerPos(), {0, 0}, Angle::zero(),
               AngularVelocity::zero(), Timestamp::fromSeconds(0)),
         // Robot at center field
         Robot(1, {2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))},
        Duration::fromSeconds(10));
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(
        {// Enemy goalie
         Robot(0, world.field().enemyGoalCenter() + Vector(-0.1, 0.5), {0, 0},
               Angle::quarter(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
         // Enemy marking friendly in the center
         Robot(1, {2.4, 0}, {0, 0}, Angle::half(), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))},
        Duration::fromSeconds(10));
    world.updateEnemyTeamState(enemy_team);

    Pass pass(world.field().enemyCornerPos(), {1.8, 0.8}, 4.8);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_GE(pass_rating, 0.1);
    EXPECT_LE(pass_rating, 0.7);
}

TEST_F(PassingEvaluationTest, ratePass_below_min_ball_speed)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 0}, {2, 0}, passing_config.min_pass_speed_m_per_s() - 0.1);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_LE(0.0, pass_rating);
    EXPECT_GE(0.05, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_above_max_ball_speed)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 0}, {2, 0}, passing_config.max_pass_speed_m_per_s() + 0.1);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_LE(0.0, pass_rating);
    EXPECT_GE(0.05, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_only_passer_on_field)
{
    // If there is only a passer on the field, no pass is possible
    World world = ::TestUtil::createBlankTestingWorld();

    Robot passer = Robot(13, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));
    Team friendly_team({passer}, Duration::fromSeconds(10));
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({0, 0}, {0.1, 0.1}, avg_desired_pass_speed);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_DOUBLE_EQ(0, pass_rating);
}

// TODO (#1988) renable this test once ratePass is fixed. Currently, since the
// ratePassShootScore is probably 0, ratePass is ~0, so this test fails.
TEST_F(PassingEvaluationTest, DISABLED_ratePass_attempting_to_pass_and_receive_no_shot)
{
    // Test that a pass which does NOT result in a good shot on goal is rated
    // highly if we are rating it as a pass which is intended to be received

    World world = ::TestUtil::createBlankTestingWorld();
    world.updateFriendlyTeamState(Team({
        Robot(0, {1, 0}, {0, 0}, Angle::half(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    }));
    world.updateEnemyTeamState(Team({
        Robot(0, world.field().enemyGoalCenter(), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(1, world.field().enemyGoalCenter() - Vector(0, 0.2), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(2, world.field().enemyGoalCenter() + Vector(0, 0.2), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
    }));

    // Since we're passing from the origin to a point directly in front of the goal,
    // the receiving robot would have to turn 180 degrees to take a shot after
    // receiving the ball
    Pass pass({0, 0}, {1, 0}, avg_desired_pass_speed);

    double pass_rating = ratePass(world, pass, *entire_field, passing_config);
    EXPECT_GE(pass_rating, 0.9);
    EXPECT_LE(pass_rating, 1.0);
}

TEST_F(PassingEvaluationTest, ratePassShootScore_no_robots_and_directly_facing_goal)
{
    // No robots on the field, we receive the pass and are directly facing the goal
    // and are right in front of it
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});
    Field field = Field::createSSLDivisionBField();
    Pass pass({4, 0}, {3.5, 0}, 1);

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass, passing_config);
    EXPECT_LE(0.95, pass_shoot_score);
    EXPECT_GE(1, pass_shoot_score);
}

TEST_F(PassingEvaluationTest,
       DISABLED_ratePassShootScore_no_robots_and_facing_directly_away_from_goal)
{
    // No robots on the field, we receive the pass and are facing directly away from the
    // goal
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});
    Field field = Field::createSSLDivisionBField();
    Pass pass({-1, 0}, {0, 0}, 1);

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass, passing_config);
    EXPECT_GE(pass_shoot_score, 0.0);
    EXPECT_LE(pass_shoot_score, 0.05);
}

TEST_F(PassingEvaluationTest,
       ratePassShootScore_some_robots_far_away_and_facing_directly_away_from_goal)
{
    // Robots on field, but none that are in the way of us shooting after the pass
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {0, 10}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, -10}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    Field field = Field::createSSLDivisionBField();
    Pass pass({3.5, 0}, {3, 0}, 1);

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass, passing_config);
    EXPECT_GE(pass_shoot_score, 0.9);
    EXPECT_LE(pass_shoot_score, 1.0);
}

// TODO (#1988) renable this test once ratePass is fixed. Currently, since the
// ratePassShootScore is probably 0, ratePass is ~0, so this test fails.
TEST_F(PassingEvaluationTest, ratePassShootScore_no_open_shot_to_goal)
{
    // Test rating a pass that results in no open shot to goal
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        // Robot directly in front of the pass position
        Robot(0, {0.2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    Field field = Field::createSSLDivisionBField();
    Pass pass({1, 1}, {0, 0}, 1);

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass, passing_config);
    EXPECT_LE(0, pass_shoot_score);
    EXPECT_GE(0.2, pass_shoot_score);
}

TEST_F(PassingEvaluationTest, ratePassShootScore_decreasing_open_angle_to_goal)
{
    // As we decrease the open angle to the goal, the shot score should also decrease
    Field field = Field::createSSLDivisionBField();
    Pass pass({3.5, 0.0}, {3, 0}, 1);
    Team enemy_team(Duration::fromSeconds(10));

    enemy_team.updateRobots({
        Robot(0, {3.5, 0.1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {3.5, 0.1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    std::vector<Robot> robots_on_field = {};
    double pass_shoot_score0 =
        ratePassShootScore(field, enemy_team, pass, passing_config);
    enemy_team.updateRobots({
        Robot(0, {3.5, 0.2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {3.5, 0.2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    double pass_shoot_score1 =
        ratePassShootScore(field, enemy_team, pass, passing_config);
    enemy_team.updateRobots({
        Robot(0, {3.5, 0.3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {3.5, 0.3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    double pass_shoot_score2 =
        ratePassShootScore(field, enemy_team, pass, passing_config);
    EXPECT_LT(pass_shoot_score0, pass_shoot_score1);
    EXPECT_LT(pass_shoot_score1, pass_shoot_score2);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_no_enemy_robots)
{
    Team enemy_team(Duration::fromSeconds(10));
    Pass pass({0, 0}, {10, 10}, 3);

    auto enemy_reaction_time =
        Duration::fromSeconds(passing_config.enemy_reaction_time());
    auto enemy_proximity_importance = passing_config.enemy_proximity_importance();

    double pass_rating = ratePassEnemyRisk(enemy_team, pass, enemy_reaction_time,
                                           enemy_proximity_importance);
    EXPECT_EQ(1, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_no_robots_near)
{
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {20, 20}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {-30, 50}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    Pass pass({0, 0}, {10, 10}, 4);

    auto enemy_reaction_time =
        Duration::fromSeconds(passing_config.enemy_reaction_time());
    auto enemy_proximity_importance = passing_config.enemy_proximity_importance();

    double pass_rating = ratePassEnemyRisk(enemy_team, pass, enemy_reaction_time,
                                           enemy_proximity_importance);
    EXPECT_GE(pass_rating, 0.9);
    EXPECT_LE(pass_rating, 1.0);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_one_robot_near_receiver_point)
{
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {10, 10}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {10, 10}, 3);

    auto enemy_reaction_time =
        Duration::fromSeconds(passing_config.enemy_reaction_time());
    auto enemy_proximity_importance = passing_config.enemy_proximity_importance();

    double pass_rating = ratePassEnemyRisk(enemy_team, pass, enemy_reaction_time,
                                           enemy_proximity_importance);
    EXPECT_LE(0, pass_rating);
    EXPECT_GE(0.1, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_robot_near_center_of_pass)
{
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {5.2, 6.2}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {10, 10}, 3);

    auto enemy_reaction_time =
        Duration::fromSeconds(passing_config.enemy_reaction_time());
    auto enemy_proximity_importance = passing_config.enemy_proximity_importance();

    double pass_rating = ratePassEnemyRisk(enemy_team, pass, enemy_reaction_time,
                                           enemy_proximity_importance);
    EXPECT_LE(0, pass_rating);
    EXPECT_GE(0.1, pass_rating);
}

TEST_F(PassingEvaluationTest,
       ratePassEnemyRisk_robot_near_center_of_pass_and_by_receiver_point)
{
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {5.2, 6.2}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(1, {10, 10}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {10, 10}, 3);

    auto enemy_reaction_time =
        Duration::fromSeconds(passing_config.enemy_reaction_time());
    auto enemy_proximity_importance = passing_config.enemy_proximity_importance();

    double pass_rating = ratePassEnemyRisk(enemy_team, pass, enemy_reaction_time,
                                           enemy_proximity_importance);
    EXPECT_LE(0, pass_rating);
    EXPECT_GE(0.1, pass_rating);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_team_no_robots)
{
    Team enemy_team(Duration::fromSeconds(10));
    Pass pass({0, 0}, {10, 10}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_team, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_EQ(0, intercept_risk);
}

TEST_F(PassingEvaluationTest,
       calculateInterceptRisk_for_team_several_robots_first_is_close_to_pass)
{
    // Test with an enemy team that has several robots, the first of which is close to
    // the pass trajectory
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {5, 5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {30, 50}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    Pass pass({0, 0}, {10, 10}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_team, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest,
       calculateInterceptRisk_for_team_several_robots_last_is_close_to_pass)
{
    // Test with an enemy team that has several robots, the last of which is close to
    // the pass trajectory
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {30, 50}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {5, 5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    Pass pass({0, 0}, {10, 10}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_team, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_robot_sitting_on_pass_trajectory)
{
    // Test calculating the intercept risk for a robot that is located directly
    // along the trajectory of the pass
    Robot enemy_robot(0, {5, 5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {10, 10}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_robot, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_robot_just_off_pass_trajectory)
{
    // Test calculating the intercept risk for a robot that is located just off to the
    // side of the pass trajectory, but close enough that it will be able to move onto
    // the pass trajectory and intercept it
    Robot enemy_robot(0, {6, 7}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {10, 10}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_robot, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_robot_far_away_from_trajectory)
{
    // Test calculating the intercept risk for a robot that is located far enough away
    // from the pass trajectory that there is no way it will be able to intercept it
    Robot enemy_robot(0, {20, -50}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {10, 10}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_robot, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0, intercept_risk);
    EXPECT_GE(0.1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_robot_at_far_end_of_field)
{
    // Test passing across the enemy end of the field, with an enemy robot over in the
    // friendly end of the field. The enemy robot should not be able to intercept the pass
    Robot enemy_robot(0, {0, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({3, -3}, {3, 3}, 3);

    double intercept_risk = calculateInterceptRisk(
        enemy_robot, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0, intercept_risk);
    EXPECT_GE(0.1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_enemy_moving_far_away)
{
    // An enemy robot moving towards the pass from far away should not be able to
    // intercept it
    Robot enemy_robot(0, {5, -5}, {0, 3}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({1, 1}, {4, 4}, 2);

    double intercept_risk = calculateInterceptRisk(
        enemy_robot, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0, intercept_risk);
    EXPECT_GE(0.1, intercept_risk);
}

TEST_F(PassingEvaluationTest,
       calculateInterceptRisk_just_barely_intercept_starting_from_stop)
{
    // Test a pass that the enemy robot should just barely be able to intercept, starting
    // from a stop
    // x = u*t + 1/2*at^2, u=0, t=1
    double enemy_travel_distance_from_stop =
        0.5 * ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
    Robot enemy_robot(0, {0, enemy_travel_distance_from_stop}, {0, 0}, Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {0, 2}, 0.5);

    double intercept_risk = calculateInterceptRisk(
        enemy_robot, pass, Duration::fromSeconds(passing_config.enemy_reaction_time()));
    EXPECT_LE(0.5, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_no_robots_on_team)
{
    Team team(Duration::fromSeconds(10));
    team.updateRobots({});
    Pass pass({0, 0}, {1, 1}, 10);

    // If there are no robots on the team, then there is no way we can receive a pass
    EXPECT_EQ(0, ratePassFriendlyCapability(team, pass, passing_config));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_pass_speed_0)
{
    Team team(Duration::fromSeconds(10));
    team.updateRobots(
        {Robot(0, {15.5, -10}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0)),
         Robot(1, {100, -100}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {1, 1}, 0);

    // If there are no robots on the team, then there is no way we can receive a pass
    EXPECT_EQ(0, ratePassFriendlyCapability(team, pass, passing_config));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_one_robot_near_pass_one_far_away)
{
    // Test getting friendly capability for a team with two robots, one near the pass
    // reception point and the other far away
    Team team(Duration::fromSeconds(10));
    team.updateRobots(
        {Robot(0, {15.5, -10}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0)),
         Robot(1, {100, -100}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {15, -10.1}, 10);

    // There should be a very high probability that we can receive this pass
    EXPECT_LE(0.9, ratePassFriendlyCapability(team, pass, passing_config));
    EXPECT_GE(1, ratePassFriendlyCapability(team, pass, passing_config));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_should_ignore_passer_robot)
{
    // Test getting friendly capability with two friendly robots:
    // - one robot in the perfect position to receive the pass, but it's the passer
    //   robot, so we should ignore it
    // - one robot fairly far away from the pass receive point, so it won't be able to
    //   receive the pass in time
    // The net result should be a poor friendly capability, as we can only pass to the
    // one robot that can't get to the pass reception point in time

    Robot passer = Robot(13, {2, 2}, {0, 0}, Angle::fromDegrees(270),
                         AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0));
    Robot potential_receiver =
        Robot(1, {-3, 3}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::fromDegrees(0),
              Timestamp::fromSeconds(0));

    Team team({passer, potential_receiver}, Duration::fromSeconds(10));
    Pass pass({2, -2}, {0, 0}, 10);

    double friendly_capability = ratePassFriendlyCapability(team, pass, passing_config);
    EXPECT_GE(friendly_capability, 0);
    EXPECT_LE(friendly_capability, 0.05);
}

TEST_F(PassingEvaluationTest,
       ratePassFriendlyCapability_multiple_robots_all_too_far_from_reception_point)
{
    // Test case where there are lots of robots far away from the reception point and
    // there *is not* enough time for them to get to the reception point
    Team team(Duration::fromSeconds(10));
    team.updateRobots(
        {Robot(0, {15.5, -10}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0)),
         Robot(1, {100, -100}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {1, 1}, 10);

    EXPECT_GE(0.1, ratePassFriendlyCapability(team, pass, passing_config));
    EXPECT_LE(0, ratePassFriendlyCapability(team, pass, passing_config));
}

TEST_F(PassingEvaluationTest,
       ratePassFriendlyCapability_multiple_robots_close_enough_to_reception_point)
{
    // Test case where there are lots of robots far away from the reception point, but
    // when *there is* enough time for them to reach the receive point
    Team team(Duration::fromSeconds(10));
    team.updateRobots(
        {Robot(0, {110, 110}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(5)),
         Robot(1, {100, -100}, {0, 0}, Angle::fromDegrees(0),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(5))});
    Pass pass({100, 100}, {120, 105}, 1);

    EXPECT_LE(0.9, ratePassFriendlyCapability(team, pass, passing_config));
    EXPECT_GE(1, ratePassFriendlyCapability(team, pass, passing_config));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_single_robot_cant_turn_in_time)
{
    // Test case where this is one robot, but it is turned in the wrong direction and
    // will not be able to turn in time to receive the pass
    Team team(Duration::fromSeconds(10));
    team.updateRobots(
        {Robot(0, {1, 0}, {0, 0}, Angle::quarter(), AngularVelocity::fromDegrees(0),
               Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {1, 0}, 6);

    EXPECT_GE(ratePassFriendlyCapability(team, pass, passing_config), 0.0);
    EXPECT_LE(ratePassFriendlyCapability(team, pass, passing_config), 0.1);
}

TEST_F(PassingEvaluationTest,
       ratePassFriendlyCapability_single_robot_already_pointing_in_right_direction)
{
    // Test case where the receiver is already lined up to receive the pass
    Team team(Duration::fromSeconds(10));
    Pass pass({0, 0}, {1, 0}, 3);
    team.updateRobots(
        {Robot(0, {1, 0}, {0, 0}, pass.receiverOrientation(),
               AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0))});

    EXPECT_GE(ratePassFriendlyCapability(team, pass, passing_config), 0.80);
    EXPECT_LE(ratePassFriendlyCapability(team, pass, passing_config), 1.0);
}


TEST_F(PassingEvaluationTest, getStaticPositionQuality_on_field_quality)
{
    Field f = Field::createSSLDivisionBField();

    // Check that the static quality is basically 0 at the edge of the field
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.5, 0), passing_config), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(4.5, 0), passing_config), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, -3.0), passing_config), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, 3.0), passing_config), 0.13);
}

TEST_F(PassingEvaluationTest, getStaticPositionQuality_near_own_goal_quality)
{
    Field f = Field::createSSLDivisionBField();

    // Check that we have a static quality of almost 0 near our goal
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.0, 0), passing_config), 0.14);
}

TEST_F(PassingEvaluationTest, getStaticPositionQuality_near_enemy_goal_quality)
{
    Field f = Field::createSSLDivisionBField();

    // Check that we have a large static quality near the enemy goal
    EXPECT_GE(getStaticPositionQuality(f, Point(3.0, 0), passing_config), 0.80);

    // But we should have basically 0 static quality too close to the enemy goal,
    // as there is a defense area around the net that we cannot pass to
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.3, 1.9), passing_config), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.3, -1.9), passing_config), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.4, 1.9), passing_config), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.4, -1.9), passing_config), 0.0, 0.1);
}
