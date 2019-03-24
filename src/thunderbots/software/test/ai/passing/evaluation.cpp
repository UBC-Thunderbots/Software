/**
 * This file contains unit tests passing evaluation functions
 */

#include "ai/passing/evaluation.h"

#include <gtest/gtest.h>
#include <util/parameter/dynamic_parameters.h>

#include "../shared/constants.h"
#include "test/test_util/test_util.h"

using namespace AI::Passing;

class PassingEvaluationTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        avg_desired_pass_speed =
            (max_pass_speed_param - min_pass_speed_param) / 2 + min_pass_speed_param;
    }

    // We get these values here so we can make these tests robust to change
    double min_pass_speed_param =
        Util::DynamicParameters::AI::Passing::min_pass_speed_m_per_s.value();
    double max_pass_speed_param =
        Util::DynamicParameters::AI::Passing::max_pass_speed_m_per_s.value();
    double avg_desired_pass_speed;
};

TEST_F(PassingEvaluationTest, ratePass_no_target_region)
{
    // This should be a really good pass, and since there is no target region it should
    // be highly scored
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 0}, {2, 0}, avg_desired_pass_speed, Timestamp::fromSeconds(2));

    double pass_rating = ratePass(world, pass, std::nullopt);
    EXPECT_LE(0.5, pass_rating);
    EXPECT_GE(1.0, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_with_target_region)
{
    // This should be a really good pass, but it's outside our target region, so it
    // should be rate poorly
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 0}, {2, 0}, avg_desired_pass_speed, Timestamp::fromSeconds(2));

    Rectangle target_region({1, 1}, {2, 2});
    double pass_rating = ratePass(world, pass, target_region);
    EXPECT_LE(0.0, pass_rating);
    EXPECT_GE(0.1, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_pass_at_past_time)
{
    // We should very poorly rate a pass that has occurred in the past
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    // We update the the ball state because that's what is used as a reference for the
    // current time by the evaluation function
    // TODO (Issue #423): Change this to use the `World` timestamp when `World` has one
    world.updateBallState(Ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5)));

    Pass pass({3, 0}, {2, 0}, avg_desired_pass_speed, Timestamp::fromSeconds(2));

    double pass_rating = ratePass(world, pass, std::nullopt);
    EXPECT_LE(0.0, pass_rating);
    EXPECT_GE(0.01, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_below_min_ball_speed)
{
    // We should very poorly rate a pass that has occurred in the past
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 0}, {2, 0}, min_pass_speed_param - 0.1, Timestamp::fromSeconds(1));

    double pass_rating = ratePass(world, pass, std::nullopt);
    EXPECT_LE(0.0, pass_rating);
    EXPECT_GE(0.01, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePass_above_max_ball_speed)
{
    // We should very poorly rate a pass that has occurred in the past
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Pass pass({3, 0}, {2, 0}, max_pass_speed_param + 0.1, Timestamp::fromSeconds(1));

    double pass_rating = ratePass(world, pass, std::nullopt);
    EXPECT_LE(0.0, pass_rating);
    EXPECT_GE(0.01, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePassShootScore_no_robots_and_directly_facing_goal)
{
    // No robots on the field, we receive the pass and are directly facing the goal
    // and are right in front of it
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});
    Field field = ::Test::TestUtil::createSSLDivBField();
    Pass pass({4, 0}, {3.5, 0}, 1, Timestamp::fromSeconds(1));

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass);
    EXPECT_LE(0.99, pass_shoot_score);
    EXPECT_GE(1, pass_shoot_score);
}

TEST_F(PassingEvaluationTest,
       ratePassShootScore_no_robots_and_facing_directly_away_from_goal)
{
    // No robots on the field, we receive the pass and are facing directly away from the
    // goal
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});
    Field field = ::Test::TestUtil::createSSLDivBField();
    Pass pass({-1, 0}, {0, 0}, 1, Timestamp::fromSeconds(1));

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass);
    EXPECT_LE(0, pass_shoot_score);
    EXPECT_GE(0.5, pass_shoot_score);
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
    Field field = ::Test::TestUtil::createSSLDivBField();
    Pass pass({3.5, 0}, {3, 0}, 1, Timestamp::fromSeconds(1));

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass);
    EXPECT_LE(0.9, pass_shoot_score);
    EXPECT_GE(1, pass_shoot_score);
}

TEST_F(PassingEvaluationTest, ratePassShootScore_no_open_shot_to_goal)
{
    // Test rating a pass that results in no open shot to goal
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        // Robot directly in front of the pass position
        Robot(0, {0.2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    Field field = ::Test::TestUtil::createSSLDivBField();
    Pass pass({1, 1}, {0, 0}, 1, Timestamp::fromSeconds(1));

    double pass_shoot_score = ratePassShootScore(field, enemy_team, pass);
    EXPECT_LE(0, pass_shoot_score);
    EXPECT_GE(0.05, pass_shoot_score);
}

TEST_F(PassingEvaluationTest, ratePassShootScore_decreasing_open_angle_to_goal)
{
    // As we decrease the open angle to the goal, the shot score should also decrease
    Field field = ::Test::TestUtil::createSSLDivBField();
    Pass pass({3.5, 1}, {3, 0}, 1, Timestamp::fromSeconds(1));
    Team enemy_team(Duration::fromSeconds(10));

    enemy_team.updateRobots({
        Robot(0, {3.5, 0.1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {3.5, 0.1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    std::vector<Robot> robots_on_field = {};
    double pass_shoot_score0           = ratePassShootScore(field, enemy_team, pass);
    enemy_team.updateRobots({
        Robot(0, {3.5, 0.2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {3.5, 0.2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    double pass_shoot_score1 = ratePassShootScore(field, enemy_team, pass);
    enemy_team.updateRobots({
        Robot(0, {3.5, 0.3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {3.5, 0.3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    double pass_shoot_score2 = ratePassShootScore(field, enemy_team, pass);
    EXPECT_LT(pass_shoot_score0, pass_shoot_score1);
    EXPECT_LT(pass_shoot_score1, pass_shoot_score2);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_no_enemy_robots)
{
    Team enemy_team(Duration::fromSeconds(10));
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double pass_rating = ratePassEnemyRisk(enemy_team, pass);
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
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double pass_rating = ratePassEnemyRisk(enemy_team, pass);
    EXPECT_LE(0.95, pass_rating);
    EXPECT_GE(1, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_one_robot_near_receiver_point)
{
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {10, 10}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double pass_rating = ratePassEnemyRisk(enemy_team, pass);
    EXPECT_LE(0, pass_rating);
    EXPECT_GE(0.1, pass_rating);
}

TEST_F(PassingEvaluationTest, ratePassEnemyRisk_robot_near_center_of_pass)
{
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {5.2, 6.2}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double pass_rating = ratePassEnemyRisk(enemy_team, pass);
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
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double pass_rating = ratePassEnemyRisk(enemy_team, pass);
    EXPECT_LE(0, pass_rating);
    EXPECT_GE(0.1, pass_rating);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_team_no_robots)
{
    Team enemy_team(Duration::fromSeconds(10));
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_team, pass);
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
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_team, pass);
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
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_team, pass);
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_robot_sitting_on_pass_trajectory)
{
    // Test calculating the intercept risk for a robot that is located directly
    // along the trajectory of the pass
    Robot enemy_robot(0, {5, 5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
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
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_for_robot_far_away_from_trajectory)
{
    // Test calculating the intercept risk for a robot that is located far enough away
    // from the pass trajectory that there is no way it will be able to intercept it
    Robot enemy_robot(0, {20, -50}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {10, 10}, 3, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
    EXPECT_LE(0, intercept_risk);
    EXPECT_GE(0.1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_robot_at_far_end_of_field)
{
    // Test passing across the enemy end of the field, with an enemy robot over in the
    // friendly end of the field. The enemy robot should not be able to intercept the pass
    Robot enemy_robot(0, {0, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({3, -3}, {3, 3}, 2, Timestamp::fromSeconds(1));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
    EXPECT_LE(0, intercept_risk);
    EXPECT_GE(0.1, intercept_risk);
}

TEST_F(PassingEvaluationTest, calculateInterceptRisk_enemy_moving_far_away)
{
    // An enemy robot moving towards the pass from far away should not be able to
    // intercept it
    Robot enemy_robot(0, {5, -5}, {0, 3}, Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
    Pass pass({1, 1}, {4, 4}, 2, Timestamp::fromSeconds(0));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
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
    Pass pass({0, 0}, {0, 2}, 0.5, Timestamp::fromSeconds(0));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
    EXPECT_LE(0.5, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest,
       calculateInterceptRisk_just_barely_intercept_starting_with_initial_velocity)
{
    // Test a pass that the enemy robot should just barely be able to intercept, with a
    // given initial velocity
    // x = u*t + 1/2*at^2, u=max_enemy_robot_vel, t=1
    double enemy_travel_distance_from_stop =
        ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND +
        0.5 * ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
    Robot enemy_robot(0, {0, enemy_travel_distance_from_stop}, {0, 0}, Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {0, 2}, 0.5, Timestamp::fromSeconds(0));

    double intercept_risk = calculateInterceptRisk(enemy_robot, pass);
    EXPECT_LE(0.9, intercept_risk);
    EXPECT_GE(1, intercept_risk);
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_no_robots_on_team)
{
    Team team(Duration::fromSeconds(10));
    team.updateRobots({});
    Pass pass({0, 0}, {1, 1}, 10, Timestamp::fromSeconds(10));

    // If there are no robots on the team, then there is no way we can receive a pass
    EXPECT_EQ(0, ratePassFriendlyCapability(team, pass));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_pass_speed_0)
{
    Team team(Duration::fromSeconds(10));
    team.updateRobots({Robot(0, {15.5, -10}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0)),
                       Robot(1, {100, -100}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {1, 1}, 0, Timestamp::fromSeconds(10));

    // If there are no robots on the team, then there is no way we can receive a pass
    EXPECT_EQ(0, ratePassFriendlyCapability(team, pass));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_one_robot_near_pass_one_far_away)
{
    // Test getting friendly capability for a team with two robots, one near the pass
    // reception point and the other far away
    Team team(Duration::fromSeconds(10));
    team.updateRobots({Robot(0, {15.5, -10}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0)),
                       Robot(1, {100, -100}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {15, -10.1}, 10, Timestamp::fromSeconds(10));

    // There should be a very high probability that we can receive this pass
    EXPECT_LE(0.9, ratePassFriendlyCapability(team, pass));
    EXPECT_GE(1, ratePassFriendlyCapability(team, pass));
}

TEST_F(PassingEvaluationTest,
       ratePassFriendlyCapability_multiple_robots_all_too_far_from_reception_point)
{
    // Test case where there are lots of robots far away from the reception point and
    // there *is not* enough time for them to get to the reception point
    Team team(Duration::fromSeconds(10));
    team.updateRobots({Robot(0, {15.5, -10}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0)),
                       Robot(1, {100, -100}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {1, 1}, 10, Timestamp::fromSeconds(1));

    EXPECT_GE(0.1, ratePassFriendlyCapability(team, pass));
    EXPECT_LE(0, ratePassFriendlyCapability(team, pass));
}

TEST_F(PassingEvaluationTest,
       ratePassFriendlyCapability_multiple_robots_close_enough_to_reception_point)
{
    // Test case where there are lots of robots far away from the reception point, but
    // when *there is* enough time for them to reach the receive point
    Team team(Duration::fromSeconds(10));
    team.updateRobots({Robot(0, {110, 110}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(5)),
                       Robot(1, {100, -100}, {0, 0}, Angle::ofDegrees(0),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(5))});
    Pass pass({100, 100}, {120, 105}, 1, Timestamp::fromSeconds(10));

    EXPECT_LE(0.9, ratePassFriendlyCapability(team, pass));
    EXPECT_GE(1, ratePassFriendlyCapability(team, pass));
}

TEST_F(PassingEvaluationTest, ratePassFriendlyCapability_single_robot_cant_turn_in_time)
{
    // Test case where this is one robot, but it is turned in the wrong direction and
    // will not be able to turn in time to receive the pass
    Team team(Duration::fromSeconds(10));
    team.updateRobots({Robot(0, {1, 0}, {0, 0}, Angle::quarter(),
                             AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))});
    Pass pass({0, 0}, {-1, -1}, 3, Timestamp::fromSeconds(0.1));

    EXPECT_GE(0.1, ratePassFriendlyCapability(team, pass));
    EXPECT_LE(0, ratePassFriendlyCapability(team, pass));
}


TEST_F(PassingEvaluationTest, getStaticPositionQuality_on_field_quality)
{
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that the static quality is basically 0 at the edge of the field
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.5, 0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(4.5, 0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, -3.0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, 3.0)), 0.13);
}

TEST_F(PassingEvaluationTest, getStaticPositionQuality_near_own_goal_quality)
{
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that we have a static quality of almost 0 near our goal
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.0, 0)), 0.14);
}

TEST_F(PassingEvaluationTest, getStaticPositionQuality_near_enemy_goal_quality)
{
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that we have a large static quality near the enemy goal
    EXPECT_GE(getStaticPositionQuality(f, Point(3.0, 0)), 0.80);

    // But we should have basically 0 static quality too close to the enemy goal,
    // as there is a defense area around the net that we cannot pass to
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.3, 1.9)), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.3, -1.9)), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.4, 1.9)), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.4, -1.9)), 0.0, 0.1);
}

TEST_F(PassingEvaluationTest, rectangleSigmoid_value_in_rectangle_centered_rectangle)
{
    Rectangle r1(Point(-1, -2), Point(1, 2));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {0, 0}, 0.1), 0.982);
}

TEST_F(PassingEvaluationTest, rectangleSigmoid_value_in_rectangle_offset_rectangle)
{
    Rectangle r1(Point(-2, -1), Point(0, 3));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {-1, 1}, 0.1), 0.9);
}

TEST_F(PassingEvaluationTest, rectangleSigmoid_values_outside_rectangle)
{
    Rectangle r1(Point(-1, -2), Point(1, 2));

    // Check that values off in x are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.2, 0}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.2, 0}, 0.1), 0.1);

    // Check that values off in y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {0, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {0, 2.1}, 0.1), 0.1);

    // Check that values off in x and y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.1, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {-1.1, 2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1, 2.1}, 0.1), 0.1);
}

TEST_F(PassingEvaluationTest, circleSigmoid_value_in_circle_centered_circle)
{
    Circle circle(Point(0, 0), 1);

    EXPECT_GE(circleSigmoid(circle, {0, 0}, 0.1), 0.982);
}

TEST_F(PassingEvaluationTest, circleSigmoid_value_in_circle_offset_circle)
{
    Circle circle(Point(1, -1), 1);

    EXPECT_GE(circleSigmoid(circle, {1, -1}, 0.1), 0.982);
}

TEST_F(PassingEvaluationTest, circleSigmoid_value_on_circle_edge)
{
    Circle circle(Point(1, -1), 1);

    EXPECT_EQ(circleSigmoid(circle, {0, -1}, 0.1), 0.5);
    EXPECT_NEAR(circleSigmoid(circle, {1 + std::sqrt(2) / 2, -1 - std::sqrt(2) / 2}, 0.1),
                0.5, 0.01);
}

TEST_F(PassingEvaluationTest, circleSigmoid_value_outside_circle_offset_circle)
{
    Circle circle(Point(1, -1), 1);

    // Check that values off in x are basically 0
    EXPECT_LE(circleSigmoid(circle, {-0.2, 0}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {2.2, 0}, 0.1), 0.018);

    // Check that values off in y are basically 0
    EXPECT_LE(circleSigmoid(circle, {0, 0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {0, -2.2}, 0.1), 0.018);

    // Check that values off in x and y are basically 0
    EXPECT_LE(circleSigmoid(circle, {-0.2, 0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {2.2, -1.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {0.2, -0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {-2.2, 1.2}, 0.1), 0.018);
}

TEST_F(PassingEvaluationTest, sigmoid_sig_width_is_respected)
{
    // Test the value at sig_width/2 is 0.982
    EXPECT_NEAR(sigmoid(5, 0, 10), 0.982, 0.0001);

    // Test the value at -sig_width/2 is 0.018
    EXPECT_NEAR(sigmoid(-5, 0, 10), 0.018, 0.001);
}

TEST_F(PassingEvaluationTest, sigmoid_offset)
{
    // Test that the value at 0 is 0.5 if no offset
    EXPECT_DOUBLE_EQ(0.5, sigmoid(0, 0, 1));

    // Test that the value at the offset is 0.5
    EXPECT_DOUBLE_EQ(0.5, sigmoid(2, 2, 1));
    EXPECT_DOUBLE_EQ(0.5, sigmoid(-2, -2, 1));
}

TEST_F(PassingEvaluationTest, sigmoid_negating_sig_width_flips_sigmoid)
{
    // Test that negating sig_width inverts the sigmoid
    EXPECT_NEAR(sigmoid(-5, 0, -10), 0.982, 0.0001);
    EXPECT_NEAR(sigmoid(5, 0, -10), 0.018, 0.0001);
}
