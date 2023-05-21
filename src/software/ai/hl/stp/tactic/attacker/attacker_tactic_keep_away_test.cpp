#include <gtest/gtest.h>

#include <sstream>
#include <utility>

#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/cost_function.h"
#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class AttackerTacticKeepAwayTest
    : public SimulatedErForceSimPlayTestFixture,
      public ::testing::WithParamInterface<std::tuple<
          Pass, RobotStateWithId, BallState, std::vector<RobotStateWithId>, bool>>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_P(AttackerTacticKeepAwayTest, attacker_test_keep_away)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state         = std::get<2>(GetParam());
    auto enemy_robots            = std::get<3>(GetParam());
    bool ignore_score_checks     = std::get<4>(GetParam());

    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)});
    friendly_robots.emplace_back(robot_state);

    TbotsProto::AiConfig ai_config;

    // force passing for this test by setting min acceptable shot angle very high
    ai_config.mutable_attacker_tactic_config()->set_min_open_angle_for_shot_deg(90);
    ai_config.mutable_attacker_tactic_config()->set_enemy_about_to_steal_ball_radius(
        0.01);

    auto tactic = std::make_shared<AttackerTactic>(ai_config);

    // force the keep away state
    tactic->updateControlParams(pass, false);
    setTactic(1, tactic);

    // we use default parameters for testing ratePassEnemyRisk because that's (partially)
    // what the play uses to determine pass/no pass. Keep away state should try to
    // push the best pass in the play above the threshold to commit to passing.
    auto passing_config = TbotsProto::PassingConfig();
    auto enemy_reaction_time =
        Duration::fromSeconds(passing_config.enemy_reaction_time());
    auto enemy_proximity_importance = passing_config.enemy_proximity_importance();

    // we have to create a Team for the enemy here to evaluate the initial enemy risk
    // score
    std::vector<Robot> enemy_team_robots;
    std::transform(enemy_robots.begin(), enemy_robots.end(),
                   std::back_inserter(enemy_team_robots),
                   [](const RobotStateWithId& robot_state) {
                       return Robot(robot_state.id, robot_state.robot_state,
                                    Timestamp::fromSeconds(0));
                   });
    Team enemy_team(enemy_team_robots);

    static const auto CHECK_SCORE_INTERVAL = Duration::fromSeconds(0.5);
    static constexpr auto NUM_CHECKS       = 5;

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        // test the proximity risk every CHECK_SCORE_INTERVAL time and make sure
        // it doesn't get substantially worse compared to the last check
        // and that it is an improvement compared to the starting state
        [&](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (ignore_score_checks)
            {
                yield("");
            }

            // this value is copypasted from software/ai/evaluation/keep_away.cpp
            static constexpr double PASSER_ENEMY_PROXIMITY_IMPORTANCE = 1.5;

            auto initial_enemy_proximity_risk = calculateProximityRisk(
                world_ptr->ball().position(), world_ptr->enemyTeam(),
                PASSER_ENEMY_PROXIMITY_IMPORTANCE);

            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(1))
            {
                yield("");
            }

            auto last_timestamp = world_ptr->getMostRecentTimestamp();

            // test runs for NUM_CHECKS * CHECK_SCORE_INTERVAL seconds
            for (int i = 0; i < NUM_CHECKS; i++)
            {
                // only check the enemy risk score every 0.5s to mitigate the "noise"
                // inherent in the ratePass______ functions
                while (world_ptr->getMostRecentTimestamp() - last_timestamp <
                       CHECK_SCORE_INTERVAL)
                {
                    yield("");
                }
                last_timestamp = world_ptr->getMostRecentTimestamp();

                auto current_enemy_proximity_risk = calculateProximityRisk(
                    world_ptr->ball().position(), world_ptr->enemyTeam(),
                    PASSER_ENEMY_PROXIMITY_IMPORTANCE);

                // make sure we improved over the initial proximity risk score
                if (current_enemy_proximity_risk > initial_enemy_proximity_risk + 0.05)
                {
                    std::stringstream ss;
                    ss << "At " << last_timestamp
                       << " calculateProximityRisk didn't improve over initial! Went from "
                       << initial_enemy_proximity_risk << " to "
                       << current_enemy_proximity_risk;
                    yield(ss.str());
                }
            }
        },
        // test the ratePassEnemyRisk every CHECK_SCORE_INTERVAL time and make sure
        // it doesn't get substantially worse compared to the last check
        // and that it is an improvement compared to the starting state
        [&](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (ignore_score_checks)
            {
                yield("");
            }

            auto initial_enemy_risk_score = ratePassEnemyRisk(
                enemy_team, pass, enemy_reaction_time, enemy_proximity_importance);

            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(1))
            {
                yield("");
            }

            auto last_timestamp = world_ptr->getMostRecentTimestamp();

            // test runs for NUM_CHECKS * CHECK_SCORE_INTERVAL seconds
            for (int i = 0; i < NUM_CHECKS; i++)
            {
                // only check the enemy risk score every 0.5s to mitigate the "noise"
                // inherent in the ratePass______ functions
                while (world_ptr->getMostRecentTimestamp() - last_timestamp <
                       CHECK_SCORE_INTERVAL)
                {
                    yield("");
                }
                last_timestamp = world_ptr->getMostRecentTimestamp();

                // update the pass to reflect the new passer point, now that
                // the ball has (probably) moved
                Pass new_pass(world_ptr->ball().position(), pass.receiverPoint(),
                              pass.speed());

                auto current_enemy_risk_score =
                    ratePassEnemyRisk(enemy_team, new_pass, enemy_reaction_time,
                                      enemy_proximity_importance);

                // make sure we improved over the initial enemy risk score
                if (current_enemy_risk_score < (initial_enemy_risk_score - 0.01))
                {
                    std::stringstream ss;
                    ss << "At " << last_timestamp
                       << " ratePassEnemyRisk didn't improve over initial! Went from "
                       << initial_enemy_risk_score << " to " << current_enemy_risk_score;
                    yield(ss.str());
                }
            }
        },
        [&](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(1, world_ptr, yield);
        },
        [&](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // test that the ball is always in the field boundaries
            if (!contains(world_ptr->field().fieldLines(), world_ptr->ball().position()))
            {
                yield("Ball left the field boundaries!");
            }
            else
            {
                yield("");
            }
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots, {},
            non_terminating_validation_functions, Duration::fromSeconds(3.0));
}

auto FIELD_TOP_LEFT = Field::createSSLDivisionBField().fieldLines().negXPosYCorner();

INSTANTIATE_TEST_CASE_P(
    PassEnvironment, AttackerTacticKeepAwayTest,
    ::testing::Values(
        std::make_tuple(
            // the best pass so far to pass into the AttackerTactic
            Pass(Point(-0.2, 0.0), Point(-3, 2.5), 5),
            // the state of the friendly robot
            RobotStateWithId{
                1, RobotState(Point(0.25, 0), Vector(0, 0), Angle::fromDegrees(180),
                              Angle::fromDegrees(0))},
            // the state of the ball
            BallState(Point(0., 0.), Vector(0, 0)),
            // the states of the enemy robots
            TestUtil::createStationaryRobotStatesWithId(
                {Point(-0.6, 0.25), Point(0., 0.6), Point(-0.25, 0.5),
                 Point(0.6, -0.25)}),
            // whether to ignore the intercept and proximity risk checks in the test
            false),
        std::make_tuple(
            // the best pass so far to pass into the AttackerTactic
            Pass(Point(0.0, 0.0), Point(-3, 2.5), 5),
            // the state of the friendly robot
            RobotStateWithId{1, RobotState(Point(0.25, 0), Vector(0, 0),
                                           Angle::fromDegrees(0), Angle::fromDegrees(0))},
            // the state of the ball
            BallState(Point(0., 0.), Vector(0, 0)),
            // the states of the enemy robots
            TestUtil::createStationaryRobotStatesWithId({Point(-0.5, 0.5)}),
            // whether to ignore the intercept and proximity risk checks in the test
            false),
        std::make_tuple(
            // the best pass so far to pass into the AttackerTactic
            Pass(Point(FIELD_TOP_LEFT.x() + 0.05, FIELD_TOP_LEFT.y() - 0.05), Point(0, 0),
                 5),
            // the state of the friendly robot
            RobotStateWithId{1, RobotState(FIELD_TOP_LEFT, Vector(0, 0),
                                           Angle::fromDegrees(0), Angle::fromDegrees(0))},
            // the state of the ball
            BallState(Point(FIELD_TOP_LEFT.x() + 0.05, FIELD_TOP_LEFT.y() - 0.2),
                      Vector(0, 0)),
            // the states of the enemy robots
            TestUtil::createStationaryRobotStatesWithId(
                {Point(-4, 2), Point(-4, 2.25), Point(-4, 2.5), Point(-4, 2.75)}),
            // whether to ignore the intercept and proximity risk checks in the test
            // we ignore the score checks on this one because we need to make sure that we
            // stay in field bounds, even if leaving the field bounds improves the score
            true)));
