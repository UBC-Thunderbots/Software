#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/cost_function.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedAttackerTacticKeepAwayTest
    : public SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<
          std::tuple<Pass, RobotStateWithId, BallState, std::vector<RobotStateWithId>>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(SimulatedAttackerTacticKeepAwayTest, attacker_test_keep_away)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state         = std::get<2>(GetParam());
    auto enemy_robots            = std::get<3>(GetParam());

    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)});
    friendly_robots.emplace_back(robot_state);

    auto attacker_tactic_config = std::make_shared<AttackerTacticConfig>();
    // force passing for this test by setting min acceptable shot angle very high
    attacker_tactic_config->getMutableMinOpenAngleForShotDeg()->setValue(90);
    attacker_tactic_config->getMutableEnemyAboutToStealBallRadius()->setValue(0.01);
    auto tactic = std::make_shared<AttackerTactic>(attacker_tactic_config);
    // force the keep away state
    tactic->updateControlParams(pass, false);
    setTactic(tactic);
    setRobotId(1);

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

    // constants copypasted from software/ai/evaluation/keep_away.cpp for now
    // TODO: cleanup passing parameters as part of #1987
    static constexpr double ENEMY_PROXIMITY_IMPORTANCE = 0.5;
    static const auto ENEMY_REACTION_TIME              = Duration::fromSeconds(0.4);

    auto initial_enemy_risk_score = ratePassEnemyRisk(
        enemy_team, pass, ENEMY_REACTION_TIME, ENEMY_PROXIMITY_IMPORTANCE);
    (void)initial_enemy_risk_score;

    std::vector<ValidationFunction> terminating_validation_functions = {
        [initial_enemy_risk_score, pass](std::shared_ptr<World> world_ptr,
                                         ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(1))
            {
                yield("Timestamp not at 1s");
            }

            while (ratePassEnemyRisk(world_ptr->enemyTeam(), pass, ENEMY_REACTION_TIME,
                                     ENEMY_PROXIMITY_IMPORTANCE) <
                   initial_enemy_risk_score)
            {
                yield("ratePassEnemyRisk score not improved!");
            }

            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield("Timestamp not at 2s");
            }

            while (ratePassEnemyRisk(world_ptr->enemyTeam(), pass, ENEMY_REACTION_TIME,
                                     ENEMY_PROXIMITY_IMPORTANCE) <
                   initial_enemy_risk_score)
            {
                yield("ratePassEnemyRisk score not improved!");
            }

            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(3))
            {
                yield("Timestamp not at 3s");
            }

            while (ratePassEnemyRisk(world_ptr->enemyTeam(), pass, ENEMY_REACTION_TIME,
                                     ENEMY_PROXIMITY_IMPORTANCE) <
                   initial_enemy_risk_score)
            {
                yield("ratePassEnemyRisk score not improved!");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(4));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironment, SimulatedAttackerTacticKeepAwayTest,
    ::testing::Values(
        // Stationary Ball Tests
        // Attacker point != Balls location & Balls location != Robots Location
        std::make_tuple(
            // the best pass so far to pass into the AttackerTactic
            Pass(Point(0.0, 0.0), Point(-3, 2.5), 5),
            // the state of the friendly robot
            RobotStateWithId{1, RobotState(Point(0.25, 0), Vector(0, 0),
                                           Angle::fromDegrees(0), Angle::fromDegrees(0))},
            // the state of the ball
            BallState(Point(0., 0.), Vector(0, 0)),
            // the states of the enemy robots
            TestUtil::createStationaryRobotStatesWithId(
                {Point(-0.6, 0.25), Point(0., 0.6), Point(-0.25, 0.5),
                 Point(0.6, -0.25)})),
        std::make_tuple(
            // the best pass so far to pass into the AttackerTactic
            Pass(Point(0.0, 0.0), Point(-3, 2.5), 5),
            // the state of the friendly robot
            RobotStateWithId{1, RobotState(Point(0.25, 0), Vector(0, 0),
                                           Angle::fromDegrees(0), Angle::fromDegrees(0))},
            // the state of the ball
            BallState(Point(0., 0.), Vector(0, 0)),
            // the states of the enemy robots
            TestUtil::createStationaryRobotStatesWithId({Point(-0.5, 0.5)})),
        std::make_tuple(
            // the best pass so far to pass into the AttackerTactic
            Pass(Point(0.0, 0.0), Point(-3, 2.5), 5),
            // the state of the friendly robot
            RobotStateWithId{1, RobotState(Point(0.25, 0), Vector(0, 0),
                                           Angle::fromDegrees(0), Angle::fromDegrees(0))},
            // the state of the ball
            BallState(Point(0., 0.), Vector(0, 0)),
            // the states of the enemy robots
            TestUtil::createStationaryRobotStatesWithId({Point(-0.5, 0.5),
                                                         Point(-0.5, 0)}))));
