#include "software/ai/hl/stp/play/defense_play.h"
#include "software/ai/hl/stp/play/offense/offense_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/non_terminating_validation_functions/enemy_never_scores_validation.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

/**
 * Issues
 *    - very erratic
 *    - not deterministic at all - run to run variations are huge
 *    - terminates at 10s regardless of timeout...
 *
 * */
class AttackVsDefencePlayTest
        : public SimulatedPlayTestFixture
{
protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(AttackVsDefencePlayTest, test_attack_vs_defence)
{
BallState ball_state(field.friendlyPenaltyMark(), Vector(0, 0));

auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-2, -2), Point(-2, -1), Point(-2, 0), Point(-2, 1), Point(-2, 2)
        });

auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(2, -2), Point(2, -1), Point(2, 0), Point(2, 1), Point(2, 2)
        });

setFriendlyGoalie(0);
setEnemyGoalie(1);
setFriendlyAIPlay(TYPENAME(OffensePlay));
setEnemyAIPlay(TYPENAME(DefensePlay));
setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::STOP);
GameState gameState = GameState();
gameState.
updateRefereeCommand(RefereeCommand::FORCE_START);
setGameState(gameState);

std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 10 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type &yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(30)) {
                yield("simulated penalty kick friendly vs enemy play test not finished!");
            }
        }};

std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type &yield) {
        }};

runTest(field, ball_state, friendly_robots, enemy_robots,
        terminating_validation_functions, non_terminating_validation_functions,
        Duration::fromSeconds(10));
}

