#include "software/ai/hl/stp/play/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy_play.h"

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
 * Simulated tests for adhering to penalty kick rules as specified in link:
 * https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick
 * */
class PenaltyKickFriendlyVsEnemyPlayTest
        : public SimulatedPlayTestFixture
{
protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(PenaltyKickFriendlyVsEnemyPlayTest, test_penalty_kick_plays)
{
BallState ball_state(field.enemyPenaltyMark(), Vector(0, 0));

auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-1, 2), Point(-1, 1), Point(-1, 0), Point(-1, 1), Point(-1, 2)
        });

auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 2), Point(1, 1), Point(1, 0), Point(1, 1), Point(1, 2)
        });

setFriendlyGoalie(0);
setEnemyGoalie(1);
setFriendlyAIPlay(TYPENAME(PenaltyKickPlay));
setEnemyAIPlay(TYPENAME(PenaltyKickEnemyPlay));
setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_THEM
);
GameState gameState = GameState();
gameState.
updateRefereeCommand(RefereeCommand::FORCE_START);
setGameState(gameState);

std::vector<ValidationFunction> terminating_validation_functions = {
            // This will keep the test running for 10 seconds to give everything enough
            // time to settle into position and be observed with the Visualizer
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type &yield) {
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(10)) {
                    yield("simulated penalty kick friendly vs enemy play test not finished!");
                }
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type &yield) {
                enemyNeverScores(world_ptr, yield);
            }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
