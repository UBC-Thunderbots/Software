#include "software/ai/hl/stp/play/penalty_kick_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class PenaltyKickEnemyPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(PenaltyKickEnemyPlayTest, test_penalty_kick_enemy_play)
{
    setBallState(BallState(field().penaltyEnemy(), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({
        field().enemyGoalCenter(),
        Point(3, 2.5),
        Point(3.2, 2.5),
        Point(3.4, 2.5),
        Point(3, -2.5),
        Point(3.2, -2.5),
    }));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickEnemyPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_THEM);

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
