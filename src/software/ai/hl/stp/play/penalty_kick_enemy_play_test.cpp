#include "software/ai/hl/stp/play/penalty_kick_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class PenaltyKickEnemyPlayTest : public SimulatedPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(PenaltyKickEnemyPlayTest, test_penalty_kick_enemy_play)
{
    BallState ball_state(field.enemyPenaltyMark(), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
        Point(field.enemyPenaltyMark().x() + 0.3, 0),
        Point(field.enemyPenaltyMark().x() + 1, 0),
        Point(field.enemyPenaltyMark().x() + 1, 4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, 8 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, -4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, -8 * ROBOT_MAX_RADIUS_METERS),
    });
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickEnemyPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_THEM);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
