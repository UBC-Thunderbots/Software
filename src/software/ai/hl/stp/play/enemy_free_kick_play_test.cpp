#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class EnemyFreekickPlayTest : public SimulatedPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(EnemyFreekickPlayTest, test_enemy_free_kick_play)
{
    BallState ball_state(Point(0.9, 2.85), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 0), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -3.0)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
        Point(1, 3),
        Point(-1, -0.25),
        Point(-2, -1.25),
    });
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(EnemyFreekickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_THEM);

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

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {}};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
