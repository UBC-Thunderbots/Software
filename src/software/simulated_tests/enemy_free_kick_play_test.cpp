#include <gtest/gtest.h>

#include "software/ai/hl/stp/play/enemy_freekick_play.h"
#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class EnemyFreekickPlayTest : public SimulatedTestFixture
{
};

TEST_F(EnemyFreekickPlayTest, test_enemy_freekick_play)
{
    setBallState(BallState(Point(0.9, 2.85), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -3.0)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({
        field().enemyGoalCenter(),
        field().enemyDefenseArea().negXNegYCorner(),
        field().enemyDefenseArea().negXPosYCorner(),
        Point(1, 3),
        Point(-1, -0.25),
        Point(-2, -1.25),
    }));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(EnemyFreekickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_THEM);

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1396
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {}};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
