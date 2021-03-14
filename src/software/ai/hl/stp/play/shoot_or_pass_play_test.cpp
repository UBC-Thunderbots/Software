#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class ShootOrPassPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(ShootOrPassPlayTest, test_shoot_or_pass_play)
{
    setBallState(BallState(Point(-4.4, 2.9), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId({
        field().friendlyGoalCenter(),
        Point(-4.5, 3.0),
        Point(-2, 1.5),
        Point(-2, 0.5),
        Point(-2, -0.5),
        Point(-2, -1.5),
    }));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(CLASS_TYPENAME(ShootOrPassPlay));
    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1396
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
