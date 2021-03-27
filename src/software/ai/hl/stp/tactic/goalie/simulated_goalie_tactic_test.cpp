#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedGoalieTacticTest : public SimulatedTacticTestFixture
{
};

TEST_F(SimulatedGoalieTacticTest, test_stationary_ball_far_away)
{
    Point initial_position = Point(-4.5, 0);
    setBallState(BallState(Point(3, -2), Vector(-0.5, 1)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
             field().enemyDefenseArea().negXNegYCorner(),
             field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<GoalieTactic>(true, std::make_shared<const GoalieTacticConfig>());
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                // add terminating validation functions here
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}