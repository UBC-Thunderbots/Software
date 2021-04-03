#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/passer/passer_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedPasserTacticTest
    : public virtual SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<std::tuple<Pass, RobotStateWithId, BallState>>
{
};

TEST_P(SimulatedPasserTacticTest, passer_test)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state         = std::get<2>(GetParam());

    setBallState(ball_state);
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)}));
    addFriendlyRobots({robot_state});

    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<PasserTactic>(pass, false);
    tactic->updateControlParams(pass);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [pass, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic did not complete!");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironment, SimulatedPasserTacticTest,
    ::testing::Values(
        std::make_tuple(Pass(Point(0, 0), Point(0, 1), 5, Timestamp::fromSeconds(0)),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))},
                        BallState(Point(1, 1), Vector(0, 0))),

        std::make_tuple(Pass(Point(0, 0), Point(0, 1), 5, Timestamp::fromSeconds(0)),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))},
                        BallState(Point(1, 1), Vector(0, 0)))));
