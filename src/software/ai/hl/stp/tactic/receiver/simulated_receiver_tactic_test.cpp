#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedReceiverTacticTest
    : public virtual SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<std::tuple<Pass, RobotStateWithId, BallState>>
{
};

TEST_P(SimulatedReceiverTacticTest, receiver_test)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state         = std::get<2>(GetParam());

    setBallState(ball_state);
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)}));
    addFriendlyRobots({robot_state});

    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<ReceiverTactic>(pass, false);
    tactic->updateControlParams(pass);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [pass, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            ballKicked(pass.passerOrientation(), world_ptr, yield);
            robotAtOrientation(1, world_ptr, pass.passerOrientation(),
                               Angle::fromDegrees(5), yield);
            while (!tactic->done())
            {
                yield("Passer tactic kicked ball but is not done");
            }
            robotAtOrientation(1, world_ptr, pass.passerOrientation(),
                               Angle::fromDegrees(5), yield);
            robotAtPosition(1, world_ptr, pass.passerPoint(), 0.1, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironment, SimulatedReceiverTacticTest,
    ::testing::Values(
        // Stationary Ball Tests
        // Passer point != Balls location & Balls location != Robots Location
        std::make_tuple(Pass(Point(0.0, 0.5), Point(0, 0), 5, Timestamp::fromSeconds(0)),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))},
                        BallState(Point(0.5, 0.5), Vector(0, 0)))));
