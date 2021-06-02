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
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(SimulatedReceiverTacticTest, receiver_test)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state         = std::get<2>(GetParam());

    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)});
    friendly_robots.emplace_back(robot_state);

    auto tactic = std::make_shared<ReceiverTactic>(pass);
    tactic->updateControlParams(pass);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions     = {};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(5));
}

INSTANTIATE_TEST_CASE_P(PassEnvironment, SimulatedReceiverTacticTest,
                        ::testing::Values(std::make_tuple(
                            Pass(Point(3.0, -0.5), Point(2.0, 2.0), 5),
                            RobotStateWithId{1, RobotState(Point(-3, -1), Vector(0, 0),
                                                           Angle::fromDegrees(0),
                                                           Angle::fromDegrees(0))},
                            BallState(Point(3.0, -0.5), Vector(0, 0)))));
