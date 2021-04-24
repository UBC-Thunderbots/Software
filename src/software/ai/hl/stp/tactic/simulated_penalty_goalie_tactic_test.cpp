#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/penalty_goalie_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedPenaltyGoalieTacticTest : public SimulatedTacticTestFixture
{
};

TEST_F(SimulatedPenaltyGoalieTacticTest, test_initial_position_on_goal_line) {
    // set ball on mid-line, 6 metres away from our goal
    setBallState(BallState(Point(field().friendlyGoalCenter().x() + 6, 0),Vector(0, 0)));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);
    Point intial = Point(0, 0);
    Point position = Point(field().friendlyGoalCenter().x(), 0);

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId({intial}));

    auto tactic = std::make_shared<PenaltyGoalieTactic>(Ball(), field(), );

    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [position, tactic](std::shared_ptr<World> world_ptr,
                               ValidationCoroutine::push_type& yield) {
                while (!tactic->done()) {
                    yield("Tactic not done");
                }
                robotAtPosition(1, world_ptr, position, 0.05, yield);
                unsigned num_ticks = 1000;
                for (unsigned i = 0; i < num_ticks; i++) {
                    robotAtPosition(1, world_ptr, position, 0.05, yield);
                }
            }
    };

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions, Duration::fromSeconds(10));
}


