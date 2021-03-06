#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_at_position_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedThetaStarTest : public SimulatedTacticTestFixture
{
};

TEST_F(SimulatedThetaStarTest, test_move_across_field)
{
    Point initial_position = Point(-3, 1.5);
    Point destination      = Point(1.05, 0);
    setBallState(BallState(Point(0, 0), Vector(0, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({Point(1, 0), Point(0, 0), Point(0, 0.5), Point(0, 1), Point(0, 1.5), Point(-0.7, 0)}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);
    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::set<MotionConstraint> motion_constraints;
    motion_constraints.insert(MotionConstraint::ENEMY_ROBOTS_COLLISION);
    setMotionConstraints(motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield();
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}
