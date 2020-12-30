#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_action_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robot_at_position_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedMoveActionTest : public SimulatedActionTestFixture
{
};

TEST_F(SimulatedMoveActionTest, test_move_across_field)
{
    Point initial_position = Point(-3, 1.5);
    Point destination      = Point(2.5, -1.1);
    setBallState(BallState(Point(4.5, -3), Vector(0, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto action = std::make_shared<MoveAction>(false);
    action->updateControlParams(Robot(1,
                                      RobotState(Point(-3, 1.5), Vector(0, 0),
                                                 Angle::zero(), AngularVelocity::zero()),
                                      Timestamp::fromSeconds(0)),
                                destination, Angle::zero(), 0, DribblerMode::OFF,
                                BallCollisionType::AVOID);
    setAction(action);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, action](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            while (!action->done())
            {
                yield();
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
