#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_at_position_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedMoveTacticTest : public SimulatedTacticTestFixture
{
};

TEST_F(SimulatedMoveTacticTest, test_move_across_field)
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

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              TerminatingValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_autochip_move)
{
    Point initial_position = Point(-3, 1.5);
    Point destination      = Point(0, 1.5);
    setBallState(BallState(Point(0, 1.5), Vector(0, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0, DribblerMode::OFF,
                                BallCollisionType::ALLOW,
                                {AutoChipOrKickMode::AUTOCHIP, 2.0});
    setTactic(tactic);
    setRobotId(1);

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              TerminatingValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            ballKicked(Angle::zero(), world_ptr, yield);
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_autokick_move)
{
    Point initial_position = Point(-1, 2.5);
    Point destination      = Point(-1, -1);
    setBallState(BallState(Point(-1, -1), Vector(0, 1)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0, DribblerMode::OFF,
                                BallCollisionType::ALLOW,
                                {AutoChipOrKickMode::AUTOKICK, 3.0});
    setTactic(tactic);
    setRobotId(1);

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              TerminatingValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            ballKicked(Angle::threeQuarter(), world_ptr, yield);
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
