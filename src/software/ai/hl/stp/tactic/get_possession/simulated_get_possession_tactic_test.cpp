#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/get_possession/get_possession_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedMoveTacticTest : public SimulatedTacticTestFixture
{
   protected:
    void checkPossession(std::shared_ptr<GetPossessionTactic> tactic,
                         std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
    {
        while (!tactic->done())
        {
            yield();
        }
        robotReceivedBall(1, world_ptr, yield);
        auto received_ball_time = world_ptr->getMostRecentTimestamp();
        while (world_ptr->getMostRecentTimestamp() <
               received_ball_time + Duration::fromSeconds(1))
        {
            yield();
        }
        robotReceivedBall(1, world_ptr, yield);
    }
};

TEST_F(SimulatedMoveTacticTest, test_moving_ball)
{
    Point initial_position = Point(-3, 1.5);
    setBallState(BallState(Point(3, -2), Vector(-0.5, 1)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<GetPossessionTactic>();
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_stopped_ball)
{
    Point initial_position = Point(-3, 1.5);
    setBallState(BallState(Point(-1, 1.5), Vector(0, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(3, 3), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<GetPossessionTactic>();
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_ball_bounce_of_enemy_robot)
{
    Point initial_position = Point(-3, 1.5);
    setBallState(BallState(Point(0, 0), Vector(2.5, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(3, 3), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);

    auto tactic = std::make_shared<GetPossessionTactic>();
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
