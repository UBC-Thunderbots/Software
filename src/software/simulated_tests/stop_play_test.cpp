#include "software/ai/hl/stp/play/stop_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robot_slows_down_validation.h"
#include "software/simulated_tests/validation_functions/robot_avoids_ball_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class StopPlayTest : public SimulatedTestFixture
{
};

TEST_F(StopPlayTest, test_stop_play)
{
    setBallState(BallState(Point(0, 0.5), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(StopPlay));
    setRefereeCommand(RefereeCommand::STOP, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            // This will keep the test running for 9.5 seconds to give everything enough
            // time to settle into position and be observed with the Visualizer
            // TODO: Implement proper validation
            // https://github.com/UBC-Thunderbots/Software/issues/1396

            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                /*while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
                {
                    yield();
                }*/
                robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);

                robotAvoidsBall(0, world_ptr, yield);
                robotAvoidsBall(1, world_ptr, yield);
                robotAvoidsBall(2, world_ptr, yield);
                robotAvoidsBall(3, world_ptr, yield);
                robotAvoidsBall(4, world_ptr, yield);
                robotAvoidsBall(5, world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_enemy_half_near_ball)
{
    setBallState(BallState(Point(0, 0.5), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-3, 2.5), Point(1.5, -1.1)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
             field().enemyDefenseArea().negXNegYCorner(),
             field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(StopPlay));
    setRefereeCommand(RefereeCommand::STOP, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            // This will keep the test running for 9.5 seconds to give everything enough
            // time to settle into position and be observed with the Visualizer
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
                {
                    yield();
                }
                robotSlowsDown(1, world_ptr, yield);
                robotAvoidsBall(1, world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_friendly_half_far_from_ball)
{
    setBallState(BallState(Point(0, 0.5), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-3, 2.5), Point(-4, -2.5)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
             field().enemyDefenseArea().negXNegYCorner(),
             field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(StopPlay));
    setRefereeCommand(RefereeCommand::STOP, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            // This will keep the test running for 9.5 seconds to give everything enough
            // time to settle into position and be observed with the Visualizer
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
                {
                    yield();
                }
                robotSlowsDown(1, world_ptr, yield);
                robotAvoidsBall(1, world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_at_ball)
{
    setBallState(BallState(Point(0, 0.5), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-3, 2.5), Point(0, 0.8)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
             field().enemyDefenseArea().negXNegYCorner(),
             field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(StopPlay));
    setRefereeCommand(RefereeCommand::STOP, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            // This will keep the test running for 9.5 seconds to give everything enough
            // time to settle into position and be observed with the Visualizer
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
                {
                    yield();
                }
                robotSlowsDown(1, world_ptr, yield);
                robotAvoidsBall(1, world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_robots_too_fast)
{

}
