#include "software/ai/hl/stp/play/stop_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robots_avoid_ball_validation.h"
#include "software/simulated_tests/validation_functions/robots_slow_down_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class StopPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(StopPlayTest, test_stop_play_ball_at_centre_robots_spread_out)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), Point(-0.3, 0), Point(0.3, 0), Point(0, 0.3), Point(-3, -1.5),
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// This test produced the warning, "No intent set for this tactic: MoveTactic"
TEST_F(StopPlayTest, test_stop_play_friendly_half_robots_spread_out)
{
    setBallState(BallState(Point(-1, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), Point(-2.3, 0), Point(-1.7, 0), Point(-2, 0.3), Point(-3, -1.5),
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// Robots' positioning was bad, two went outside of the field and were not goal-side.
// This test produced warning, "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, test_stop_play_friendly_half_corner_robots_close_together)
{
    setBallState(BallState(Point(-4, -2.5), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, -2.5), Point(-4, -2), Point(-2, -2.5), Point(-3, -2), Point(-3.5, -2),
         Point(-3, -1)}));
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_enemy_half_robots_spread_out)
{
    setBallState(BallState(Point(2, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), Point(1.7, 0), Point(2.3, 0), Point(2, 0.3), Point(-3, -1.5),
         Point(3, -3)}));
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// This test produced the warning, "Navigator's path manager could not find a path for
// RobotId = 3"
TEST_F(StopPlayTest, test_stop_play_enemy_half_corner_robots_close_together)
{
    setBallState(BallState(Point(4, -2.5), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(2, -2.5), Point(4, -1), Point(3, -2.5), Point(3, -2), Point(3.5, -2),
         Point(3, -1)}));
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_centre_robots_close_together)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-2, 0), Point(0, 0.3), Point(0.3, 0), Point(0, -0.3), Point(-0.3, 0),
         Point(0.2, 0.2)}));
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// This test produced the warning, "Navigator's path manager could not find a path"
// Robot positioning was good.
TEST_F(StopPlayTest, test_stop_play_ball_in_front_of_enemy_defense_area)
{
    setBallState(BallState(Point(3, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 2), Point(0, 0.3), Point(0.3, 0), Point(0, -0.3), Point(-0.3, 0),
         Point(0.2, 0.2)}));
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// This test is disabled due to a bug that causes Robot 2 to move too close to the ball.
// Robot 2 is boxed in by defence area, Robot 5, and the ball, so it moves closer to the
// ball to try to make space. Robots' positioning was not good, two robots were on the
// wrong side of the ball.
// This test produced the warning, "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, DISABLED_test_stop_play_ball_in_front_of_friendly_defense_area)
{
    setBallState(BallState(Point(-3, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 2), Point(0, 3), Point(-1, -1), Point(0, 0), Point(-1, 0),
         Point(2, 2)}));
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
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots that start too close to the ball to move away
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield();
            }
            // TODO: #1882 implement robots slow down when responding to stop command
            // robotsSlowDown(world_ptr, yield);
            robotsAvoidBall(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
