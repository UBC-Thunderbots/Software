#include "software/ai/hl/stp/play/stop_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robot_slows_down_validation.h"
#include "software/simulated_tests/validation_functions/robot_avoids_ball_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class StopPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(StopPlayTest, test_stop_play_ball_at_centre_robots_spread_out)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    Point near_friendly_goal = Point(-4,0);
    Point close_friendly_side = Point(-0.3, 0);
    Point close_enemy_side = Point(0.3, 0);
    Point close_centre_line = Point(0,0.3);
    Point far_friendly_side = Point(-3,-1.5);
    Point far_enemy_side = Point(4.6, -3.1);

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {near_friendly_goal, close_friendly_side, close_enemy_side, close_centre_line, far_friendly_side,
         far_enemy_side}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
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

// This test produced the warning, "No intent set for this tactic: MoveTactic"
TEST_F(StopPlayTest, test_stop_play_friendly_half_robots_spread_out)
{
    setBallState(BallState(Point(-1, 0), Vector(0, 0)));

    Point near_friendly_goal = Point(-4,0);
    Point close_goal_side = Point(-2.3, 0);
    Point close_offence_side = Point(-1.7, 0);
    Point close_beside_ball = Point(-2,0.3);
    Point far_friendly_side = Point(-3,-1.5);
    Point far_enemy_side = Point(4.6, -3.1);

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {near_friendly_goal, close_goal_side, close_offence_side, close_beside_ball, far_friendly_side,
         far_enemy_side}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);
*/
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

// Robots' positioning was bad, two went outside of the field and were not goal-side.
// This test produced warning, "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, test_stop_play_friendly_half_corner_robots_close_together)
{
    setBallState(BallState(Point(-4,-2.5), Vector(0, 0)));

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-3, -2.5), Point(-4, -2), Point(-2,-2.5), Point(-3,-2),
             Point(-3.5,-2), Point(-3,-1)}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);
*/
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

TEST_F(StopPlayTest, test_stop_play_enemy_half_robots_spread_out)
{
    setBallState(BallState(Point(2, 0), Vector(0, 0)));

    Point near_friendly_goal = Point(-4,0);
    Point close_goal_side = Point(1.7, 0);
    Point close_offence_side = Point(2.3, 0);
    Point close_beside_ball = Point(2,0.3);
    Point far_friendly_side = Point(-3,-1.5);
    Point far_enemy_side = Point(3, -3);

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {near_friendly_goal, close_goal_side, close_offence_side, close_beside_ball, far_friendly_side,
             far_enemy_side}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);*/

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

// This test produced the warning, "Navigator's path manager could not find a path for RobotId = 3"
TEST_F(StopPlayTest, test_stop_play_enemy_half_corner_robots_close_together)
{
    setBallState(BallState(Point(4,-2.5), Vector(0, 0)));

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(2, -2.5), Point(4, -1), Point(3,-2.5), Point(3,-2),
             Point(3.5,-2), Point(3,-1)}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);*/

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

TEST_F(StopPlayTest, test_stop_play_centre_robots_close_together)
{
    setBallState(BallState(Point(0,0), Vector(0, 0)));

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-2, 0), Point(0,0.3), Point(0.3, 0), Point(0,-0.3),
             Point(-0.3,0), Point(0.2, 0.2)}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);*/

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

// This test produced the warning, "Navigator's path manager could not find a path"
// Robot positioning was good.
TEST_F(StopPlayTest, test_stop_play_ball_in_front_of_enemy_defense_area)
{
    setBallState(BallState(Point(3,0), Vector(0, 0)));

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-4.5, 2), Point(0,0.3), Point(0.3, 0), Point(0,-0.3),
             Point(-0.3,0), Point(0.2, 0.2)}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);*/

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

// This test is disabled due to a bug that causes Robot 2 to move too close to the ball.
// Robot 2 is boxed in by defence area, Robot 5, and the ball, so it moves closer to the ball to try to make space.
// Robots' positioning was not good, two robots were on the wrong side of the ball
// This test produced the warning, "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, DISABLED_test_stop_play_ball_in_front_of_friendly_defense_area)
{
    setBallState(BallState(Point(-3,0), Vector(0, 0)));

    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
            {Point(-4.5, 2), Point(0,3), Point(-1, -1), Point(0,0),
             Point(-1,0), Point(2, 2)}));
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
                // this waits 2 seconds to allow robots that are initially too close to the ball to move away from it
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
                {
                    yield();
                }
                // TODO: #1882 implement robots slow down when responding to stop command
                /*robotSlowsDown(0, world_ptr, yield);
                robotSlowsDown(1, world_ptr, yield);
                robotSlowsDown(2, world_ptr, yield);
                robotSlowsDown(3, world_ptr, yield);
                robotSlowsDown(4, world_ptr, yield);
                robotSlowsDown(5, world_ptr, yield);*/

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
