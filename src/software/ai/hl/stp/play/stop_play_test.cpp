#include "software/ai/hl/stp/play/stop_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/non_terminating_validation_functions/robots_avoid_ball_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_slow_down_validation.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class StopPlayTest : public SimulatedPlayTestFixture
{
   protected:
    StopPlayTest() : stop_play_rules(initStopPlayRules()) {}

    std::vector<ValidationFunction> stop_play_rules;


    std::vector<ValidationFunction> initStopPlayRules()
    {
        return {
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                // Wait 2 seconds for robots that start too close to the ball to move away
                if (world_ptr->getMostRecentTimestamp() >= Timestamp::fromSeconds(2))
                {
                    // TODO: (#1882) implement robots slow down for stop play
                    // robotsSlowDown(1.5, world_ptr, yield);
                    robotsAvoidBall(0.5, {}, world_ptr, yield);
                }
            }};
    }

    void SetUp() override
    {
        SimulatedPlayTestFixture::SetUp();
        setFriendlyGoalie(0);
        setEnemyGoalie(0);
        setAIPlay(TYPENAME(StopPlay));
        setRefereeCommand(RefereeCommand::STOP, RefereeCommand::STOP);
    }
    Field field = Field::createSSLDivisionBField();
    std::vector<RobotStateWithId> enemy_robots =
        TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
             field.enemyDefenseArea().negXNegYCorner(),
             field.enemyDefenseArea().negXPosYCorner()});
};

// TODO: (#1948) fix warning message "No intent set for this tactic: MoveTactic"
TEST_F(StopPlayTest, test_stop_play_ball_at_centre_robots_spread_out)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), Point(-0.3, 0), Point(0.3, 0), Point(0, 0.3), Point(-3, -1.5),
         Point(4.6, -3.1)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// TODO: (#1948) fix warning message "No intent set for this tactic: MoveTactic"
TEST_F(StopPlayTest, test_stop_play_friendly_half_robots_spread_out)
{
    BallState ball_state(Point(-1, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), Point(-2.3, 0), Point(-1.7, 0), Point(-2, 0.3), Point(-3, -1.5),
         Point(4.6, -3.1)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// TODO: (#1948) fix warning message "Navigator's path manager could not find a path" and
// improve robot positioning
TEST_F(StopPlayTest, test_stop_play_friendly_half_corner_robots_close_together)
{
    BallState ball_state(Point(-4, -2.5), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, -2.5), Point(-4, -2), Point(-2, -2.5), Point(-3, -2), Point(-3.5, -2),
         Point(-3, -1)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_enemy_half_robots_spread_out)
{
    BallState ball_state(Point(2, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), Point(1.7, 0), Point(2.3, 0), Point(2, 0.3), Point(-3, -1.5),
         Point(3, -3)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// TODO: (#1948) fix warning message "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, test_stop_play_enemy_half_corner_robots_close_together)
{
    BallState ball_state(Point(4, -2.5), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(2, -2.5), Point(4, -1), Point(3, -2.5), Point(3, -2), Point(3.5, -2),
         Point(3, -1)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(StopPlayTest, test_stop_play_centre_robots_close_together)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-2, 0), Point(0, 0.3), Point(0.3, 0), Point(0, -0.3), Point(-0.3, 0),
         Point(0.2, 0.2)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// TODO: #1948 fix warning message "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, test_stop_play_ball_in_front_of_enemy_defense_area)
{
    BallState ball_state(Point(3, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 2), Point(0, 0.3), Point(0.3, 0), Point(0, -0.3), Point(-0.3, 0),
         Point(0.2, 0.2)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// TODO: (#1948) Re-enable test once bug is fixed, fix warning and robot positioning
// This test is disabled due to a bug that causes Robot 2 to move too close to the ball.
// Robots' positioning was not good, two robots were on the wrong side of the ball.
// This test produced the warning, "Navigator's path manager could not find a path"
TEST_F(StopPlayTest, DISABLED_test_stop_play_ball_in_front_of_friendly_defense_area)
{
    BallState ball_state(Point(-3, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 2), Point(0, 3), Point(-1, -1), Point(0, 0), Point(-1, 0),
         Point(2, 2)});

    std::vector<ValidationFunction> terminating_validation_functions = {};
    std::vector<ValidationFunction> non_terminating_validation_functions =
        stop_play_rules;

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
