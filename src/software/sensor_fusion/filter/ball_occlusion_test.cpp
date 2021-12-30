#include <gtest/gtest.h>

#include <cmath>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/game_state.h"
#include "software/world/world.h"

class BallOcclusionTest : public SimulatedPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

/* friendly and enemy orientation are based on coordinate plane shown in field.h */

TEST_F(BallOcclusionTest, ball_vertical_friendly_left_travel_occlusion)
{
    /* ball travels from right bottom corner to right top corner */
    BallState ball_state(Point(-4.5, 3), Vector(1, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 2.8), Point(-2.5, 2.8), Point(-1, 2.8), Point(0.5, 2.8), Point(2, 2.8),
         Point(3.5, 2.8)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, ball_vertical_friendly_right_side_travel_occlusion)
{
    /* ball travels from left bottom corner to left top corner */
    BallState ball_state(Point(-4.5, -2), Vector(1, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, -1.8), Point(-2.5, -1.8), Point(-1, -1.8), Point(0.5, -1.8),
         Point(2, -1.8), Point(3.5, -1.8)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, ball_horizontal_friendly_side_travel_occlusion)
{
    BallState ball_state(Point(-4.5, 3), Vector(0, -1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.3, -2), Point(-4.3, -1), Point(-4.3, 0), Point(-4.3, 1), Point(-4.3, 2),
         Point(-4.3, 2.2)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, ball_horizontal_enemy_side_travel_occlusion)
{
    BallState ball_state(Point(4.5, 3), Vector(0, -1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(4.3, -2), Point(4.3, -1), Point(4.3, 0), Point(4.3, 1), Point(4.3, 2),
         Point(4.3, 2.2)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, simulate_shot_from_center_to_friendly)
{
    BallState ball_state(Point(-3, 0), Vector(-1, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(-2.8, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, simulate_shot_from_center_to_enemy)
{
    BallState ball_state(Point(3, 0), Vector(0, 1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(2.8, 0)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, simulate_shot_from_friendly_right_to_friendly)
{
    BallState ball_state(Point(-3.5, -1), Vector(-1, 1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3.5 + 0.2 / sqrt(2.0), -1 - 0.2 / sqrt(2.0))});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, simulate_shot_from_friendly_right_to_enemy)
{
    BallState ball_state(Point(3.5, -1), Vector(1, 1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(3.5 - 0.2 / sqrt(2.0), -1 - 0.2 / sqrt(2.0))});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, simulate_shot_from_friendly_left_to_friendly)
{
    BallState ball_state(Point(-3.5, 1), Vector(-1, -1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3.5 + 0.2 / sqrt(2.0), 1 + 0.2 / sqrt(2.0))});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, simulate_shot_from_friendly_left_to_enemy)
{
    BallState ball_state(Point(3.5, 1), Vector(1, -1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(3.5 - 0.2 / sqrt(2), 1 + 0.2 / sqrt(2))});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, pass_large_diagonal_friendly_left_corner_to_near_center)
{
    BallState ball_state(Point(-4.5 + 0.2 / sqrt(2.0), 3 - 0.2 / sqrt(2.0)),
                         Vector(3.0 / sqrt(13.0), -2.0 / sqrt(13.0)));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 3), Point(-1.5, 1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, pass_large_diagonal_friendly_right_corner_to_near_center)
{
    BallState ball_state(Point(-4.5 + 0.2 / sqrt(2.0), -3 + 0.2 / sqrt(2.0)),
                         Vector(3.0 / sqrt(13.0), 2.0 / sqrt(13.0)));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, -3), Point(-1.5, -1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, pass_large_diagonal_friendly_left_enemy_corner_to_near_center)
{
    BallState ball_state(Point(4.5 - 0.2 / sqrt(2.0), 3 - 0.2 / sqrt(2.0)),
                         Vector(-3.0 / sqrt(13.0), -2.0 / sqrt(13.0)));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 3), Point(1.5, 1)});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)});
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, pass_large_diagonal_friendly_right_enemy_corner_to_near_center)
{
    BallState ball_state(Point(4.5 - 0.2 / sqrt(2.0), -3 + 0.2 / sqrt(2.0)),
                         Vector(-3.0 / sqrt(13.0), 2.0 / sqrt(13.0)));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(4.5, -3), Point(1.5, -1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

/* one test for horizontal and vertical on one half (vertical or horizontal) should be
 * representative*/
TEST_F(BallOcclusionTest, horizontal_cross_court_pass_with_robots_close_to_path)
{
    BallState ball_state(Point(-4, -3), Vector(1, 3));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3.9, -2.9), Point(-2.9, 0.1), Point(-1.9, 3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.1, -3.1), Point(-3.1, -0.1), Point(-2.1, 2.9)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}

TEST_F(BallOcclusionTest, vertical_cross_court_pass_with_robots_close_to_path)
{
    BallState ball_state(Point(-4, -3), Vector(4, 1));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3.9, -2.9), Point(0.1, -1.9), Point(4.1, -0.9)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.1, -3.1), Point(-0.1, -2.1), Point(3.9, -1.1)});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function     = {};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(0));
}
