#include "software/ai/hl/stp/play/penalty_kick_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"

class PenaltyKickEnemyPlayTest
        : public SimulatedPlayTestFixture,
          public ::testing::WithParamInterface<std::tuple<RefereeCommand, RefereeCommand,
          std::vector<RobotStateWithId>>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(PenaltyKickEnemyPlayTest, test_penalty_kick_enemy_play_setup)
{
    RefereeCommand current_command = std::get<0>(GetParam());
    RefereeCommand previous_command = std::get<1>(GetParam());
    BallState ball_state(field.enemyPenaltyMark(), Vector(0, 0));

    std::vector<RobotStateWithId> friendly_robots = std::get<2>(GetParam());

    // enemy robots behind the penalty mark
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
        Point(field.enemyPenaltyMark().x() + 0.3, 0),
        Point(field.enemyPenaltyMark().x() + 1, 0),
        Point(field.enemyPenaltyMark().x() + 1, 4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, 8 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, -4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, -8 * ROBOT_MAX_RADIUS_METERS),
    });
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickEnemyPlay));
    setRefereeCommand(current_command, previous_command);
    Polygon behind_ball = Polygon({Point(field.enemyPenaltyMark().x() + 1, field.yLength() / 2),
                                   Point(field.enemyPenaltyMark().x() + 1, -field.yLength() / 2),
                                   Point(field.xLength() / 2, field.yLength() / 2),
                                   Point(-field.xLength() / 2, -field.yLength() / 2)});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [behind_ball](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotAtOrientation(0, world_ptr, Angle::zero(), Angle::fromDegrees(5), yield);
            robotAtPosition(0, world_ptr, world_ptr->field().friendlyGoalCenter(), 0.05,
                            yield);
            for (unsigned int id = 1; id <= 6; id++) {
                robotInPolygon(id, behind_ball, world_ptr, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(RobotPositions, PenaltyKickEnemyPlayTest,
                        ::testing::Values(
                                std::make_tuple(RefereeCommand::PREPARE_PENALTY_THEM, RefereeCommand::HALT,
                                        TestUtil::createStationaryRobotStatesWithId({Point(-1, -2), Point(1, 2),
                                                                                     Point(-2.5, 3), Point(2, -1),
                                                                                     Point(0, 3), Point(3, 0)})),
                                std::make_tuple(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_THEM,
                                        TestUtil::createStationaryRobotStatesWithId({Point(-0.5, -2.1), Point(2.2, 1.2),
                                                                                     Point(-2.5, 1.3), Point(1.2, -1.5),
                                                                                     Point(0, 2), Point(1, 0)}))));

TEST_F(PenaltyKickEnemyPlayTest, test_penalty_kick_enemy_play_goalie)
{
    BallState ball_state(field.enemyPenaltyMark(), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
            {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
             Point(4.6, -3.1)});

    // enemy robots behind the penalty mark
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
                                        Point(field.enemyPenaltyMark().x() + 0.3, 0),
                                        Point(field.enemyPenaltyMark().x() + 1, 0),
                                        Point(field.enemyPenaltyMark().x() + 1, 4 * ROBOT_MAX_RADIUS_METERS),
                                        Point(field.enemyPenaltyMark().x() + 1, 8 * ROBOT_MAX_RADIUS_METERS),
                                        Point(field.enemyPenaltyMark().x() + 1, -4 * ROBOT_MAX_RADIUS_METERS),
                                        Point(field.enemyPenaltyMark().x() + 1, -8 * ROBOT_MAX_RADIUS_METERS),
                                });
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickEnemyPlay));
    setRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM, RefereeCommand::HALT);
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_THEM);

    std::vector<ValidationFunction> terminating_validation_functions = {
            // This will keep the test running for 9.5 seconds to give everything enough
            // time to settle into position and be observed with the Visualizer
            // TODO: Implement proper validation
            // https://github.com/UBC-Thunderbots/Software/issues/1971
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
                {
                    yield("Timestamp not at 9.5s");
                }
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

