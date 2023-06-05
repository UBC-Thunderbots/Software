#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/non_terminating_validation_functions/enemy_never_scores_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

/**
 * Simulated tests for adhering to penalty kick rules as specified in link:
 * https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick
 * */
class PenaltyKickEnemyPlayTest
    : public SimulatedErForceSimPlayTestFixture,
      public ::testing::WithParamInterface<std::tuple<
          RefereeCommand, RefereeCommand, std::vector<RobotStateWithId>, float>>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

// TODO (#2714): Re-enable tests
TEST_P(PenaltyKickEnemyPlayTest, DISABLED_test_penalty_kick_enemy_play_setup)
{
    RefereeCommand current_command  = std::get<0>(GetParam());
    RefereeCommand previous_command = std::get<1>(GetParam());
    BallState ball_state(field.enemyPenaltyMark(), Vector(0, 0));

    std::vector<RobotStateWithId> friendly_robots = std::get<2>(GetParam());
    float enemy_distance_behind_ball              = std::get<3>(GetParam());

    // enemy robots behind the penalty mark
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
        Point(field.enemyPenaltyMark().x() + 0.3, 0),  // kicker robot
        Point(field.enemyPenaltyMark().x() + enemy_distance_behind_ball, 0),
        Point(field.enemyPenaltyMark().x() + enemy_distance_behind_ball,
              4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + enemy_distance_behind_ball,
              8 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + enemy_distance_behind_ball,
              -4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + enemy_distance_behind_ball,
              -8 * ROBOT_MAX_RADIUS_METERS),
    });
    setFriendlyGoalie(0);
    setEnemyGoalie(0);
    setAiPlay(TbotsProto::PlayName::PenaltyKickEnemyPlay);
    setRefereeCommand(current_command, previous_command);
    Polygon behind_ball_region =
        Polygon({Point(field.enemyPenaltyMark().x() + 1, field.yLength() / 2),
                 Point(field.enemyPenaltyMark().x() + 1, -field.yLength() / 2),
                 Point(field.xLength() / 2, field.yLength() / 2),
                 Point(-field.xLength() / 2, -field.yLength() / 2)});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [behind_ball_region](std::shared_ptr<World> world_ptr,
                             ValidationCoroutine::push_type& yield) {
            robotAtOrientation(0, world_ptr, Angle::zero(), Angle::fromDegrees(5), yield);
            robotAtPosition(0, world_ptr, world_ptr->field().friendlyGoalCenter(), 0.05,
                            yield);
            robotInPolygon(behind_ball_region, 5, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    RobotPositions, PenaltyKickEnemyPlayTest,
    ::testing::Values(
        std::make_tuple(RefereeCommand::PREPARE_PENALTY_THEM, RefereeCommand::HALT,
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(1, 2), Point(-1, -2), Point(-2.5, 3), Point(2, -1),
                             Point(0, 3), Point(3, 0)}),
                        1),
        std::make_tuple(RefereeCommand::NORMAL_START,
                        RefereeCommand::PREPARE_PENALTY_THEM,
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.2, 1.2), Point(-0.5, -2.1), Point(-2.5, 1.3),
                             Point(1.2, -1.5), Point(0, 2), Point(1, 0)}),
                        1.3),
        std::make_tuple(RefereeCommand::NORMAL_START,
                        RefereeCommand::PREPARE_PENALTY_THEM,
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.2, 1.2), Point(-0.5, -2.1), Point(-2.5, 1.3),
                             Point(1.2, -1.5), Point(0, 2), Point(1, 0)}),
                        1.4),
        std::make_tuple(RefereeCommand::NORMAL_START,
                        RefereeCommand::PREPARE_PENALTY_THEM,
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.2, 1.2), Point(-0.5, -2.1), Point(-2.5, 1.3),
                             Point(1.2, -1.5), Point(0, 2), Point(1, 0)}),
                        1.45),
        std::make_tuple(RefereeCommand::NORMAL_START,
                        RefereeCommand::PREPARE_PENALTY_THEM,
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.2, 1.2), Point(-0.5, -2.1), Point(-2.5, 1.3),
                             Point(1.2, -1.5), Point(0, 2), Point(1, 0)}),
                        1.5),
        std::make_tuple(RefereeCommand::NORMAL_START,
                        RefereeCommand::PREPARE_PENALTY_THEM,
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.2, 1.2), Point(-0.5, -2.1), Point(-2.5, 1.3),
                             Point(1.2, -1.5), Point(0, 2), Point(1, 0)}),
                        1.6)));

TEST_F(PenaltyKickEnemyPlayTest, test_penalty_kick_enemy_play_goalie)
{
    BallState ball_state(field.enemyPenaltyMark(), Vector(-3, 0.2));

    // friendly robots already in position
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {field.friendlyGoalCenter(), Point(field.enemyPenaltyMark().x() + 1.5, 0),
         Point(field.enemyPenaltyMark().x() + 1.5, 4 * ROBOT_MAX_RADIUS_METERS),
         Point(field.enemyPenaltyMark().x() + 1.5, -4 * ROBOT_MAX_RADIUS_METERS),
         Point(field.enemyPenaltyMark().x() + 1.5, 8 * ROBOT_MAX_RADIUS_METERS),
         Point(field.enemyPenaltyMark().x() + 1.5, -8 * ROBOT_MAX_RADIUS_METERS)});

    // enemy robots behind the penalty mark
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
        Point(field.enemyPenaltyMark().x() + 0.3, 0),
        Point(field.enemyPenaltyMark().x() + 1, 0),
        Point(field.enemyPenaltyMark().x() + 1, 4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, 8 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, -4 * ROBOT_MAX_RADIUS_METERS),
        Point(field.enemyPenaltyMark().x() + 1, -8 * ROBOT_MAX_RADIUS_METERS),
    });
    setFriendlyGoalie(0);
    setEnemyGoalie(0);
    setAiPlay(TbotsProto::PlayName::PenaltyKickEnemyPlay);
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_THEM);
    GameState gameState = GameState();
    gameState.updateRefereeCommand(RefereeCommand::FORCE_START);
    setGameState(gameState);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 10 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(10))
            {
                yield("simulated penalty kick goalie test not finished!");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            enemyNeverScores(world_ptr, yield);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
