#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

#include <gtest/gtest.h>

#include <utility>

#include "software/simulated_tests/non_terminating_validation_functions/ball_in_play_or_scored_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/ball_never_moves_backward_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/test_util/test_util.h"
#include "software/world/field.h"

// goalie
class PenaltyKickTacticTest : public SimulatedErForceSimPlayTestFixture,
                              public ::testing::WithParamInterface<RobotStateWithId>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
    BallState ball           = BallState(field.friendlyPenaltyMark(), Vector(0, 0));
    Point initial_position   = field.friendlyPenaltyMark() + Vector(-0.1, 0);
    RobotStateWithId shooter = {
        0, RobotState(initial_position, Vector(0, 0), Angle::zero(), Angle::zero())};
    TbotsProto::AiConfig ai_config;
};

// TODO (#2232): Improve dribbling control so the ball is not lost during this test
TEST_P(PenaltyKickTacticTest, DISABLED_penalty_kick_test)
{
    RobotStateWithId enemy_robot = GetParam();

    auto tactic               = std::make_shared<PenaltyKickTactic>(ai_config);
    static RobotId shooter_id = 0;
    setTactic(shooter_id, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        friendlyScored,
    };

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        ballInPlay,
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            ballNeverMovesBackward(world_ptr, yield);
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(shooter_id, world_ptr, yield);
        }};

    runTest(field_type, ball, {shooter}, {enemy_robot}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

// TODO (#2519): fix and re-enable
TEST_F(PenaltyKickTacticTest, DISABLED_penalty_no_goalie)
{
    auto tactic               = std::make_shared<PenaltyKickTactic>(ai_config);
    static RobotId shooter_id = 0;
    setTactic(shooter_id, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        friendlyScored,
    };

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        ballInPlay,
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            ballNeverMovesBackward(world_ptr, yield);
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(shooter_id, world_ptr, yield);
        }};

    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(0, -2.5)});

    runTest(field_type, ball, {shooter}, enemy_robots, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    RobotLocations, PenaltyKickTacticTest,
    ::testing::Values(
        // enemy robot stationary at centre of goal
        RobotStateWithId{0, RobotState(Field::createSSLDivisionBField().enemyGoalCenter(),
                                       Vector(0, 0), Angle::half(), Angle::zero())},

        // enemy robot stationary left of net
        RobotStateWithId{0,
                         RobotState(Field::createSSLDivisionBField().enemyGoalpostNeg(),
                                    Vector(0, 0), Angle::half(), Angle::zero())},
        // enemy robot stationary right of net
        RobotStateWithId{0,
                         RobotState(Field::createSSLDivisionBField().enemyGoalpostPos(),
                                    Vector(0, 0), Angle::half(), Angle::zero())},
        // enemy robot left of net but moving right
        RobotStateWithId{0,
                         RobotState(Field::createSSLDivisionBField().enemyGoalpostNeg(),
                                    Vector(0, 1.2), Angle::half(), Angle::zero())},
        // enemy robot right of net but moving left
        RobotStateWithId{0,
                         RobotState(Field::createSSLDivisionBField().enemyGoalpostPos(),
                                    Vector(0, -1.2), Angle::half(), Angle::zero())}));
