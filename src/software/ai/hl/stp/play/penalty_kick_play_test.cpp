#include "software/ai/hl/stp/play/penalty_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/non_terminating_validation_functions/ball_in_play_or_scored_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/ball_never_moves_backward_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_avoid_ball_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class PenaltyKickPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_F(PenaltyKickPlayTest, test_penalty_kick_setup)
{
    BallState ball_state(field.friendlyPenaltyMark(), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-2, -2), Point(-3, -1), Point(-3, 0), Point(-3, 1), Point(-3, 2),
         Point(2, 2.5)});
    setFriendlyGoalie(0);
    auto enemy_robots =
        TestUtil::createStationaryRobotStatesWithId({field.enemyGoalCenter()});
    setEnemyGoalie(0);
    setAIPlay(TbotsProto::PlayName::PenaltyKickPlay);
    setRefereeCommand(RefereeCommand::PREPARE_PENALTY_US, RefereeCommand::NORMAL_START);

    RobotId shooter_id                                               = 5;
    std::vector<ValidationFunction> terminating_validation_functions = {
        [shooter_id](std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield) {
            robotAtPosition(shooter_id, world_ptr,
                            world_ptr->field().friendlyPenaltyMark(), 0.3, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // making sure that the robot doesn't move the ball while the penalty is
            // setting up
            ASSERT_TRUE(
                TestUtil::equalWithinTolerance(world_ptr->field().friendlyPenaltyMark(),
                                               world_ptr->ball().position(), 1e-6));
        },
        [shooter_id](std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield) {
            // Wait 2 seconds for robots to start moving adequately far away from the ball
            if (world_ptr->getMostRecentTimestamp() >= Timestamp::fromSeconds(2))
            {
                robotsAvoidBall(1, {shooter_id}, world_ptr, yield);
            }
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(9.5));
}

// TODO (#2167): disabled due to physics simulator bug causing failure
TEST_F(PenaltyKickPlayTest, DISABLED_test_penalty_kick_take)
{
    Vector behind_ball_direction =
        (field.friendlyPenaltyMark() - field.enemyGoalCenter()).normalize();

    Point behind_ball = field.friendlyPenaltyMark() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS + 0.1);
    double non_shooter_x_pos = field.friendlyPenaltyMark().x() - 1.5;
    BallState ball_state(field.friendlyPenaltyMark(), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), behind_ball, Point(non_shooter_x_pos, 0),
         Point(non_shooter_x_pos, 1), Point(non_shooter_x_pos, -1),
         Point(non_shooter_x_pos, 2)});
    setFriendlyGoalie(0);
    Point goalie      = Point(field.enemyGoalpostNeg().x(), 0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({goalie});
    setEnemyGoalie(0);
    setAIPlay(TbotsProto::PlayName::PenaltyKickPlay);
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        friendlyScored,
    };

    RobotId shooter_id                                                   = 1;
    std::vector<ValidationFunction> non_terminating_validation_functions = {
        ballInPlay,
        [shooter_id](std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield) {
            ballNeverMovesBackward(world_ptr, yield);
        },
        [shooter_id](std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(shooter_id, world_ptr, yield);
        },
        [shooter_id](std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield) {
            robotsAvoidBall(1, {shooter_id}, world_ptr, yield);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
