#include "software/ai/hl/stp/play/penalty_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_at_position_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/ball_in_play_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class PenaltyKickPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(PenaltyKickPlayTest, test_penalty_kick_setup)
{
    setBallState(BallState(field().enemyPenaltyMark(), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-2, -2), Point(-3, -1), Point(-3, 0), Point(-3, 1), Point(-3, 2),
         Point(2, 2.5)}));
    setFriendlyGoalie(0);
    addEnemyRobots(
        TestUtil::createStationaryRobotStatesWithId({field().enemyGoalCenter()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickPlay));
    setRefereeCommand(RefereeCommand::PREPARE_PENALTY_US, RefereeCommand::NORMAL_START);

    RobotId shooter_id = 5;
    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1396
		[shooter_id](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            auto friendly_robots_except_shooter_1_meter_from_ball =
                [shooter_id](std::shared_ptr<World> world_ptr) {
                    Point ball_position = world_ptr->ball().position();
                    for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
                    {
						if ((robot.id() != shooter_id) &&
							((robot.position()-ball_position).length() < 1))
                        {
							return false;
                        }
                    }
                return true;
                };

            while (!friendly_robots_except_shooter_1_meter_from_ball(world_ptr))
            {
				yield();
			}},
		[shooter_id](std::shared_ptr<World> world_ptr,
		   ValidationCoroutine::push_type& yield) {
			robotAtPosition(shooter_id, world_ptr,
			 			    world_ptr->field().enemyPenaltyMark(), 0.4, 
                            yield);
		}};     

    std::vector<ValidationFunction> non_terminating_validation_functions = {
		[](std::shared_ptr<World> world_ptr,
		   ValidationCoroutine::push_type& yield)
		{
			ASSERT_EQ(world_ptr->field().enemyPenaltyMark(), world_ptr->ball().position());
		}
	};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(9.5));
}

TEST_F(PenaltyKickPlayTest, test_penalty_kick_take)
{
    Vector behind_ball_direction =
        (field().enemyPenaltyMark() - field().enemyGoalpostPos()).normalize();

    Point behind_ball = field().enemyPenaltyMark() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS + 0.1);
    double non_shooter_x_pos = field().enemyPenaltyMark().x() - 1.5;
    setBallState(BallState(field().enemyPenaltyMark(), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), behind_ball, Point(non_shooter_x_pos, 0),
         Point(non_shooter_x_pos, 1), Point(non_shooter_x_pos, -1),
         Point(non_shooter_x_pos, 2)}));
    setFriendlyGoalie(0);
    Point goalie = Point(field().enemyGoalCenter().x(), 0);
    addEnemyRobots(
		   TestUtil::createStationaryRobotStatesWithId({goalie}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test srunning for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1396
        [](std::shared_ptr<World> world_ptr, 
        ValidationCoroutine::push_type& yield) {
		  friendlyScored(world_ptr, yield);
	  }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
        {
            ballInPlay(world_ptr, yield);
        }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
