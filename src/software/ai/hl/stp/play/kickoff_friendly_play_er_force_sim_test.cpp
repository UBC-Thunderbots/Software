#include <gtest/gtest.h>

#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_in_friendly_half_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_not_in_center_circle_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_center_circle_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class KickoffFriendlyPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(KickoffFriendlyPlayTest, test_kickoff_friendly_play)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -2.5)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlayConstructor([this]() {
        return std::make_unique<KickoffFriendlyPlay>(thunderbots_config->getPlayConfig());
    });
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Robot 9 is the only robot allowed to be in the center circle and start
            // the kickoff
            robotInCenterCircle(world_ptr, yield);
            robotReceivedBall(world_ptr, yield);
            ballKicked(Angle::zero(), world_ptr, yield);

            // Two Friendly robots defending the exterior of defense box and one goalie
            Rectangle robots_defensive_rect(Point(-4, 2), Point(-5, -2));
            robotInPolygon(robots_defensive_rect, 3, world_ptr, yield);

            // Two friendly robots near the half line setting up for offense
            Rectangle robots_offensive_rect(Point(0, 3.5), Point(-1.5, -3.5));
            robotInPolygon(robots_offensive_rect, 2, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            for (RobotId robot_id : {0, 1, 2, 3, 4, 5, 6, 7, 8, 10})
            {
                {
                    robotInFriendlyHalf(robot_id, world_ptr, yield);
                    robotNotInCenterCircle(robot_id, world_ptr, yield);
                }
            }
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
