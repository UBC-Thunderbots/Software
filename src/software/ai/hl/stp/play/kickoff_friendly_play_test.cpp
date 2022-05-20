#include "software/ai/hl/stp/play/kickoff_friendly_play.h"

#include <gtest/gtest.h>

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
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

// TODO (#2608): re-enable when fixed
TEST_F(KickoffFriendlyPlayTest, DISABLED_test_kickoff_friendly_play)
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
    setAIPlay(std::move(std::make_unique<KickoffFriendlyPlay>(getAiConfig())));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Robot 4 is the only robot allowed to be in the center circle and start
            // the kickoff
            robotInCenterCircle(world_ptr, yield);
            robotReceivedBall(world_ptr, yield);
            ballKicked(Angle::zero(), world_ptr, yield);

            // Two friendly robots near the half line setting up for offense
            Rectangle robotsOffensiveRect(Point(-0.5, 2.5), Point(-1.5, -2.5));
            robotInPolygon(robotsOffensiveRect, 2, world_ptr, yield);


            // Two Friendly robots defending the exterior of defense box and one goalie
            Rectangle robotsDefensiveRect(Point(-3.2, 1.1), Point(-3.51, -1.1));
            robotInPolygon(robotsDefensiveRect, 3, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            for (RobotId robot_id : {0, 1, 2, 3, 5})
            {
                {
                    robotInFriendlyHalf(robot_id, world_ptr, yield);
                    robotNotInCenterCircle(robot_id, world_ptr, yield);
                }
            }
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
