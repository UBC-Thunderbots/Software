#include "software/ai/hl/stp/play/kickoff_friendly_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/non_terminating_validation_functions/robots_in_friendly_half_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_not_in_center_circle_validation.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_center_circle_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_kicked_ball_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class KickoffFriendlyPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(KickoffFriendlyPlayTest, test_kickoff_friendly_play)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -2.5)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(KickoffFriendlyPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Two friendly robots near the half line setting up for offense
            Rectangle robotsOffensiveRect(Point(-1.75, 2.5), Point(-1.5, -2.5));
            robotInPolygon(1, robotsOffensiveRect, world_ptr, yield);
            robotInPolygon(5, robotsOffensiveRect, world_ptr, yield);


            // Two Friendly robots defending the exterior of defense box and one goalie
            Rectangle robotsDefensiveRect(Point(-3.2, 1.1), Point(-3.5, -1.1));
            robotInPolygon(0, robotsDefensiveRect, world_ptr, yield);
            robotInPolygon(2, robotsDefensiveRect, world_ptr, yield);
            robotInPolygon(3, robotsDefensiveRect, world_ptr, yield);

            // Robot 4 is the only robot allowed to be in the center circle and start
            // the kickoff
            robotInCenterCircle(4, world_ptr, yield);
            robotReceivedBall(4, world_ptr, yield);
            robotKickedBall(4, Angle::zero(), world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            Robot robot0 = world_ptr->friendlyTeam().getRobotById(0).value();
            Robot robot1 = world_ptr->friendlyTeam().getRobotById(1).value();
            Robot robot2 = world_ptr->friendlyTeam().getRobotById(2).value();
            Robot robot3 = world_ptr->friendlyTeam().getRobotById(3).value();
            Robot robot5 = world_ptr->friendlyTeam().getRobotById(5).value();

            robotInFriendlyHalf(robot0, world_ptr, yield);
            robotInFriendlyHalf(robot1, world_ptr, yield);
            robotInFriendlyHalf(robot2, world_ptr, yield);
            robotInFriendlyHalf(robot3, world_ptr, yield);
            robotInFriendlyHalf(robot5, world_ptr, yield);
            robotNotInCenterCircle(robot0, world_ptr, yield);
            robotNotInCenterCircle(robot1, world_ptr, yield);
            robotNotInCenterCircle(robot2, world_ptr, yield);
            robotNotInCenterCircle(robot3, world_ptr, yield);
            robotNotInCenterCircle(robot5, world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
