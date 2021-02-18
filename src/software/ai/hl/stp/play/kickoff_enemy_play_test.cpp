#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_in_friendly_half_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_not_in_center_circle_validation.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class KickoffEnemyPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(KickoffEnemyPlayTest, test_kickoff_enemy_play)
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
    setAIPlay(TYPENAME(KickoffEnemyPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_THEM);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // TODO: Fix bug with robot three not shadowing the enemy kicker in
            // kickoff_enemy_play
            // https://github.com/UBC-Thunderbots/Software/issues/1945

            // Three Friendly robots in position to shadow enemy robots
            Rectangle robotOneShadowingRect(Point(0, 2.2), Point(-0.4, 1.8));
            Rectangle robotFiveShadowingRect(Point(0, -2.2), Point(-0.4, -1.8));
            // Rectangle robotThreeShadowingRect(Point(-0.49, 0.1), Point(-0.75,
            // -0.1));
            robotInPolygon(1, robotOneShadowingRect, world_ptr, yield);
            robotInPolygon(5, robotFiveShadowingRect, world_ptr, yield);
            // robotInPolygon(3, robotThreeShadowingRect, world_ptr);

            // Two Friendly robots defending the exterior of defense box
            Rectangle robotsDefendingRect(Point(-3.2, 1.1), Point(-3.5, -1.1));
            robotInPolygon(2, robotsDefendingRect, world_ptr, yield);
            robotInPolygon(4, robotsDefendingRect, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotsInFriendlyHalf(world_ptr, yield);
            robotsNotInCenterCircle(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
