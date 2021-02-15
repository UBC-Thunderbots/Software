#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robots_on_friendly_half_validation.h"
#include "software/simulated_tests/validation_functions/robots_on_center_circle_validation.h"
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
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the FullSystemGUI
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1397
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield();
            }
            // check two defending robots positions
            Point robotOneExpectedPos = Point(world.field().friendlyGoalpostNeg().x() +
                    world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                    -world.field().defenseAreaYLength() / 2.0);
            Point robotTwoExpectedPos = Point(world.field().friendlyGoalpostPos().x() +
                    world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                    world.field().defenseAreaYLength() / 2.0);

            robotAtPosition(1, world_ptr, robotOneExpectedPos, 0.05, yield);
            robotAtPosition(2, world_ptr, robotTwoExpectedPos, 0.05, yield);

            // check the positions of the three robots shadowing enemy robots
            // add a TODO for robot 3 and comment the check out, since it is bugged and will not go to correct spot, thus failing the test

            robotAtPosition(3, world_ptr, robotThreeExpectedPos, 0.05, yield);
            robotAtPosition(4, world_ptr, robotFourExpectedPos, 0.05, yield);
            robotAtPosition(5, world_ptr, robotFiveExpectedPos, 0.05, yield);
            // give enemy robot some velocity to hit the ball
            // World::updateEnemyTeamState(const Team &new_enemy_team_data)
            world_ptr->enemy_team()


            // check if we touch the ball?! 

        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) 
        {
            robotsOnFriendlyHalf(world_ptr, yield);
            robotsOnCenterCircle(world_ptr, yield);
        }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
