#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robots_in_friendly_half_validation.h"
#include "software/simulated_tests/validation_functions/robots_not_in_center_circle_validation.h"
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
            auto robotsShadowingEnemy = [](std::shared_ptr<World> world_ptr) -> bool {
                // TODO: Fix bug with robot three not shadowing the enemy kicker in
                // kickoff_enemy_play
                // https://github.com/UBC-Thunderbots/Software/issues/1945

                Rectangle robotOneShadowingRect(Point(0, 2.2), Point(-0.4, 1.8));
                Rectangle robotFiveShadowingRect(Point(0, -2.2), Point(-0.4, -1.8));

                Point robotOnePos =
                    world_ptr->friendlyTeam().getRobotById(1).value().position();
                Point robotFivePos =
                    world_ptr->friendlyTeam().getRobotById(5).value().position();

                return contains(robotOneShadowingRect, robotOnePos) &&
                       contains(robotFiveShadowingRect, robotFivePos);
            };

            auto robotsDefendingPosts = [](std::shared_ptr<World> world_ptr) -> bool {
                // Positions taken from kickoff_enemy_play
                Point robotTwoExpectedPos =
                    Point(world_ptr->field().friendlyGoalpostPos().x() +
                              world_ptr->field().defenseAreaXLength() +
                              2 * ROBOT_MAX_RADIUS_METERS,
                          world_ptr->field().defenseAreaYLength() / 2.0);
                Point robotFourExpectedPos =
                    Point(world_ptr->field().friendlyGoalpostNeg().x() +
                              world_ptr->field().defenseAreaXLength() +
                              2 * ROBOT_MAX_RADIUS_METERS,
                          -world_ptr->field().defenseAreaYLength() / 2.0);

                double tolerance = 0.05;

                Circle robotTwoCircle(robotTwoExpectedPos, tolerance);
                Circle robotFourCircle(robotFourExpectedPos, tolerance);

                Point robotTwoPos =
                    world_ptr->friendlyTeam().getRobotById(2).value().position();
                Point robotFourPos =
                    world_ptr->friendlyTeam().getRobotById(4).value().position();

                return contains(robotTwoCircle, robotTwoPos) &&
                       contains(robotFourCircle, robotFourPos);
            };

            while (!robotsDefendingPosts(world_ptr) || !robotsShadowingEnemy(world_ptr))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotsInFriendlyHalf(world_ptr, yield);
            robotsNotInCenterCircle(world_ptr, yield);
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
