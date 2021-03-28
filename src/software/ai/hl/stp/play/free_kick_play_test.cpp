#include "software/ai/hl/stp/play/free_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class FreeKickPlayTest : public SimulatedPlayTestFixture
{
};

 TEST_F(FreeKickPlayTest, test_free_kick_play_on_enemy_half)
 {
     setBallState(BallState(Point(1.5, -3), Vector(0, 0)));
     addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
         {Point(-4.5, 0), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
          Point(4, -2.5)}));

     setFriendlyGoalie(0);
     addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
         {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
          field().enemyDefenseArea().negXNegYCorner(),
          field().enemyDefenseArea().negXPosYCorner()}));

     setEnemyGoalie(0);
     setAIPlay(TYPENAME(FreeKickPlay));
     setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

     std::vector<ValidationFunction> terminating_validation_functions = {
         // This will keep the test running for 9.5 seconds to give everything enough
         // time to settle into position and be observed with the Visualizer
         // TODO: Implement proper validation
         // https://github.com/UBC-Thunderbots/Software/issues/1971
         [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
             while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
             {
                 yield("Timestamp not at 9.5s");
             }
         }};

     std::vector<ValidationFunction> non_terminating_validation_functions = {};

     runTest(terminating_validation_functions, non_terminating_validation_functions,
             Duration::fromSeconds(10));
 }

TEST_F(FreeKickPlayTest, test_free_kick_play_on_friendly_half)
{
    setBallState(BallState(Point(-1.5, -3), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 0), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(FreeKickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
