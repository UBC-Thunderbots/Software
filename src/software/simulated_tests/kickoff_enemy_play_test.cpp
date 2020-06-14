#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

class KickoffEnemyPlayTest : public SimulatedTest
{
};

TEST_F(KickoffEnemyPlayTest, test_kickoff_enemy_play)
{
    World world = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setEnemyRobotPositions(
        world,
        {
            Point(1, 0),
            Point(1, 2.5),
            Point(1, -2.5),
            world.field().enemyGoalCenter(),
            world.field().enemyDefenseArea().negXNegYCorner(),
            world.field().enemyDefenseArea().negXPosYCorner(),
        },
        Timestamp::fromSeconds(0));

    world = ::TestUtil::setFriendlyRobotPositions(world,
                                                  {
                                                      Point(-3, 2.5),
                                                      Point(-3, 1.5),
                                                      Point(-3, 0.5),
                                                      Point(-3, -0.5),
                                                      Point(-3, -1.5),
                                                      Point(-3, 2.5),
                                                  },
                                                  Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().assignGoalie(0);

    world.mutableBall() = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));

    std::vector<ValidationFunction> validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1397
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableOverrideAIPlay()
        ->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableCurrentAIPlay()
        ->setValue(KickoffEnemyPlay::name);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableFriendlyGoalieId()
        ->setValue(0);

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(10));
    EXPECT_TRUE(test_passed);
}
