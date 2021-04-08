#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/non_terminating_validation_functions/enemy_never_scores_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedGoalieTacticTest : public SimulatedTacticTestFixture
{
   protected:
    void SetUp() override
    {
        SimulatedTacticTestFixture::SetUp();
        addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
                {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
                 field().enemyDefenseArea().negXNegYCorner(),
                 field().enemyDefenseArea().negXPosYCorner()}));
    }
};

TEST_F(SimulatedGoalieTacticTest, test_stationary_ball_far_away)
{
//    Point initial_position = field().friendlyGoalCenter();
    setBallState(BallState(Point(0,0), Vector(-1, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-2,1), Point(-4,-1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();

    auto tactic = std::make_shared<GoalieTactic>();
    tactic->updateControlParams(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(1);


    std::vector<ValidationFunction> terminating_validation_functions = {};
//            [this, tactic](std::shared_ptr<World> world_ptr,
//                           ValidationCoroutine::push_type& yield) {
//                while (!tactic->done())
//                {
//                    yield("Tactic not done");
//                }
//                // add terminating validation functions here
//            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            [](std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield) {
                enemyNeverScores(world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}
