#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/non_terminating_validation_functions/enemy_never_scores_validation.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
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

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_ball_panic)
{
      setBallState(BallState(Point(0,0), Vector(-3,0)));
      addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-2,1), Point(-4,-1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    Angle chip_angle = (field().enemyGoalCenter() - field().friendlyGoalCenter()).orientation();

    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    tactic->updateControlParams(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic, chip_angle](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                // add terminating validation functions here
                ballKicked(chip_angle, world_ptr, yield);
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            [](std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield) {
                enemyNeverScores(world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedGoalieTacticTest, test_slow_ball_chip)
{
    setBallState(BallState(Point(-4,0.8), Vector(-0.5, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-2,1), Point(-4,0)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
//    Angle chip_angle = (field().enemyGoalCenter() - field().friendlyGoalCenter()).orientation();

    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    tactic->updateControlParams(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                                       ValidationCoroutine::push_type& yield) {
//                while (!tactic->done())
//                {
//                    yield("Tactic not done");
//                }
                // add terminating validation functions here
                robotReceivedBall(1, world_ptr, yield);
//                ballKicked(chip_angle, world_ptr, yield);
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            [](std::shared_ptr<World> world_ptr,
               ValidationCoroutine::push_type& yield) {
                enemyNeverScores(world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedGoalieTacticTest, test_dribble_then_chip)
{
    setBallState(BallState(field().friendlyGoalCenter() + Vector(0.1, 0.1), Vector(0, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-2,1), Point(-4,-1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    Angle chip_angle = (field().enemyGoalCenter() - field().friendlyGoalCenter()).orientation();

    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    tactic->updateControlParams(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, chip_angle, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
//                while (!tactic->done())
//                {
//                    yield("Tactic not done");
//                }
                // add terminating validation functions here
                robotReceivedBall(1, world_ptr, yield);
                ballKicked(chip_angle, world_ptr, yield);
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
            [](std::shared_ptr<World> world_ptr,
               ValidationCoroutine::push_type& yield) {
                enemyNeverScores(world_ptr, yield);
            }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}



