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
    void checkClearanceSuccess(std::shared_ptr<World> world_ptr,
                               ValidationCoroutine::push_type& yield)
    {
        auto clearance_time = world_ptr->getMostRecentTimestamp();
        while (world_ptr->getMostRecentTimestamp() <
                clearance_time + Duration::fromSeconds(1))
        {
            yield("Waiting 1 second to see if enemy did not score");
        }
    }
    void SetUp() override
    {
        SimulatedTacticTestFixture::SetUp();
        addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
                {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
                 field().enemyDefenseArea().negXNegYCorner(),
                 field().enemyDefenseArea().negXPosYCorner()}));
    }
};

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_panic_ball_very_fast_in_straight_line)
{
      setBallState(BallState(Point(0,0), Vector(-3,0)));
      addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-4,-1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();

    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
//    tactic->updateControlParams(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                checkClearanceSuccess(world_ptr, yield);
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

// clears ball but does not register it as "received"
TEST_F(SimulatedGoalieTacticTest, DISABLED_test_panic_ball_very_fast_in_diagonal_line)
{
    setBallState(BallState(Point(0, 0), Vector(-4.5, 0.25)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({field().friendlyGoalCenter() + Vector(0, -0.5)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(std::make_shared<const GoalieTacticConfig>());
    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
//                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                checkClearanceSuccess(world_ptr, yield);
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

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_slow_ball_in_friendly_defense_area)
{
    setBallState(BallState(Point(-4,0.8), Vector(-0.5, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-4,0)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    Angle clear_angle = Vector(1, 0.5).orientation();
    tactic->updateControlParams(goalie_tactic_config, std::nullopt, clear_angle);

    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, clear_angle, tactic](std::shared_ptr<World> world_ptr,
                                       ValidationCoroutine::push_type& yield) {
                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                ballKicked(clear_angle, world_ptr, yield);
                checkClearanceSuccess(world_ptr, yield);
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

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_stationary_ball_in_friendly_defense_area)
{
    setBallState(BallState(Point(-4,0), Vector(0, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({field().friendlyGoalpostPos()}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    Angle clear_angle = Vector(1, 0).orientation();
    tactic->updateControlParams(goalie_tactic_config, std::nullopt, clear_angle);

    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, clear_angle, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                ballKicked(clear_angle, world_ptr, yield);
                checkClearanceSuccess(world_ptr, yield);
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

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_stationary_ball_inside_no_chip_rectangle)
{
    setBallState(BallState(field().friendlyGoalCenter() + Vector(0.1, 0.1), Vector(0, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-4,-1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    Point clear_origin = field().friendlyGoalpostPos() + Vector(0.5, 0);
    Angle clear_angle = Vector(1,0).orientation();
    tactic->updateControlParams(goalie_tactic_config, clear_origin, clear_angle);

    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, clear_angle, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                ballKicked(clear_angle, world_ptr, yield);
                checkClearanceSuccess(world_ptr, yield);
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

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_fast_ball_inside_no_chip_rectangle)
{
    setBallState(BallState(field().friendlyGoalCenter() + Vector(0.09, 0), Vector(0, -0.5)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-3.5,1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);

    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                ballKicked((world_ptr->ball().position() - field().friendlyGoalCenter()).orientation(), world_ptr, yield);
                checkClearanceSuccess(world_ptr, yield);
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

TEST_F(SimulatedGoalieTacticTest, DISABLED_test_slow_ball_inside_no_chip_rectangle)
{
    setBallState(BallState(field().friendlyGoalCenter() + Vector(0.1, 0), Vector(0.1, -0.1)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-3.5,1)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    Point chip_origin = field().friendlyGoalpostNeg() + Vector(0.5, 0);
    tactic->updateControlParams(goalie_tactic_config, chip_origin, std::nullopt);

    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                robotReceivedBall(0, world_ptr, yield);
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
                ballKicked((world_ptr->ball().position() - field().friendlyGoalCenter()).orientation(), world_ptr, yield);
                checkClearanceSuccess(world_ptr, yield);
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

// does not pass because never leaves position_to_block state
TEST_F(SimulatedGoalieTacticTest, DISABLED_test_ball_very_fast_misses_net)
{
    setBallState(BallState(Point(0, 0), Vector(-4, 1)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-4.5,0)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
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

// does not pass because never leaves position_to_block state
TEST_F(SimulatedGoalieTacticTest, DISABLED_test_slow_ball_at_sharp_angle_to_friendly_goal)
{
    setBallState(BallState(Point(-4.5, -3), Vector(0, 0.1)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)}));

    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config = std::make_shared<const GoalieTacticConfig>();
    auto tactic = std::make_shared<GoalieTactic>(goalie_tactic_config);
    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [this, tactic](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
                while (!tactic->done())
                {
                    yield("Tactic not done");
                }
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
