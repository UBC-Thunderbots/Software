#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedMoveTacticTest : public SimulatedTacticTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(SimulatedMoveTacticTest, test_move_across_field)
{
    Point initial_position = Point(-3, 1.5);
    Point destination      = Point(2.5, -1.1);
    BallState ball_state(Point(4.5, -3), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            // Check that conditions hold for 1000 ticks
            unsigned num_ticks = 1000;
            for (unsigned i = 0; i < num_ticks; i++)
            {
                robotAtPosition(1, world_ptr, destination, 0.05, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_autochip_move)
{
    Point initial_position = Point(-3, 1.5);
    Point destination      = Point(0, 1.5);
    BallState ball_state(Point(0, 1.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(
        destination, Angle::zero(), 0, DribblerMode::OFF, BallCollisionType::ALLOW,
        {AutoChipOrKickMode::AUTOCHIP, 2.0}, MaxAllowedSpeedMode::TIPTOE);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            ballKicked(Angle::zero(), world_ptr, yield);
            // Check that conditions hold for 1000 ticks
            unsigned num_ticks = 1000;
            for (unsigned i = 0; i < num_ticks; i++)
            {
                robotAtPosition(1, world_ptr, destination, 0.05, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_autokick_move)
{
    Point initial_position = Point(-1, -0.5);
    Point destination      = Point(-1, -1);
    BallState ball_state(Point(-1, -1), Vector(0, 0));
    auto friendly_robots = {RobotStateWithId{
        .id          = 1,
        .robot_state = RobotState(initial_position, Vector(0, 0), Angle::threeQuarter(),
                                  AngularVelocity::zero())}};
    auto enemy_robots    = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::threeQuarter(), 0, DribblerMode::OFF,
                                BallCollisionType::ALLOW,
                                {AutoChipOrKickMode::AUTOKICK, 3.0},
                                MaxAllowedSpeedMode::TIPTOE);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            ballKicked(Angle::threeQuarter(), world_ptr, yield);
            // Check that conditions hold for 1000 ticks
            unsigned num_ticks = 1000;
            for (unsigned i = 0; i < num_ticks; i++)
            {
                robotAtPosition(1, world_ptr, destination, 0.05, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_spinning_move_clockwise)
{
    Point initial_position = Point(-4, 2);
    Point destination      = Point(4, 2);
    BallState ball_state(Point(1, 1), Vector(0, 0));
    auto friendly_robots = {RobotStateWithId{
        .id          = 1,
        .robot_state = RobotState(initial_position, Vector(0, 0), Angle::zero(),
                                  AngularVelocity::quarter())}};

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0, DribblerMode::OFF,
                                BallCollisionType::ALLOW, {AutoChipOrKickMode::OFF, 0.0},
                                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 1.0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            robotAtAngularVelocity(1, world_ptr, AngularVelocity::fromDegrees(1 * 360),
                                   AngularVelocity::fromDegrees(50), yield);
            robotAtOrientation(1, world_ptr, Angle::half(), Angle::fromDegrees(5), yield);
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            robotAtOrientation(1, world_ptr, Angle::zero(), Angle::fromDegrees(5), yield);
            while (!tactic->done())
            {
                yield("Tactic is not done");
            }
            // Check that conditions hold for 1000 ticks
            unsigned num_ticks = 1000;
            for (unsigned i = 0; i < num_ticks; i++)
            {
                robotAtPosition(1, world_ptr, destination, 0.05, yield);
                robotAtOrientation(1, world_ptr, Angle::zero(), Angle::fromDegrees(5),
                                   yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

TEST_F(SimulatedMoveTacticTest, test_spinning_move_counter_clockwise)
{
    Point initial_position = Point(4, 2);
    Point destination      = Point(-4, 2);
    BallState ball_state(Point(1, 1), Vector(0, 0));
    auto friendly_robots = {RobotStateWithId{
        .id          = 1,
        .robot_state = RobotState(initial_position, Vector(0, 0), Angle::quarter(),
                                  AngularVelocity::zero())}};

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::half(), 0, DribblerMode::OFF,
                                BallCollisionType::ALLOW, {AutoChipOrKickMode::OFF, 0.0},
                                MaxAllowedSpeedMode::PHYSICAL_LIMIT, -4.0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            robotAtAngularVelocity(1, world_ptr, AngularVelocity::fromDegrees(-4 * 360),
                                   AngularVelocity::fromDegrees(50), yield);
            robotAtOrientation(1, world_ptr, Angle::zero(), Angle::fromDegrees(5), yield);
            robotAtPosition(1, world_ptr, destination, 0.05, yield);
            robotAtOrientation(1, world_ptr, Angle::half(), Angle::fromDegrees(5), yield);
            while (!tactic->done())
            {
                yield("Tactic is not done");
            }
            // Check that conditions hold for 1000 ticks
            unsigned num_ticks = 1000;
            for (unsigned i = 0; i < num_ticks; i++)
            {
                robotAtPosition(1, world_ptr, destination, 0.05, yield);
                robotAtOrientation(1, world_ptr, Angle::half(), Angle::fromDegrees(5),
                                   yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}
