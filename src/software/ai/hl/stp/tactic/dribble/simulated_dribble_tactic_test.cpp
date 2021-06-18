#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedDribbleTacticTest : public SimulatedTacticTestFixture
{
   protected:
    void checkPossession(std::shared_ptr<DribbleTactic> tactic,
                         std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
    {
        while (!tactic->done())
        {
            yield("Tactic not done");
        }
        robotReceivedBall(1, world_ptr, yield);
        auto received_ball_time = world_ptr->getMostRecentTimestamp();
        while (world_ptr->getMostRecentTimestamp() <
               received_ball_time + Duration::fromSeconds(1))
        {
            yield("Waiting 1 second to see if possession is maintained");
        }
        robotReceivedBall(1, world_ptr, yield);
    }

    void SetUp() override
    {
        SimulatedTacticTestFixture::SetUp();
        setMotionConstraints({MotionConstraint::ENEMY_DEFENSE_AREA});
    }
    Field field = Field::createSSLDivisionBField();
    std::vector<RobotStateWithId> enemy_robots =
        TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
             field.enemyDefenseArea().negXNegYCorner(),
             field.enemyDefenseArea().negXPosYCorner()});
};

TEST_F(SimulatedDribbleTacticTest, test_intercept_ball_behind_enemy_robot)
{
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(3, -2), Vector(-0.5, 1));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedDribbleTacticTest, test_stopped_ball)
{
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(-1, 1.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(3, 3), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedDribbleTacticTest, test_ball_bounce_off_of_enemy_robot)
{
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(0, 0), Vector(2.5, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(3, 3), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedDribbleTacticTest, test_moving_ball_dribble_dest)
{
    Point initial_position    = Point(-3, 1.5);
    Point dribble_destination = Point(-3, 1);
    BallState ball_state(Point(3, -2), Vector(-1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(dribble_destination, std::nullopt);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, tactic](std::shared_ptr<World> world_ptr,
                                            ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedDribbleTacticTest, test_moving_ball_dribble_orientation)
{
    Point initial_position    = Point(-3, 1.5);
    Angle dribble_orientation = Angle::quarter();
    BallState ball_state(Point(3, -2), Vector(-1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(std::nullopt, dribble_orientation);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_orientation, tactic](std::shared_ptr<World> world_ptr,
                                            ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedDribbleTacticTest, test_moving_ball_dribble_dest_and_orientation)
{
    Point initial_position    = Point(-2, 1.5);
    Point dribble_destination = Point(-1, 2);
    Angle dribble_orientation = Angle::zero();
    BallState ball_state(Point(2, -2), Vector(1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(dribble_destination, dribble_orientation);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}

TEST_F(SimulatedDribbleTacticTest, test_dribble_dest_and_orientation_around_rectangle)
{
    Point initial_position    = Point(3, -3);
    Point dribble_destination = Point(4, 2.5);
    Angle dribble_orientation = Angle::half();
    BallState ball_state(Point(4, -2.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});
    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(dribble_destination, dribble_orientation);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}

TEST_F(SimulatedDribbleTacticTest,
       test_dribble_dest_and_orientation_around_rectangle_with_excessive_dribbling)
{
    Point initial_position    = Point(3, -3);
    Point dribble_destination = Point(4, 2.5);
    Angle dribble_orientation = Angle::half();
    BallState ball_state(Point(4, -2.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});
    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(dribble_destination, dribble_orientation, true);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedDribbleTacticTest, test_running_into_enemy_robot_knocking_ball_away)
{
    Point initial_position    = Point(-2, 1.5);
    Point dribble_destination = Point(-1, 2);
    Angle dribble_orientation = Angle::half();
    BallState ball_state(Point(2, -2), Vector(1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});
    enemy_robots.emplace_back(RobotStateWithId{
        .id          = 7,
        .robot_state = RobotState(Point(1, 1.1), Vector(), Angle::fromDegrees(-30),
                                  AngularVelocity::zero())});

    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(dribble_destination, dribble_orientation);
    // Don't avoid enemy robots to knock ball away
    setMotionConstraints({});
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}
