#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_violating_motion_constraint.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_stationary_in_polygon_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedHRVOTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_F(SimulatedHRVOTest, test_drive_in_straight_line_with_moving_enemy_robot_from_behind)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(-2.3, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-0, 1), initial_position});
    auto enemy_robots =
        TestUtil::createMovingRobotStatesWithId({Point(-4.2, 0)}, {Vector(5, 0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(6));
}

TEST_F(SimulatedHRVOTest, test_drive_in_straight_line_with_moving_enemy_robot_from_side)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(-2.5, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots =
        TestUtil::createMovingRobotStatesWithId({Point(-1, -3.5)}, {Vector(0, 6)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(6));
}

TEST_F(SimulatedHRVOTest, test_drive_in_straight_line_with_no_obstacle)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(-2.5, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(-2, -2)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(6));
}

TEST_F(SimulatedHRVOTest, test_drive_in_straight_line_with_friendly_robot_infront)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(-2.5, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(2, 0), initial_position});
    std::vector<RobotStateWithId> enemy_robots = {};

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(8));
}

TEST_F(SimulatedHRVOTest, test_single_enemy_directly_infront)
{
    Point destination      = Point(2, 0);
    Point initial_position = Point(0.7, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(6));
}

TEST_F(SimulatedHRVOTest, test_three_robot_wall)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(0, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 0.3), Point(1, -0.3)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(8));
}

TEST_F(SimulatedHRVOTest, test_zig_zag_movement)
{
    // The x value of the wall in front of the friendly robot
    int front_wall_x = -2;
    // each gate refers to the center to center distance between each wall and the front
    // wall The constant offsets can be tweaked to get different distances between each
    // wall
    int gate_1           = 1;
    int gate_2           = gate_1 + 2;
    int gate_3           = gate_2 + 1;
    double robot_y_delta = 0.2;

    Point destination      = Point(front_wall_x + gate_3 + 0.5, 0);
    Point initial_position = Point(front_wall_x - 0.5, 0);
    BallState ball_state(Point(0, -2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(front_wall_x, 0.0), Point(front_wall_x, robot_y_delta),
         Point(front_wall_x, 2 * robot_y_delta), Point(front_wall_x + gate_1, 0.0),
         Point(front_wall_x + gate_1, -robot_y_delta),
         Point(front_wall_x + gate_1, -2 * robot_y_delta),
         Point(front_wall_x + gate_2, 0.0), Point(front_wall_x + gate_2, robot_y_delta),
         Point(front_wall_x + gate_2, 2 * robot_y_delta),
         Point(front_wall_x + gate_3, 0.0), Point(front_wall_x + gate_3, -robot_y_delta),
         Point(front_wall_x + gate_3, -2 * robot_y_delta)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(SimulatedHRVOTest, test_agent_not_going_in_static_obstacles)
{
    Point destination      = Point(2.9, 1);
    Point initial_position = Point(2.9, -1);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(2, 2), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(2, 0.0), Point(2.2, 0.0), Point(2.4, 0.0), Point(2.6, 0.0),
         Point(2.8, 0.0), Point(3, 0.0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::shared_ptr<RobotNavigationObstacleFactory> obstacle_factory =
        std::make_shared<RobotNavigationObstacleFactory>(
            TbotsProto::RobotNavigationObstacleConfig());

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic, obstacle_factory](std::shared_ptr<World> world_ptr,
                                                ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [obstacle_factory](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
            robotsViolatingMotionConstraint(
                world_ptr, yield, obstacle_factory,
                TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}

TEST_F(SimulatedHRVOTest, test_start_in_local_minima)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(0.7, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 0.3), Point(1, 0.6), Point(0.7, 0.6), Point(1, -0.3),
         Point(1, -0.6), Point(0.7, -0.6)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}

TEST_F(SimulatedHRVOTest, test_start_in_local_minima_with_open_end)
{
    Point destination      = Point(2.8, 0);
    Point initial_position = Point(0.7, 0);
    BallState ball_state(Point(1, 2), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1.5, 0), Point(1, 0.3), Point(1, 0.6), Point(0.7, 0.6), Point(1, -0.3),
         Point(1, -0.6), Point(0.7, -0.6)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(7));
}

TEST_F(SimulatedHRVOTest, test_robot_avoiding_ball_obstacle)
{
    Point destination      = Point(0.0, 0);
    Point initial_position = Point(0.0, 0);
    BallState ball_state(Point(0.1, 0), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(0, tactic);
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    setGameState(game_state);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // TODO (#2519): re-enable this test
        // [destination, tactic](std::shared_ptr<World> world_ptr,
        //                       ValidationCoroutine::push_type& yield) {
        //     // Small rectangle around the destination point that the robot should be
        //     // stationary within for 15 ticks
        //      float threshold = 0.05f;
        //      Rectangle expected_final_position(
        //         Point(destination.x() - threshold, destination.y() - threshold),
        //         Point(destination.x() + threshold, destination.y() + threshold));
        //      robotStationaryInPolygon(0, expected_final_position, 15, world_ptr,
        //      yield);
        // }
    };
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(7));
}
