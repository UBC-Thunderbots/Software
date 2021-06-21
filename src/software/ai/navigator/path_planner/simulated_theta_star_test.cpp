#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_stationary_in_polygon_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedThetaStarTest : public SimulatedTacticTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(SimulatedThetaStarTest, test_theta_star_robot_and_dest_in_obstacle)
{
    // Both initial_position and destination are in obstacle.
    // The two points are not in the same coordinated initially, however after
    // initial_position is moved to a free point, it is in the same coordinate as
    // destination. Values are chosen from a case which previously did not find a path.
    Point destination      = Point(1, 0);
    Point initial_position = Point(0.96905413818359376, -0.26277551269531252);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            Rectangle expected_final_position(Point(0.9, -0.1), Point(1.1, 0.1));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest, test_theta_star_dest_in_obstacle)
{
    // Destination is in obstacle, but initial point is open
    Point destination      = Point(1, -0.1);
    Point initial_position = Point(1, 2);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            Rectangle expected_final_position(Point(0.9, -0.2), Point(1.1, 0.0));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}


TEST_F(SimulatedThetaStarTest,
       test_theta_star_not_stuck_when_start_point_and_first_grid_point_is_close)
{
    // Destination is in obstacle, but initial point is open
    Point destination = Point(4, 2.4);

    Point initial_position = Point(3.21118, 1.06699);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    setMotionConstraints({MotionConstraint::ENEMY_DEFENSE_AREA});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            Rectangle expected_final_position(Point(3.9, 2.3), Point(4.1, 2.5));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest, test_theta_star_robot_in_obstacle)
{
    // Destination is in a free point, but initial point is in an obstacle
    Point destination      = Point(0, 0);
    Point initial_position = Point(1, 0);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            Rectangle expected_final_position(Point(0.03, 0.03), Point(-0.03, -0.03));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest, test_theta_no_obstacle_straight_path)
{
    Point destination      = Point(-2, 1);
    Point initial_position = Point(2, 1);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            Rectangle expected_final_position(Point(-2.04, 1.04), Point(-1.96, 0.96));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest, test_theta_star_zig_zag_test)
{
    /* enemy robots placed so the friendly robot is forced to take a path which
     * zig-zags. (inspired by the 2021 SSL dribbling hardware challenge)
     * https://robocup-ssl.github.io/ssl-hardware-challenge-rules/rules.html#_challenge_3_dribbling
     */

    // The x value of the wall in front of the friendly robot
    int front_wall_x = -1;
    // each gate refers to the center to center distance between each wall and the front
    // wall The constant offsets can be tweaked to get different distances between each
    // wall
    int gate_1 = 1;
    int gate_2 = gate_1 + 2;
    int gate_3 = gate_2 + 1;

    Point destination      = Point(front_wall_x + gate_3 + 0.5, 0);
    Point initial_position = Point(front_wall_x - 0.5, 0);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(front_wall_x, 0.0), Point(front_wall_x, 0.5), Point(front_wall_x, 1),
         Point(front_wall_x + gate_1, 0.0), Point(front_wall_x + gate_1, -0.5),
         Point(front_wall_x + gate_1, -1), Point(front_wall_x + gate_2, 0.0),
         Point(front_wall_x + gate_2, 0.5), Point(front_wall_x + gate_2, 1),
         Point(front_wall_x + gate_3, 0.0), Point(front_wall_x + gate_3, -0.5),
         Point(front_wall_x + gate_3, -1)});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            Rectangle expected_final_position(
                Point(destination.x() + 0.04, destination.y() + 0.04),
                Point(destination.x() - 0.04, destination.y() - 0.04));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest, test_theta_star_oscillation)
{
    /*
     * When DISTANCE_THRESHOLD (what determines if robot is close enough to destination)
     * in transition_condition.h is lowered (eg. to 0.02), the robot oscillates back
     * and forth and does not reach a steady state at the destination in some cases.
     * This test is for visual purposes to check for this bug.
     * Known destinations that oscillate with DISTANCE_THRESHOLD = 0.02:
     *  - (-2.5,-2.5)
     *  - (2.5,-2.5)
     */
    Point destination      = Point(-2.5, -2.5);
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle around the destination point that the robot should be
            // stationary within for 15 ticks
            Rectangle expected_final_position(Point(-2.55, -2.55), Point(-2.45, -2.45));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}
