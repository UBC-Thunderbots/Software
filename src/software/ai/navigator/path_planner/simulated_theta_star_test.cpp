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

TEST_F(SimulatedThetaStarTest, DISABLED_test_theta_star_robot_and_dest_in_same_obstacle)
{
    // TODO (ISSUE #2227): Enemy robots obstacle size changing when friendly robot is in
    // close proximity This issue results in the robot constantly oscillating and not
    // settling down at the destination.
    // https://github.com/UBC-Thunderbots/Software/issues/2227

    // Both initial_position and destination are in the same obstacle (enemy robot).
    // The two points are not in the same coordinate initially, however after
    // initial_position is moved to a free point, it is in the same coordinate as
    // destination. Values are chosen from a case which previously did not find a path.
    Point destination      = Point(1, 0);
    Point initial_position = Point(0.96905413818359376, -0.26277551269531252);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest,
       DISABLED_test_theta_star_robot_and_dest_in_different_obstacle)
{
    // TODO (ISSUE #2227): Enemy robots obstacle size changing when friendly robot is in
    // close proximity This issue results in the robot constantly oscillating and not
    // settling down at the destination.
    // https://github.com/UBC-Thunderbots/Software/issues/2227

    // Both initial_position and destination are in obstacle.
    // The two points are placed in different obstacles (enemy robots)
    Point destination      = Point(3, 0);
    Point initial_position = Point(0.96905413818359376, -0.26277551269531252);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(1, 0), Point(3, 0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
            robotStationaryInPolygon(1, expected_final_position, 15, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest, DISABLED_test_theta_star_dest_in_obstacle)
{
    // TODO (ISSUE #2227): Enemy robots obstacle size changing when friendly robot is in
    // close proximity This issue results in the robot constantly oscillating and not
    // settling down at the destination.
    // https://github.com/UBC-Thunderbots/Software/issues/2227

    // Destination is in obstacle, but initial point is open
    Point destination      = Point(1, -0.1);
    Point initial_position = Point(1, 2);
    BallState ball_state(Point(0, -0.6), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(1, 0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
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

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    setMotionConstraints({MotionConstraint::ENEMY_DEFENSE_AREA});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle near the destination point that is outside of the obstacle
            // which the friendly robot should be stationary in for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
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

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            // Small rectangle the size of a grid square around the destination point that
            // the robot should be stationary within for 15 ticks
            float threshold = 0.05f;
            Rectangle expected_final_position(
                Point(destination.x() - threshold, destination.y() - threshold),
                Point(destination.x() + threshold, destination.y() + threshold));
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

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setFriendlyTactic(tactic);
    setFriendlyRobotId(1);

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

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

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

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

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

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest,
       test_theta_star_find_path_through_enemy_half_when_it_is_an_obstacle)
{
    Point destination      = Point(-2.5, -2.5);
    Point initial_position = field.enemyGoalCenter();
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {field.friendlyGoalCenter(), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(2, 3), Point(3, -1), Point(4, 2), Point(2, 3.2),
         Point(-1.7, 3.8)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    std::set<MotionConstraint> motion_constraints = {MotionConstraint::ENEMY_HALF};
    setMotionConstraints(motion_constraints);

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

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarTest,
       test_theta_star_find_path_through_inflated_enemy_defense_area)
{
    // Start in the inflated enemy defense area (at the bottom), and navigate to the top
    // of the inflated enemy defense area.
    Point destination      = Point(4.4, 1.5);
    Point initial_position = Point(4.4, -1.3);
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {field.friendlyGoalCenter(), initial_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(0, 0)});

    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setFriendlyRobotId(1);

    std::set<MotionConstraint> motion_constraints = {
        MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA};
    setMotionConstraints(motion_constraints);

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

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}
