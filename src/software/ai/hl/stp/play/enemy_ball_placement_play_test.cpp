#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"

#include <gtest/gtest.h>

#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_violating_motion_constraint.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class EnemyBallPlacementPlayTest
    : public SimulatedErForceSimPlayTestFixture,
      public ::testing::WithParamInterface<std::tuple<std::vector<Point>, BallState>>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_P(EnemyBallPlacementPlayTest, test_ball_placement)
{
    Point ball_placement_point(1, 0);
    BallState ball_state = std::get<1>(GetParam());
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId(std::get<0>(GetParam()));
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(EnemyBallPlacementPlay));
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    game_state.setBallPlacementPoint(ball_placement_point);
    setGameState(game_state);
    std::shared_ptr<RobotNavigationObstacleFactory> obstacle_factory =
        std::make_shared<RobotNavigationObstacleFactory>(
            getAiConfig()->getRobotNavigationObstacleConfig());

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield("Allow robots to start moving");
            }
            robotHalt(world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [obstacle_factory](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                // Empty string in yield means keep going!
                yield("");
            }

            robotsViolatingMotionConstraint(
                world_ptr, yield, obstacle_factory,
                TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(EnemyBallPlacementPlayTest, test_no_placement)
{
    Point ball_placement_point(1, 0);
    BallState ball_state(Point(-1, -2), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(0, -1.5), Point(-1, 0), Point(0, 1.5), Point(-1.5, -1.5), Point(-1.5, 0),
         Point(-1.5, 1.5)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(EnemyBallPlacementPlay));
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    setGameState(game_state);
    std::shared_ptr<RobotNavigationObstacleFactory> obstacle_factory =
        std::make_shared<RobotNavigationObstacleFactory>(
            getAiConfig()->getRobotNavigationObstacleConfig());

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield("Allow robots to start moving");
            }
            robotHalt(world_ptr, yield);
        }};


    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [obstacle_factory](std::shared_ptr<World> world_ptr,
                           ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(2))
            {
                yield("");
            }
            robotsViolatingMotionConstraint(
                world_ptr, yield, obstacle_factory,
                TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    BallAndRobotStates, EnemyBallPlacementPlayTest,
    ::testing::Values(
        // Test ball placement with rectangular motion constraint
        std::make_tuple(std::vector<Point>{Point(0, -1.5), Point(-1, 0), Point(0, 1.5),
                                           Point(-1.5, -1.5), Point(-1.5, 0),
                                           Point(-1.5, 1.5)},
                        BallState(Point(0, 0), Vector(0, 0))),
        // Test ball placement with diagonal motion constraint
        std::make_tuple(std::vector<Point>{Point(0, -1.5), Point(-1, 0), Point(0, 1.5),
                                           Point(-1.5, -1.5), Point(-1.5, 0),
                                           Point(-1.5, 1.5)},
                        BallState(Point(1.5, 2), Vector(0, 0))),
        // Test ball placement with moving ball
        std::make_tuple(std::vector<Point>{Point(0, -1.5), Point(-1, 0), Point(0, 1.5),
                                           Point(-1.5, -1.5), Point(-1.5, 0),
                                           Point(-1.5, 1.5)},
                        BallState(Point(0, 0), Vector(0.90, 0)))));
