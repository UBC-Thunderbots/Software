#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"

#include <gtest/gtest.h>

#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_violating_motion_constraint.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class EnemyBallPlacementPlayTest : public SimulatedPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(EnemyBallPlacementPlayTest, test_ball_placement_center)
{
    Point ball_placement_point(1, 0);
    BallState ball_state(Point(0, 0), Vector(0, 0));
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
                MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(EnemyBallPlacementPlayTest, test_ball_placement_diagonal)
{
    Point ball_placement_point(1, 0);
    BallState ball_state(Point(1.5, 2), Vector(0, 0));
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
                yield("");
            }



            robotsViolatingMotionConstraint(
                world_ptr, yield, obstacle_factory,
                MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
        }

    };

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(EnemyBallPlacementPlayTest, test_ball_placement_moving)
{
    Point ball_placement_point(1, 0);
    BallState ball_state(Point(0, 0), Vector(1, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(0, -1.5), Point(-1, 0), Point(0, 1.5), Point(-1.5, -1.5), Point(-1.5, 0),
         Point(-1.5, 1.5)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1.1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
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
        [this, ball_placement_point](std::shared_ptr<World> world_ptr,
                                     ValidationCoroutine::push_type& yield) {
            ballAtPoint(ball_placement_point, world_ptr, yield);
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
                MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
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
                MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
