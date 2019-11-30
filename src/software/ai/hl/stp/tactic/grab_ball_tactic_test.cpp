#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/hl/stp/action/intercept_ball_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/movespin_action.h"
#include "software/test_util/test_util.h"

TEST(GrabBallTacticTest, test_robot_intercepts_ball_if_no_enemies_near_ball)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(0.5, 0),
                                              Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(2, -1), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    GrabBallTactic tactic =
        GrabBallTactic(world.field(), world.ball(), world.enemyTeam(), true);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto intercept_action = std::dynamic_pointer_cast<InterceptBallAction>(action_ptr);
    ASSERT_FALSE(intercept_action == nullptr);
}

TEST(GrabBallTacticTest,
     test_robot_moves_to_the_ball_if_an_enemy_is_close_but_the_robot_is_far)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world =
        ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.2, 0)},
                                                     Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(2, -1), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    GrabBallTactic tactic =
        GrabBallTactic(world.field(), world.ball(), world.enemyTeam(), true);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto intercept_action = std::dynamic_pointer_cast<InterceptBallAction>(action_ptr);
    ASSERT_FALSE(intercept_action == nullptr);
}

TEST(GrabBallTacticTest, test_robot_steals_the_ball_if_close_to_an_enemy_with_the_ball)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world =
        ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(
        world, {Point(ROBOT_MAX_RADIUS_METERS, 0)}, Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(0, 0.1), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    GrabBallTactic tactic =
        GrabBallTactic(world.field(), world.ball(), world.enemyTeam(), true);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto movespin_action = std::dynamic_pointer_cast<MoveSpinAction>(action_ptr);
    ASSERT_FALSE(movespin_action == nullptr);
    EXPECT_TRUE(movespin_action->getDestination().isClose(world.ball().position(), 0.05));
}
