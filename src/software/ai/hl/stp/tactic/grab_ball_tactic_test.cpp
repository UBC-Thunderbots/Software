#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/intent/movespin_intent.h"
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
    auto intent_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(2, 0), 0.05));
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the GrabBallTactic!";
    }
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
    auto intent_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(world.ball().position(), 0.05));
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the GrabBallTactic!";
    }
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
    auto intent_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try
    {
        MoveSpinIntent movespin_intent = dynamic_cast<MoveSpinIntent &>(*intent_ptr);
        EXPECT_TRUE(
            movespin_intent.getDestination().isClose(world.ball().position(), 0.05));
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveSpinIntent was not returned by the GrabBallTactic!";
    }
}
