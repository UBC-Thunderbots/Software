/**
 * Tests for the free kick tactic
 */

#include "ai/hl/stp/tactic/free_kick_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/chip_intent.h"
#include "ai/intent/kick_intent.h"
#include "test/test_util/test_util.h"

TEST(FreeKickTacticTest, no_enemies_shoot_towards_goal)
{
    // Robot is in the enemy half facing towards enemy goal
    Robot robot = Robot(2, Point(2, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    Ball ball({2.1, 0}, {0, 0}, Timestamp::fromSeconds(0));
    World world = TestUtil::createBlankTestingWorld();
    world.updateBallState(ball);

    FreeKickTactic tactic(world, false);

    tactic.updateRobot(robot);

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent&>(*tactic.getNextIntent());
        EXPECT_EQ(world.ball().position(), kick_intent.getKickOrigin());
        EXPECT_EQ(2, kick_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE("FreeKickTactic did not return a KickIntent!");
    }
}

TEST(FreeKickTacticTest, no_direct_shot_then_chip)
{
    // Robot is in the enemy half facing towards enemy goal
    Robot robot = Robot(2, Point(2, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    Ball ball({2.1, 0}, {0, 0}, Timestamp::fromSeconds(0));
    World world = TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(2.5, 0)},
        Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    FreeKickTactic tactic(world, false);

    tactic.updateRobot(robot);

    try
    {
        ChipIntent chip_intent = dynamic_cast<ChipIntent&>(*tactic.getNextIntent());
        EXPECT_EQ(2, chip_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE("FreeKickTactic did not return a ChipIntent!");
    }
}