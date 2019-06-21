/**
 * Tests for the free kick tactic
 */

#include "ai/hl/stp/tactic/free_kick_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/kick_intent.h"
#include "ai/intent/chip_intent.h"

#include "test/test_util/test_util.h"

TEST(FreeKickTacticTest, no_enemies_sanity_test)
{

    // Robot is in the enemy half facing towards enemy goal
    Robot robot = Robot(2, Point(2, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    Ball ball({2.1, 0}, {0, 0}, Timestamp::fromSeconds(0));
    World world = TestUtil::createBlankTestingWorld();

    FreeKickTactic tactic(world, false);

    tactic.updateRobot(robot);

    KickIntent kick_intent = dynamic_cast<KickIntent&>(*tactic.getNextIntent());

    EXPECT_EQ(world.ball().position(), kick_intent.getKickOrigin());
    // EXPECT_EQ(world.field().enemyGoal(), kick_intent.)
}