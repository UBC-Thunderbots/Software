#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(GoalieFSMTest, test_transitions)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-4, 0));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
            ::TestUtil::setBallPosition(world, Point(-2, 0), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(-2, 0), Timestamp::fromSeconds(123));

    HFSM<GoalieFSM> fsm;
    // Start in GoalieFSM states' position_to_block_shot_state
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::position_to_block_shot_state>));
}
