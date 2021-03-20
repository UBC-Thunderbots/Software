#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/get_possession/get_possession_fsm.h"
#include "software/test_util/test_util.h"

TEST(GetPossessionFSMTest, test_transitions)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
        ::TestUtil::setBallPosition(world, Point(0.5, 0), Timestamp::fromSeconds(123));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -1), Timestamp::fromSeconds(123));

    HFSM<GetPossessionFSM> fsm;

    // Start in GetPossessionState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetPossessionFSM::GetPossessionState>));

    // Stay in GetPossessionState since ball not in possession yet
    fsm.process_event(GetPossessionFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetPossessionFSM::GetPossessionState>));

    // At ball point so transition to done
    robot = ::TestUtil::createRobotAtPos(Point(0.5, 0));
    fsm.process_event(GetPossessionFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
