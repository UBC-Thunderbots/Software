#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(DribbleFSMTest, test_transitions)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
        ::TestUtil::setBallPosition(world, Point(0.5, 0), Timestamp::fromSeconds(123));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -1), Timestamp::fromSeconds(123));

    HFSM<DribbleFSM> fsm;

    // Start in DribbleState
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::DribbleState>));

    // Stay in DribbleState since ball not in possession yet
    fsm.process_event(DribbleFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::DribbleState>));

    // At ball point so transition to done
    robot = ::TestUtil::createRobotAtPos(Point(0.5, 0));
    fsm.process_event(DribbleFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
