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
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::GetPossessionState>));

    // Stay in DribbleState since ball not in possession yet
    fsm.process_event(DribbleFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::GetPossessionState>));

    // At ball point so transition to done
    robot = ::TestUtil::createRobotAtPos(Point(0.5, 0));
    fsm.process_event(DribbleFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::DribbleState>));

    // No ball destination set, so tactic is done
    fsm.process_event(DribbleFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // Set ball destination, so tactic should be undone
    fsm.process_event(
        DribbleFSM::Update({Point(1, -1), std::nullopt},
                           TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::DribbleState>));

    // Move ball to destination, but not the robot, so we should try to regain possession
    world = ::TestUtil::setBallPosition(world, Point(1, -1), Timestamp::fromSeconds(124));
    fsm.process_event(
        DribbleFSM::Update({Point(1, -1), std::nullopt},
                           TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::GetPossessionState>));

    // Move robot to where the ball is, so we now have possession and ball is at the
    // destination, but we go to the dribble state before being done
    robot = ::TestUtil::createRobotAtPos(Point(1, -1));
    fsm.process_event(
        DribbleFSM::Update({Point(1, -1), std::nullopt},
                           TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::DribbleState>));

    // Finally FSM is done again
    fsm.process_event(
        DribbleFSM::Update({Point(1, -1), std::nullopt},
                           TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
