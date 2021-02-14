#include "software/ai/hl/stp/tactic/intercept_ball/intercept_ball_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(InterceptBallFSMTest, test_transitions_fast_ball)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
        ::TestUtil::setBallPosition(world, Point(0.5, 0), Timestamp::fromSeconds(123));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -1), Timestamp::fromSeconds(123));

    HFSM<InterceptBallFSM> fsm;

    // Start in chase_ball_state
    EXPECT_TRUE(fsm.is(boost::sml::state<InterceptBallFSM::chase_ball>));

    // Transition to FastBallInterceptFSM to block the ball
    fsm.process_event(InterceptBallFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // At ball point so transition to done
    robot = ::TestUtil::createRobotAtPos(Point(0.5, 0));
    fsm.process_event(InterceptBallFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(InterceptBallFSMTest, test_transitions_slow_ball)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(2, 0), Timestamp::fromSeconds(123));

    HFSM<InterceptBallFSM> fsm;

    // Start in chase_ball_state
    EXPECT_TRUE(fsm.is(boost::sml::state<InterceptBallFSM::chase_ball>));

    // Stay in chase_ball state
    fsm.process_event(InterceptBallFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<InterceptBallFSM::chase_ball>));

    // At interception point so transition to waiting for the ball
    robot = ::TestUtil::createRobotAtPos(Point(2, 0));
    fsm.process_event(InterceptBallFSM::Update(
        {}, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
