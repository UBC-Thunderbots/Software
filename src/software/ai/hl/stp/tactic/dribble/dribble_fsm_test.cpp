#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(DribbleFSMTest, test_transitions)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(0.5, 0), Timestamp::fromSeconds(123));
    ::TestUtil::setBallVelocity(world, Vector(0, -1), Timestamp::fromSeconds(123));

    TbotsProto::DribbleTacticConfig dribble_config;
    FSM<DribbleFSM> fsm{DribbleFSM(dribble_config)};

    // Start in Dribble
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::GetPossession>));

    // Stay in Dribble since ball not in possession yet
    fsm.process_event(DribbleFSM::Update(
        {std::nullopt, std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::GetPossession>));

    // Robot at ball point, so it has possession, so transition to dribble state
    robot = ::TestUtil::createRobotAtPos(Point(0.5, 0));
    fsm.process_event(DribbleFSM::Update(
        {std::nullopt, std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::Dribble>));

    // No dribble destination set, so tactic is done
    fsm.process_event(DribbleFSM::Update(
        {std::nullopt, std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // Set dribble destination, so tactic should be undone
    fsm.process_event(DribbleFSM::Update(
        {Point(1, -1), std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::Dribble>));

    // Move ball to destination, but not the robot, so we should try to regain possession
    ::TestUtil::setBallPosition(world, Point(1, -1), Timestamp::fromSeconds(124));
    fsm.process_event(DribbleFSM::Update(
        {Point(1, -1), std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::GetPossession>));

    // Move robot to where the ball is, so we now have possession and ball is at the
    // destination, but we go to the dribble state before being done
    robot = ::TestUtil::createRobotAtPos(Point(1, -1));
    fsm.process_event(DribbleFSM::Update(
        {Point(1, -1), std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM::Dribble>));

    // Finally FSM is done again
    fsm.process_event(DribbleFSM::Update(
        {Point(1, -1), std::nullopt, false},
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
