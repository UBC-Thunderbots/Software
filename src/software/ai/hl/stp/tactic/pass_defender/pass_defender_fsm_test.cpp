#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/test_util.h"

TEST(PassDefenderFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-1, 0));
    PassDefenderFSM::ControlParams control_params{.position_to_block_from = Point(-2, 0)};

    FSM<PassDefenderFSM> fsm{PassDefenderFSM()};

    // Start in BlockPassState
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));

    // Ball has not been kicked towards the pass defender (i.e. enemy has possession)
    // Should stay in BlockPassState
    fsm.process_event(PassDefenderFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));

    // Ball is now kicked towards pass defender
    world =
        ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(Angle::half()));

    // Transition to InterceptBallState
    fsm.process_event(PassDefenderFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::InterceptBallState>));

    // Deflect ball away from pass defender
    world =
        ::TestUtil::setBallPosition(world, Point(-0.5, 0), Timestamp::fromSeconds(124));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 1), Timestamp::fromSeconds(124));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(Angle::quarter()));

    // Transition back to BlockPassState
    fsm.process_event(PassDefenderFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));
}
