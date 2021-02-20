#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ChipFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    ChipFSM::ControlParams control_params{.chip_origin          = Point(-2, 1.5),
                                          .chip_direction       = Angle::threeQuarter(),
                                          .chip_distance_meters = 1.2};

    HFSM<ChipFSM> fsm;
    // Start in GetBehindBallFSM state's get_behind_ball_state
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));

    // Transition to GetBehindBallFSM state's get_behind_ball_state
    fsm.process_event(ChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));

    // Robot is now behind ball
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    fsm.process_event(ChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    // Transition to chip_state
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM::chip_state>));

    // Ball is now chipped
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(Angle::threeQuarter()));

    // Tactic is done
    fsm.process_event(ChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
