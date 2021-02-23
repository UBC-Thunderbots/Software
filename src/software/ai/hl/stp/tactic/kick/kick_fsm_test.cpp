#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(KickFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    KickFSM::ControlParams control_params{.kick_origin    = Point(-2, 1.5),
                                          .kick_direction = Angle::threeQuarter(),
                                          .kick_speed_meters_per_second = 1.2};

    HFSM<KickFSM> fsm;

    // Start in GetBehindBallFSM state's get_behind_ball_state
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));

    // Transition to GetBehindBallFSM state's get_behind_ball_state
    fsm.process_event(KickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));

    // Robot is now behind ball
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    fsm.process_event(KickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    // Transition to kick_state
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM::kick_state>));

    // Ball is now kicked
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(Angle::threeQuarter()));

    // Tactic is done
    fsm.process_event(KickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
