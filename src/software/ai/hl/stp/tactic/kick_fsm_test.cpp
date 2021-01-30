#include "software/ai/hl/stp/tactic/kick_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(KickFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    KickFSM::ControlParams control_params{.kick_origin    = Point(-2, 1.5),
                                          .kick_direction = Angle::threeQuarter(),
                                          .kick_speed_meters_per_second = 1.2};

    boost::sml::sm<KickFSM, boost::sml::process_queue<std::queue>> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::idle_state>));
    fsm.process_event(KickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    fsm.process_event(KickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM::kick_state>));
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(Angle::threeQuarter()));
    fsm.process_event(KickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
