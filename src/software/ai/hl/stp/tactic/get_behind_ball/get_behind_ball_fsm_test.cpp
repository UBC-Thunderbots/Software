#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(GetBehindBallFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    GetBehindBallFSM::ControlParams control_params{.ball_location   = Point(2, 3),
                                                   .chick_direction = Angle::quarter()};

    BaseFSM<GetBehindBallFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));
    fsm.process_event(GetBehindBallFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));

    // robot behind ball but far away
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(GetBehindBallFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));

    // robot behind ball and close enough
    robot = ::TestUtil::createRobotAtPos(Point(2, 2.8));
    fsm.process_event(GetBehindBallFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // destination updated so robot needs to move to new destination
    control_params = GetBehindBallFSM::ControlParams{.ball_location   = Point(-2, 1),
                                                     .chick_direction = Angle::quarter()};
    fsm.process_event(GetBehindBallFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));
}
