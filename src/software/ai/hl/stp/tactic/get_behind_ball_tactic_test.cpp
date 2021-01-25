#include "software/ai/hl/stp/tactic/get_behind_ball_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(GetBehindBallFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    GetBehindBallFSM::ControlParams control_params{.ball_location   = Point(2, 3),
                                                   .chick_direction = Angle::quarter()};

    boost::sml::sm<GetBehindBallFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::idle_state>));
    fsm.process_event(GetBehindBallFSM::Update{
        .control_params = control_params,
        .common         = TacticUpdate{.robot      = robot,
                               .world      = world,
                               .set_intent = [](std::unique_ptr<Intent>) {}}});
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));
    // robot behind ball but far away
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(GetBehindBallFSM::Update{
        .control_params = control_params,
        .common         = TacticUpdate{.robot      = robot,
                               .world      = world,
                               .set_intent = [](std::unique_ptr<Intent>) {}}});
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM::get_behind_ball_state>));
    // robot behind ball and close enough
    robot = ::TestUtil::createRobotAtPos(Point(2, 2.8));
    fsm.process_event(GetBehindBallFSM::Update{
        .control_params = control_params,
        .common         = TacticUpdate{.robot      = robot,
                               .world      = world,
                               .set_intent = [](std::unique_ptr<Intent>) {}}});
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
