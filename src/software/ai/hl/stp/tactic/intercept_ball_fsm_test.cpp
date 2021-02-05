#include "software/ai/hl/stp/tactic/intercept_ball_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(InterceptBallFSMTest, test_transitions)
{
    //    World world = ::TestUtil::createBlankTestingWorld();
    //    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    //    InterceptBallFSM::ControlParams control_params{.destination       = Point(2, 3),
    //                                          .final_orientation = Angle::half(),
    //                                          .final_speed       = 0.0};
    //
    //    BaseFSM<InterceptBallFSM> fsm;
    //    EXPECT_TRUE(fsm.is(boost::sml::state<InterceptBallFSM::idle_state>));
    //    fsm.process_event(InterceptBallFSM::Update(
    //        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>)
    //        {})));
    //    // robot far from destination
    //    EXPECT_TRUE(fsm.is(boost::sml::state<InterceptBallFSM::move_state>));
    //    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    //    fsm.process_event(InterceptBallFSM::Update(
    //        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>)
    //        {})));
    //    // robot close to destination
    //    EXPECT_TRUE(fsm.is(boost::sml::state<InterceptBallFSM::move_state>));
    //    robot.updateState(
    //        RobotState(Point(2, 3), Vector(), Angle::half(), AngularVelocity::zero()),
    //        Timestamp::fromSeconds(0));
    //    fsm.process_event(InterceptBallFSM::Update(
    //        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>)
    //        {})));
    //    // robot at destination and facing the right way
    //    EXPECT_TRUE(fsm.is(boost::sml::X));
}
